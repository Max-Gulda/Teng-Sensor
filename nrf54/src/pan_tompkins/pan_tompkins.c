#include "pan_tompkins.h"

#include "define.h"
#include "error_handling.h"
#include "heart_rate_finder.h"
#include "hr_filter.h"
#include "bw_filter.h"
#include "heart_rate_tuning_params.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pan_tompkins, LOG_LEVEL_INF);

#ifndef PT_HR_HISTORY_LEN
#define PT_HR_HISTORY_LEN            16u
#endif
#ifndef PT_BACKSEARCH_PEAK_WEIGHT
#define PT_BACKSEARCH_PEAK_WEIGHT    0.25f
#endif
#ifndef PT_TWAVE_SLOPE_WINDOW
#define PT_TWAVE_SLOPE_WINDOW        10u
#endif
#ifndef PT_THRESHOLD_WARMUP_SEC
#define PT_THRESHOLD_WARMUP_SEC      2u
#endif
#ifndef PT_FILTERED_REFINE_RADIUS
#define PT_FILTERED_REFINE_RADIUS    2u
#endif
#ifndef PT_PENDING_EVENTS_MAX
#define PT_PENDING_EVENTS_MAX        64u
#endif

#if PT_ARTIFACT_REF_HISTORY_LEN > PT_HR_RECENT_MAX_INTERVALS
#define PT_MEDIAN_F32_MAX_COUNT      PT_ARTIFACT_REF_HISTORY_LEN
#else
#define PT_MEDIAN_F32_MAX_COUNT      PT_HR_RECENT_MAX_INTERVALS
#endif

#define PT_EVENT_NONE  0u
#define PT_EVENT_QRS   1u
#define PT_EVENT_NOISE 2u

typedef struct {
    uint32_t sample;
    uint32_t slope;
    float peak_i;
    float peak_f;
    uint8_t kind;
    uint8_t invalid;
} pt_pending_event_t;

typedef struct {
    uint32_t left_sample;
    uint32_t right_sample;
    uint32_t rr_samples;
    uint8_t rejected;
} pt_rr_history_entry_t;

typedef struct {
    uint32_t sample_rate_hz;
    uint16_t window_size;
    uint16_t buffer_len;

    uint32_t sample_count;
    uint16_t filled;
    uint16_t current_index;

    uint8_t  have_qrs;
    uint16_t last_rr_ms;

    uint32_t last_qrs_sample;
    uint32_t last_slope;
    uint8_t  det_have_qrs;
    uint32_t det_last_qrs_sample;
    uint32_t det_last_slope;
    uint32_t artifact_guard_until_sample;
    uint32_t last_artifact_sample;

    uint32_t rr1[PT_RR_AVG_LEN];
    uint32_t rr2[PT_RR_AVG_LEN];
    pt_rr_history_entry_t rr_history[PT_HR_RECENT_MAX_INTERVALS];
    uint32_t rravg1;
    uint32_t rravg2;
    uint32_t rrlow;
    uint32_t rrhigh;
    uint32_t rrmiss;
    uint8_t  rr_history_count;
    uint8_t  rr_seeded;
    uint8_t  thresholds_ready;
    int8_t   signal_polarity;
    uint32_t warmup_max_i;
    uint64_t warmup_sum_i;
    float    warmup_max_f;
    float    warmup_max_f_pos;
    float    warmup_max_f_neg;
    float    warmup_sum_f;
    uint32_t warmup_count;

    bool regular;

    float threshold_i1;
    float threshold_i2;
    float threshold_f1;
    float threshold_f2;
    float spk_i;
    float spk_f;
    float npk_i;
    float npk_f;
    float qrs_ref_i[PT_ARTIFACT_REF_HISTORY_LEN];
    float qrs_ref_f[PT_ARTIFACT_REF_HISTORY_LEN];
    uint8_t qrs_ref_count;
    uint8_t rejected_rr_count;
    uint32_t rejected_rr_left[PT_DEBUG_MAX_REJECTED_RR];
    uint32_t rejected_rr_right[PT_DEBUG_MAX_REJECTED_RR];
    pt_pending_event_t pending[PT_PENDING_EVENTS_MAX];
    uint8_t pending_count;
    uint8_t bp_order;
    float low_cutoff_hz;
    float high_cutoff_hz;
    BWBandPass bandpass;

    int32_t signal[PAN_TOMPKINS_SIZE];
    int32_t dcblock[PAN_TOMPKINS_SIZE];
    int32_t lowpass[PAN_TOMPKINS_SIZE];
    int32_t highpass[PAN_TOMPKINS_SIZE];
    int32_t derivative[PAN_TOMPKINS_SIZE];
    int32_t squared[PAN_TOMPKINS_SIZE];
    int32_t integral[PAN_TOMPKINS_SIZE];
} pt_state_t;

static pt_state_t g_pt_state;

static int32_t clamp_i64_to_i32(int64_t value) {
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t) value;
}

static uint32_t pt_hard_refractory_samples(const pt_state_t *state) {
    return (uint32_t) ((float) state->sample_rate_hz * PT_HARD_REFRACTORY_SEC);
}

static uint32_t pt_soft_refractory_samples(const pt_state_t *state) {
    return (uint32_t) ((float) state->sample_rate_hz * PT_SOFT_REFRACTORY_SEC);
}

static void pt_refresh_thresholds(pt_state_t *state) {
    state->threshold_i1 = state->npk_i + PT_THRESHOLD_SCALE * (state->spk_i - state->npk_i);
    state->threshold_i2 = PT_THRESHOLD2_SCALE * state->threshold_i1;
    state->threshold_f1 = state->npk_f + PT_THRESHOLD_SCALE * (state->spk_f - state->npk_f);
    state->threshold_f2 = PT_THRESHOLD2_SCALE * state->threshold_f1;
}

static void pt_update_noise_thresholds(pt_state_t *state, float peak_i, float peak_f) {
    state->npk_i = PT_NOISE_PEAK_WEIGHT * peak_i + (1.0f - PT_NOISE_PEAK_WEIGHT) * state->npk_i;
    state->npk_f = PT_NOISE_PEAK_WEIGHT * peak_f + (1.0f - PT_NOISE_PEAK_WEIGHT) * state->npk_f;
    pt_refresh_thresholds(state);
}

static void pt_update_signal_thresholds(pt_state_t *state, float peak_i, float peak_f, float weight) {
    state->spk_i = weight * peak_i + (1.0f - weight) * state->spk_i;
    state->spk_f = weight * peak_f + (1.0f - weight) * state->spk_f;
    pt_refresh_thresholds(state);
}

static uint32_t pt_compute_slope(const pt_state_t *state, uint16_t end_index) {
    uint16_t start = (end_index > PT_TWAVE_SLOPE_WINDOW) ? (uint16_t) (end_index - PT_TWAVE_SLOPE_WINDOW) : 0u;
    uint32_t max_slope = 0u;

    for (uint16_t i = start; i <= end_index; i++) {
        uint32_t sample = (state->squared[i] > 0) ? (uint32_t) state->squared[i] : 0u;
        if (sample > max_slope) {
            max_slope = sample;
        }
    }

    return max_slope;
}

static uint8_t pt_is_local_peak(const int32_t *signal, uint16_t index, uint16_t valid_len) {
    if (!signal || index == 0u || (index + 1u) >= valid_len) {
        return 0u;
    }

    return signal[index] >= signal[index - 1u] && signal[index] > signal[index + 1u];
}

static uint16_t pt_find_filtered_peak(const pt_state_t *state, uint16_t integrated_peak_index) {
    uint16_t start;
    uint16_t best_index;
    float best_value;

    if (integrated_peak_index > state->window_size) {
        start = (uint16_t) (integrated_peak_index - state->window_size);
    } else {
        start = 0u;
    }

    best_index = start;
    best_value = (float) state->signal_polarity * (float) state->highpass[start];
    for (uint16_t idx = (uint16_t) (start + 1u); idx <= integrated_peak_index; idx++) {
        float value = (float) state->signal_polarity * (float) state->highpass[idx];
        if (value > best_value) {
            best_value = value;
            best_index = idx;
        }
    }

    start = (best_index > PT_FILTERED_REFINE_RADIUS) ? (uint16_t) (best_index - PT_FILTERED_REFINE_RADIUS) : 0u;
    {
        uint16_t end = best_index + PT_FILTERED_REFINE_RADIUS;
        if (end > integrated_peak_index) {
            end = integrated_peak_index;
        }
        for (uint16_t idx = start; idx <= end; idx++) {
            float value = (float) state->signal_polarity * (float) state->highpass[idx];
            if (value > best_value) {
                best_value = value;
                best_index = idx;
            }
        }
    }

    return best_index;
}

static uint16_t pt_samples_to_rr_ms(const pt_state_t *state, uint32_t rr_samples) {
    if (state->sample_rate_hz == 0u) {
        return 0u;
    }

    uint32_t rr_ms = (rr_samples * 1000u) / state->sample_rate_hz;
    if (rr_ms > UINT16_MAX) {
        return UINT16_MAX;
    }
    return (uint16_t) rr_ms;
}

static void pt_sort_u32(uint32_t *values, uint8_t count) {
    for (uint8_t i = 1u; i < count; i++) {
        uint32_t key = values[i];
        uint8_t j = i;

        while (j > 0u && values[j - 1u] > key) {
            values[j] = values[j - 1u];
            j--;
        }
        values[j] = key;
    }
}

static uint32_t pt_median_u32(uint32_t *values, uint8_t count) {
    pt_sort_u32(values, count);

    if ((count & 1u) != 0u) {
        return values[count / 2u];
    }

    return (values[(count / 2u) - 1u] + values[count / 2u]) / 2u;
}

static void pt_sort_f32(float *values, uint8_t count) {
    for (uint8_t i = 1u; i < count; i++) {
        float key = values[i];
        uint8_t j = i;

        while (j > 0u && values[j - 1u] > key) {
            values[j] = values[j - 1u];
            j--;
        }
        values[j] = key;
    }
}

static float pt_median_f32(const float *values, uint8_t count) {
    float sorted[PT_MEDIAN_F32_MAX_COUNT];

    if (!values || count == 0u || count > PT_MEDIAN_F32_MAX_COUNT) {
        return 0.0f;
    }

    for (uint8_t i = 0u; i < count; i++) {
        sorted[i] = values[i];
    }
    pt_sort_f32(sorted, count);

    if ((count & 1u) != 0u) {
        return sorted[count / 2u];
    }

    return 0.5f * (sorted[(count / 2u) - 1u] + sorted[count / 2u]);
}

static uint32_t pt_artifact_guard_samples(const pt_state_t *state) {
    return state->sample_rate_hz * PT_ARTIFACT_GUARD_SEC;
}

static void pt_get_artifact_thresholds(const pt_state_t *state, float *artifact_f, float *artifact_i) {
    float ref_i;
    float ref_f;

    if (artifact_f) {
        *artifact_f = NAN;
    }
    if (artifact_i) {
        *artifact_i = NAN;
    }

    if (!state || state->qrs_ref_count < PT_ARTIFACT_MIN_REF_COUNT) {
        return;
    }

    ref_i = pt_median_f32(state->qrs_ref_i, state->qrs_ref_count);
    ref_f = pt_median_f32(state->qrs_ref_f, state->qrs_ref_count);
    if (ref_i <= 0.0f || ref_f <= 0.0f) {
        return;
    }

    if (artifact_f) {
        *artifact_f = ref_f * PT_ARTIFACT_SCALE_F;
    }
    if (artifact_i) {
        *artifact_i = ref_i * PT_ARTIFACT_SCALE_I;
    }
}

static float pt_abs_f32(float value) {
    return (value >= 0.0f) ? value : -value;
}

static uint32_t pt_max_u32(uint32_t a, uint32_t b) {
    return (a > b) ? a : b;
}

static uint32_t pt_min_u32(uint32_t a, uint32_t b) {
    return (a < b) ? a : b;
}

static uint32_t pt_rr_history_window_samples(const pt_state_t *state) {
    return state->sample_rate_hz * PT_HR_RECENT_WINDOW_SEC;
}

static void pt_rr_history_prune(pt_state_t *state, uint32_t current_sample) {
    uint32_t window_samples;

    if (!state || state->rr_history_count == 0u) {
        return;
    }

    window_samples = pt_rr_history_window_samples(state);
    while (state->rr_history_count > 0u) {
        pt_rr_history_entry_t *entry = &state->rr_history[0];

        if (current_sample < entry->right_sample) {
            break;
        }
        if ((current_sample - entry->right_sample) <= window_samples) {
            break;
        }

        if (state->rr_history_count > 1u) {
            memmove(&state->rr_history[0],
                &state->rr_history[1],
                (size_t) (state->rr_history_count - 1u) * sizeof(state->rr_history[0]));
        }
        state->rr_history_count--;
    }
}

static void pt_rr_history_push(pt_state_t *state,
    uint32_t left_sample,
    uint32_t right_sample,
    uint32_t rr_samples,
    uint8_t rejected)
{
    pt_rr_history_entry_t *entry;

    if (!state) {
        return;
    }

    if (state->rr_history_count >= PT_HR_RECENT_MAX_INTERVALS) {
        memmove(&state->rr_history[0],
            &state->rr_history[1],
            (PT_HR_RECENT_MAX_INTERVALS - 1u) * sizeof(state->rr_history[0]));
        state->rr_history_count = PT_HR_RECENT_MAX_INTERVALS - 1u;
    }

    entry = &state->rr_history[state->rr_history_count++];
    entry->left_sample = left_sample;
    entry->right_sample = right_sample;
    entry->rr_samples = rr_samples;
    entry->rejected = rejected ? 1u : 0u;
}

static uint8_t pt_rr_history_collect_accepted(const pt_state_t *state, uint32_t *rr_values) {
    uint8_t count = 0u;

    if (!state || !rr_values) {
        return 0u;
    }

    for (uint8_t i = 0u; i < state->rr_history_count; i++) {
        if (state->rr_history[i].rejected) {
            continue;
        }
        rr_values[count++] = state->rr_history[i].rr_samples;
    }

    return count;
}

static uint8_t pt_rr_history_collect_rejected(const pt_state_t *state,
    uint32_t *left_values,
    uint32_t *right_values)
{
    uint8_t count = 0u;

    if (!state || !left_values || !right_values) {
        return 0u;
    }

    for (uint8_t i = 0u; i < state->rr_history_count; i++) {
        if (!state->rr_history[i].rejected) {
            continue;
        }
        if (count >= PT_DEBUG_MAX_REJECTED_RR) {
            break;
        }
        left_values[count] = state->rr_history[i].left_sample;
        right_values[count] = state->rr_history[i].right_sample;
        count++;
    }

    return count;
}

static uint8_t pt_rr_history_should_accept_new(const pt_state_t *state, uint32_t rr_samples) {
    uint32_t accepted_rr[PT_HR_RECENT_MAX_INTERVALS];
    float abs_dev[PT_HR_RECENT_MAX_INTERVALS];
    float sorted_abs_dev[PT_HR_RECENT_MAX_INTERVALS];
    uint32_t median_rr;
    float mad;
    uint8_t accepted_count;
    float ratio_dev;

    if (!state || rr_samples == 0u) {
        return 1u;
    }

    accepted_count = pt_rr_history_collect_accepted(state, accepted_rr);
    if (accepted_count < PT_RR_FILTER_MIN_INTERVALS) {
        return 1u;
    }

    median_rr = pt_median_u32(accepted_rr, accepted_count);
    if (median_rr == 0u) {
        return 1u;
    }

    for (uint8_t i = 0u; i < accepted_count; i++) {
        abs_dev[i] = pt_abs_f32((float) accepted_rr[i] - (float) median_rr);
        sorted_abs_dev[i] = abs_dev[i];
    }
    mad = pt_median_f32(sorted_abs_dev, accepted_count);
    ratio_dev = pt_abs_f32((float) rr_samples - (float) median_rr) / (float) median_rr;

    if (mad > 0.0f) {
        float robust_z = 0.6745f * pt_abs_f32((float) rr_samples - (float) median_rr) / mad;
        if (robust_z <= PT_RR_FILTER_ROBUST_ZMAX && ratio_dev <= PT_RR_FILTER_CORE_RATIO_TOL) {
            return 1u;
        }
        return (ratio_dev <= PT_RR_FILTER_RELAXED_RATIO_TOL) ? 1u : 0u;
    }

    if (ratio_dev <= PT_RR_FILTER_CORE_RATIO_TOL) {
        return 1u;
    }
    return (ratio_dev <= PT_RR_FILTER_RELAXED_RATIO_TOL) ? 1u : 0u;
}

static uint32_t pt_select_recent_rr_for_hr(pt_state_t *state, uint32_t current_sample) {
    uint32_t accepted_rr[PT_HR_RECENT_MAX_INTERVALS];
    uint8_t accepted_count;

    if (!state) {
        return 0u;
    }

    pt_rr_history_prune(state, current_sample);
    accepted_count = pt_rr_history_collect_accepted(state, accepted_rr);
    if (accepted_count == 0u) {
        return 0u;
    }

    return pt_median_u32(accepted_rr, accepted_count);
}

static uint8_t pt_filter_hr_from_rr_samples(const pt_state_t *state, uint32_t rr_samples) {
    uint16_t peaks[2];

    if (rr_samples == 0u || rr_samples > UINT16_MAX) {
        return 0u;
    }

    peaks[0] = 0u;
    peaks[1] = (uint16_t) rr_samples;
    return calculate_heart_from_peaks(peaks, 2u, (uint16_t) state->sample_rate_hz);
}

static uint8_t pt_threshold_warmup_active(const pt_state_t *state) {
    uint32_t warmup_samples = state->sample_rate_hz * PT_THRESHOLD_WARMUP_SEC;

    if (warmup_samples == 0u) {
        warmup_samples = state->sample_rate_hz;
    }

    return state->sample_count < warmup_samples;
}

static void pt_threshold_warmup_update(pt_state_t *state, uint32_t integral_sample, float highpass_sample) {
    float magnitude = (highpass_sample >= 0.0f) ? highpass_sample : -highpass_sample;

    if (integral_sample > state->warmup_max_i) {
        state->warmup_max_i = integral_sample;
    }
    state->warmup_sum_i += (uint64_t) integral_sample;
    state->warmup_sum_f += magnitude;
    state->warmup_count++;
    if (highpass_sample >= 0.0f) {
        if (highpass_sample > state->warmup_max_f_pos) {
            state->warmup_max_f_pos = highpass_sample;
        }
    } else {
        if (magnitude > state->warmup_max_f_neg) {
            state->warmup_max_f_neg = magnitude;
        }
    }
}

static void pt_threshold_warmup_finish(pt_state_t *state) {
    float avg_i;
    float avg_f;

    if (state->warmup_max_i == 0u) {
        state->warmup_max_i = 1u;
    }
    if (state->warmup_count == 0u) {
        state->warmup_count = 1u;
    }

    if (state->warmup_max_f_pos >= state->warmup_max_f_neg) {
        state->signal_polarity = 1;
        state->warmup_max_f = state->warmup_max_f_pos;
    } else {
        state->signal_polarity = -1;
        state->warmup_max_f = state->warmup_max_f_neg;
    }
    if (state->warmup_max_f <= 0.0f) {
        state->warmup_max_f = 1.0f;
    }

    avg_i = (float) ((double) state->warmup_sum_i / (double) state->warmup_count);
    avg_f = state->warmup_sum_f / (float) state->warmup_count;
    if (avg_i <= 0.0f) {
        avg_i = 1.0f;
    }
    if (avg_f <= 0.0f) {
        avg_f = 1.0f;
    }

    state->spk_i = 0.25f * (float) state->warmup_max_i;
    state->spk_f = 0.25f * state->warmup_max_f;
    state->npk_i = 0.5f * avg_i;
    state->npk_f = 0.5f * avg_f;
    if (state->spk_i < state->npk_i) {
        state->spk_i = state->npk_i;
    }
    if (state->spk_f < state->npk_f) {
        state->spk_f = state->npk_f;
    }
    pt_refresh_thresholds(state);
    state->thresholds_ready = 1u;

    LOG_INF("Pan-Tompkins thresholds initialized: i1=%u f1=%u polarity=%d",
        (uint32_t) state->threshold_i1, (uint32_t) state->threshold_f1, state->signal_polarity);
}

static void pt_record_qrs_reference(pt_state_t *state, float peak_i, float peak_f) {
    uint8_t idx;

    if (state->qrs_ref_count < PT_ARTIFACT_REF_HISTORY_LEN) {
        idx = state->qrs_ref_count;
        state->qrs_ref_count++;
    } else {
        memmove(&state->qrs_ref_i[0], &state->qrs_ref_i[1], (PT_ARTIFACT_REF_HISTORY_LEN - 1u) * sizeof(state->qrs_ref_i[0]));
        memmove(&state->qrs_ref_f[0], &state->qrs_ref_f[1], (PT_ARTIFACT_REF_HISTORY_LEN - 1u) * sizeof(state->qrs_ref_f[0]));
        idx = PT_ARTIFACT_REF_HISTORY_LEN - 1u;
    }

    state->qrs_ref_i[idx] = peak_i;
    state->qrs_ref_f[idx] = peak_f;
}

static uint8_t pt_is_artifact_candidate(const pt_state_t *state, float peak_i, float peak_f) {
    float ref_i;
    float ref_f;

    if (state->qrs_ref_count < PT_ARTIFACT_MIN_REF_COUNT) {
        return 0u;
    }

    ref_i = pt_median_f32(state->qrs_ref_i, state->qrs_ref_count);
    ref_f = pt_median_f32(state->qrs_ref_f, state->qrs_ref_count);
    if (ref_i <= 0.0f || ref_f <= 0.0f) {
        return 0u;
    }

    return (peak_i >= (ref_i * PT_ARTIFACT_SCALE_I) && peak_f >= (ref_f * PT_ARTIFACT_SCALE_F)) ? 1u : 0u;
}

static void pt_pending_push(pt_state_t *state,
    uint32_t sample,
    uint32_t slope,
    float peak_i,
    float peak_f,
    uint8_t kind) {
    pt_pending_event_t *event;

    if (state->pending_count >= PT_PENDING_EVENTS_MAX) {
        memmove(&state->pending[0], &state->pending[1], (PT_PENDING_EVENTS_MAX - 1u) * sizeof(state->pending[0]));
        state->pending_count = PT_PENDING_EVENTS_MAX - 1u;
    }

    event = &state->pending[state->pending_count++];
    event->sample = sample;
    event->slope = slope;
    event->peak_i = peak_i;
    event->peak_f = peak_f;
    event->kind = kind;
    event->invalid = 0u;
}

static void pt_invalidate_pending_window(pt_state_t *state, uint32_t center_sample) {
    uint32_t guard = pt_artifact_guard_samples(state);
    uint32_t start = (center_sample > guard) ? (center_sample - guard) : 0u;
    uint32_t end = center_sample + guard;

    for (uint8_t i = 0u; i < state->pending_count; i++) {
        if (state->pending[i].sample >= start && state->pending[i].sample <= end) {
            state->pending[i].invalid = 1u;
        }
    }
}

static void pt_record_artifact(pt_state_t *state, uint32_t artifact_sample) {
    uint32_t artifact_until = artifact_sample + pt_artifact_guard_samples(state);

    pt_invalidate_pending_window(state, artifact_sample);
    state->last_artifact_sample = artifact_sample;
    if (artifact_until > state->artifact_guard_until_sample) {
        state->artifact_guard_until_sample = artifact_until;
    }
}

static void pt_shift_left_i32(int32_t *buffer, uint16_t len) {
    memmove(buffer, &buffer[1], (size_t) (len - 1u) * sizeof(*buffer));
}

static uint16_t pt_push_sample(pt_state_t *state, int32_t raw_sample) {
    uint16_t current;

    if (state->filled >= state->buffer_len) {
        pt_shift_left_i32(state->signal, state->buffer_len);
        pt_shift_left_i32(state->dcblock, state->buffer_len);
        pt_shift_left_i32(state->lowpass, state->buffer_len);
        pt_shift_left_i32(state->highpass, state->buffer_len);
        pt_shift_left_i32(state->derivative, state->buffer_len);
        pt_shift_left_i32(state->squared, state->buffer_len);
        pt_shift_left_i32(state->integral, state->buffer_len);
        current = (uint16_t) (state->buffer_len - 1u);
    } else {
        current = state->filled;
        state->filled++;
    }

    state->signal[current] = raw_sample;
    state->sample_count++;
    state->current_index = current;
    return current;
}

static uint8_t pt_register_qrs(pt_state_t *state,
    uint32_t detected_sample,
    uint32_t current_sample,
    uint32_t current_slope,
    float peak_i,
    float peak_f,
    float spk_weight) {
    bool prev_regular = state->regular;
    bool had_previous_qrs = state->have_qrs;
    uint32_t prev_qrs_sample = state->last_qrs_sample;
    uint32_t rr_samples = detected_sample - state->last_qrs_sample;
    uint8_t accept_rr = 1u;

    pt_update_signal_thresholds(state, peak_i, peak_f, spk_weight);
    pt_record_qrs_reference(state, peak_i, peak_f);
    state->last_slope = current_slope;
    state->last_qrs_sample = detected_sample;
    state->have_qrs = 1u;
    pt_rr_history_prune(state, current_sample);

    if (!had_previous_qrs) {
        state->last_rr_ms = 0u;
        return 0u;
    }

    accept_rr = pt_rr_history_should_accept_new(state, rr_samples);
    pt_rr_history_push(state, prev_qrs_sample, detected_sample, rr_samples, (uint8_t) (!accept_rr));

    if (accept_rr) {
        if (!state->rr_seeded) {
            for (uint8_t i = 0u; i < PT_RR_AVG_LEN; i++) {
                state->rr1[i] = rr_samples;
                state->rr2[i] = rr_samples;
            }
            state->rravg1 = rr_samples;
            state->rravg2 = rr_samples;
            state->rrlow = (uint32_t) (PT_RR_LOW_SCALE * (float) rr_samples);
            state->rrhigh = (uint32_t) (PT_RR_HIGH_SCALE * (float) rr_samples);
            state->rrmiss = (uint32_t) (PT_RR_MISS_SCALE * (float) rr_samples);
            state->regular = true;
            state->rr_seeded = 1u;
        } else {
            state->rravg1 = 0u;
            for (uint8_t i = 0; i < PT_RR_AVG_LEN - 1u; i++) {
                state->rr1[i] = state->rr1[i + 1u];
                state->rravg1 += state->rr1[i];
            }
            state->rr1[PT_RR_AVG_LEN - 1u] = rr_samples;
            state->rravg1 += state->rr1[PT_RR_AVG_LEN - 1u];
            state->rravg1 /= PT_RR_AVG_LEN;

            if ((state->rr1[PT_RR_AVG_LEN - 1u] >= state->rrlow) &&
                (state->rr1[PT_RR_AVG_LEN - 1u] <= state->rrhigh)) {
                state->rravg2 = 0u;
                for (uint8_t i = 0; i < PT_RR_AVG_LEN - 1u; i++) {
                    state->rr2[i] = state->rr2[i + 1u];
                    state->rravg2 += state->rr2[i];
                }
                state->rr2[PT_RR_AVG_LEN - 1u] = state->rr1[PT_RR_AVG_LEN - 1u];
                state->rravg2 += state->rr2[PT_RR_AVG_LEN - 1u];
                state->rravg2 /= PT_RR_AVG_LEN;
                state->rrlow = (uint32_t) (PT_RR_LOW_SCALE * (float) state->rravg2);
                state->rrhigh = (uint32_t) (PT_RR_HIGH_SCALE * (float) state->rravg2);
                state->rrmiss = (uint32_t) (PT_RR_MISS_SCALE * (float) state->rravg2);
            }

            if (state->rravg1 == state->rravg2) {
                state->regular = true;
            } else {
                state->regular = false;
                if (prev_regular) {
                    state->threshold_i1 *= 0.5f;
                    state->threshold_f1 *= 0.5f;
                }
            }
        }
    }
    {
        uint32_t hr_rr = pt_select_recent_rr_for_hr(state, current_sample);
        if (hr_rr == 0u) {
            state->last_rr_ms = 0u;
            return 0u;
        }
        state->last_rr_ms = pt_samples_to_rr_ms(state, hr_rr);
        return pt_filter_hr_from_rr_samples(state, hr_rr);
    }
}

static uint8_t pt_commit_ready_events(pt_state_t *state, uint32_t current_sample) {
    uint32_t guard = pt_artifact_guard_samples(state);
    uint8_t latest_hr = 0u;

    while (state->pending_count > 0u) {
        pt_pending_event_t event = state->pending[0];
        uint8_t hr = 0u;

        if (current_sample < (event.sample + guard)) {
            break;
        }

        if (state->pending_count > 1u) {
            memmove(&state->pending[0], &state->pending[1], (state->pending_count - 1u) * sizeof(state->pending[0]));
        }
        state->pending_count--;

        if (event.invalid) {
            continue;
        }

        if (event.kind == PT_EVENT_QRS) {
            hr = pt_register_qrs(
                state,
                event.sample,
                current_sample,
                event.slope,
                event.peak_i,
                event.peak_f,
                PT_SIGNAL_PEAK_WEIGHT);
        } else if (event.kind == PT_EVENT_NOISE) {
            pt_update_noise_thresholds(state, event.peak_i, event.peak_f);
        }

        if (hr > 0u) {
            latest_hr = hr;
        }
    }

    return latest_hr;
}

int8_t pt_peaks_free(pt_peaks_t *pt_peaks_ptr) {
    if (!pt_peaks_ptr) {
        return -1;
    }

    free(pt_peaks_ptr->peak_indices);
    pt_peaks_ptr->peak_indices = NULL;
    free(pt_peaks_ptr);
    return 0;
}

pt_peaks_t *pt_peaks_init(uint16_t peaks_indices_len) {
    pt_peaks_t *pt_peaks_ptr;

    if (peaks_indices_len == 0u) {
        errno = EINVAL;
        ERROR_CHECK("Invalid peaks_length in peaks init");
        return NULL;
    }

    pt_peaks_ptr = (pt_peaks_t *) malloc(sizeof(pt_peaks_t));
    if (!pt_peaks_ptr) {
        errno = ENOMEM;
        ERROR_CHECK("Failed to allocate pt_peaks");
        return NULL;
    }

    pt_peaks_ptr->peak_indices = (uint16_t *) calloc(peaks_indices_len, sizeof(uint16_t));
    if (!pt_peaks_ptr->peak_indices) {
        errno = ENOMEM;
        ERROR_CHECK("Failed to allocate pt_peaks indices");
        free(pt_peaks_ptr);
        return NULL;
    }

    pt_peaks_ptr->peak_indices_len = peaks_indices_len;
    pt_peaks_ptr->nr_peaks = 0u;
    return pt_peaks_ptr;
}

void pt_reset(void) {
    uint32_t sample_rate_hz = g_pt_state.sample_rate_hz;
    uint16_t window_size = g_pt_state.window_size;
    uint16_t buffer_len = g_pt_state.buffer_len;
    uint8_t bp_order = g_pt_state.bp_order;
    float low_cutoff_hz = g_pt_state.low_cutoff_hz;
    float high_cutoff_hz = g_pt_state.high_cutoff_hz;

    memset(&g_pt_state, 0, sizeof(g_pt_state));
    g_pt_state.sample_rate_hz = sample_rate_hz;
    g_pt_state.window_size = window_size;
    g_pt_state.buffer_len = buffer_len;
    g_pt_state.bp_order = bp_order;
    g_pt_state.low_cutoff_hz = low_cutoff_hz;
    g_pt_state.high_cutoff_hz = high_cutoff_hz;
    g_pt_state.regular = true;
    g_pt_state.signal_polarity = 1;
    if (sample_rate_hz > 0u && bp_order > 0u) {
        (void) bw_band_pass_filter_init(
            &g_pt_state.bandpass,
            bp_order,
            (float) sample_rate_hz,
            low_cutoff_hz,
            high_cutoff_hz);
    }
    hr_filter_reset();
}

int8_t pt_init(uint8_t bp_order, uint32_t sample_rate_hz, float low_cutoff_hz, float high_cutoff_hz, uint32_t integration_time_ms, uint16_t window_len) {
    if (sample_rate_hz == 0u || integration_time_ms == 0u || window_len == 0u || window_len > PAN_TOMPKINS_SIZE) {
        errno = EINVAL;
        ERROR_CHECK("Invalid Pan-Tompkins configuration");
        return -1;
    }
    if (bp_order == 0u || high_cutoff_hz <= low_cutoff_hz ||
        !bw_band_pass_filter_init(&g_pt_state.bandpass, bp_order, (float) sample_rate_hz, low_cutoff_hz, high_cutoff_hz)) {
        errno = EINVAL;
        ERROR_CHECK("Invalid Pan-Tompkins bandpass configuration");
        return -1;
    }

    memset(&g_pt_state, 0, sizeof(g_pt_state));
    g_pt_state.sample_rate_hz = sample_rate_hz;
    g_pt_state.window_size = (uint16_t) ((integration_time_ms * sample_rate_hz) / 1000u);
    if (g_pt_state.window_size == 0u) {
        g_pt_state.window_size = 1u;
    }
    g_pt_state.buffer_len = window_len;
    g_pt_state.bp_order = bp_order;
    g_pt_state.low_cutoff_hz = low_cutoff_hz;
    g_pt_state.high_cutoff_hz = high_cutoff_hz;
    g_pt_state.regular = true;
    g_pt_state.signal_polarity = 1;
    (void) bw_band_pass_filter_init(&g_pt_state.bandpass, bp_order, (float) sample_rate_hz, low_cutoff_hz, high_cutoff_hz);
    hr_filter_reset();
    return 0;
}

uint16_t pt_get_last_rr_ms(void) {
    return g_pt_state.last_rr_ms;
}

void pt_get_debug_snapshot(pt_debug_snapshot_t *out) {
    float artifact_f = NAN;
    float artifact_i = NAN;
    uint16_t idx;
    uint32_t rejected_left[PT_DEBUG_MAX_REJECTED_RR];
    uint32_t rejected_right[PT_DEBUG_MAX_REJECTED_RR];
    uint8_t rejected_count;

    if (!out) {
        return;
    }

    memset(out, 0, sizeof(*out));
    pt_rr_history_prune(&g_pt_state, g_pt_state.sample_count);
    pt_get_artifact_thresholds(&g_pt_state, &artifact_f, &artifact_i);
    idx = g_pt_state.current_index;

    out->sample_count = g_pt_state.sample_count;
    out->current_index = idx;
    out->raw_sample = g_pt_state.signal[idx];
    out->filtered_sample = (float) g_pt_state.signal_polarity * (float) g_pt_state.highpass[idx];
    out->lowpass_sample = g_pt_state.lowpass[idx];
    out->highpass_sample = g_pt_state.highpass[idx];
    out->derivative_sample = g_pt_state.derivative[idx];
    out->squared_sample = g_pt_state.squared[idx];
    out->integral_sample = g_pt_state.integral[idx];
    out->threshold_f1 = g_pt_state.thresholds_ready ? g_pt_state.threshold_f1 : NAN;
    out->threshold_f2 = g_pt_state.thresholds_ready ? g_pt_state.threshold_f2 : NAN;
    out->threshold_i1 = g_pt_state.thresholds_ready ? g_pt_state.threshold_i1 : NAN;
    out->threshold_i2 = g_pt_state.thresholds_ready ? g_pt_state.threshold_i2 : NAN;
    out->artifact_f = artifact_f;
    out->artifact_i = artifact_i;
    out->signal_polarity = g_pt_state.signal_polarity;
    out->artifact_guard_until_sample = g_pt_state.artifact_guard_until_sample;
    out->last_qrs_sample = g_pt_state.last_qrs_sample;
    out->last_artifact_sample = g_pt_state.last_artifact_sample;
    out->last_rr_ms = g_pt_state.last_rr_ms;
    rejected_count = pt_rr_history_collect_rejected(&g_pt_state, rejected_left, rejected_right);
    out->rejected_rr_count = rejected_count;
    for (uint8_t i = 0u; i < rejected_count; i++) {
        out->rejected_rr_left[i] = rejected_left[i];
        out->rejected_rr_right[i] = rejected_right[i];
    }
}

uint8_t pt_process_sample(int32_t raw_sample) {
    pt_state_t *state = &g_pt_state;
    uint16_t current;
    uint32_t hard_ref;
    uint32_t soft_ref;
    float peak_i = 0.0f;
    float peak_f = 0.0f;
    float bandpass_sample;
    uint16_t candidate_index = 0u;
    uint16_t filtered_peak_index = 0u;
    uint8_t have_candidate = 0u;
    uint8_t accepted_qrs = 0u;
    uint8_t classification_kind = PT_EVENT_NONE;
    uint32_t detected_sample = 0u;
    uint32_t current_slope = 0u;
    uint8_t committed_hr = 0u;

    if (state->sample_rate_hz == 0u || state->buffer_len == 0u) {
        errno = EINVAL;
        ERROR_CHECK("Pan-Tompkins not initialized");
        return 0u;
    }

    current = pt_push_sample(state, raw_sample);
    hard_ref = pt_hard_refractory_samples(state);
    soft_ref = pt_soft_refractory_samples(state);

    state->dcblock[current] = 0;
    bandpass_sample = bw_band_pass(&state->bandpass, (float) state->signal[current]);
    state->lowpass[current] = clamp_i64_to_i32((int64_t) lroundf(bandpass_sample));
    state->highpass[current] = state->lowpass[current];

    if (current >= 4u) {
        int64_t derivative =
            (2LL * state->highpass[current]) +
            state->highpass[current - 1u] -
            state->highpass[current - 3u] -
            (2LL * state->highpass[current - 4u]);
        state->derivative[current] = clamp_i64_to_i32(derivative / 8LL);
    } else {
        state->derivative[current] = 0;
    }

    state->squared[current] = clamp_i64_to_i32((int64_t) state->derivative[current] * state->derivative[current]);

    {
        int64_t sum = 0;
        uint16_t count = 0u;
        for (uint16_t i = 0; i < state->window_size; i++) {
            if (current < i) {
                break;
            }
            sum += state->squared[current - i];
            count++;
        }
        state->integral[current] = (count > 0u) ? clamp_i64_to_i32(sum / count) : 0;
    }

    if (!state->thresholds_ready) {
        pt_threshold_warmup_update(state, (uint32_t) state->integral[current], (float) state->highpass[current]);
        if (pt_threshold_warmup_active(state)) {
            return pt_commit_ready_events(state, state->sample_count);
        }
        pt_threshold_warmup_finish(state);
        return pt_commit_ready_events(state, state->sample_count);
    }

    if (current >= 1u) {
        candidate_index = (uint16_t) (current - 1u);
        if (pt_is_local_peak(state->integral, candidate_index, (uint16_t) (current + 1u))) {
            filtered_peak_index = pt_find_filtered_peak(state, candidate_index);
            peak_i = (float) state->integral[candidate_index];
            peak_f = (float) state->signal_polarity * (float) state->highpass[filtered_peak_index];
            have_candidate = 1u;
        }
    }

    if (have_candidate && peak_i >= state->threshold_i1 && peak_f >= state->threshold_f1) {
        if (!state->det_have_qrs || state->sample_count > state->det_last_qrs_sample + hard_ref) {
            detected_sample = state->sample_count - ((uint32_t) current - (uint32_t) filtered_peak_index);
            current_slope = pt_compute_slope(state, filtered_peak_index);

            if (state->det_have_qrs && detected_sample <= state->det_last_qrs_sample + soft_ref) {
                if (current_slope > (state->det_last_slope / 2u)) {
                    accepted_qrs = 1u;
                }
            } else {
                accepted_qrs = 1u;
            }
        } else {
            classification_kind = PT_EVENT_NOISE;
        }
    }

    if (accepted_qrs) {
        if (pt_is_artifact_candidate(state, peak_i, peak_f)) {
            pt_record_artifact(state, detected_sample);
        } else if (detected_sample > state->artifact_guard_until_sample) {
            pt_pending_push(state, detected_sample, current_slope, peak_i, peak_f, PT_EVENT_QRS);
            state->det_last_slope = current_slope;
            state->det_last_qrs_sample = detected_sample;
            state->det_have_qrs = 1u;
        }
        return pt_commit_ready_events(state, state->sample_count);
    }

    if (state->det_have_qrs && state->rrmiss > 0u &&
        (state->sample_count - state->det_last_qrs_sample > state->rrmiss) &&
        (state->sample_count > state->det_last_qrs_sample + hard_ref)) {
        uint32_t age = state->sample_count - state->det_last_qrs_sample;
        int32_t search_start = (int32_t) current - (int32_t) age + (int32_t) hard_ref;
        if (search_start < 0) {
            search_start = 0;
        }

        for (uint16_t i = (uint16_t) search_start; i < current; i++) {
            uint16_t search_peak_index;
            float search_peak_f;
            uint32_t current_slope;
            uint32_t detected_sample;

            if (!pt_is_local_peak(state->integral, i, (uint16_t) (current + 1u))) {
                continue;
            }

            search_peak_index = pt_find_filtered_peak(state, i);
            search_peak_f = (float) state->signal_polarity * (float) state->highpass[search_peak_index];
            if ((float) state->integral[i] <= state->threshold_i2 || search_peak_f <= state->threshold_f2) {
                continue;
            }

            current_slope = pt_compute_slope(state, search_peak_index);
            detected_sample = state->sample_count - (current - search_peak_index);

            if (state->det_have_qrs &&
                current_slope < (state->det_last_slope / 2u) &&
                detected_sample < state->det_last_qrs_sample + soft_ref) {
                continue;
            }

            peak_i = (float) state->integral[i];
            peak_f = search_peak_f;
            accepted_qrs = 1u;
            break;
        }
    }

    if (accepted_qrs) {
        if (pt_is_artifact_candidate(state, peak_i, peak_f)) {
            pt_record_artifact(state, detected_sample);
        } else if (detected_sample > state->artifact_guard_until_sample) {
            pt_pending_push(state, detected_sample, current_slope, peak_i, peak_f, PT_EVENT_QRS);
            state->det_last_slope = current_slope;
            state->det_last_qrs_sample = detected_sample;
            state->det_have_qrs = 1u;
        }
        return pt_commit_ready_events(state, state->sample_count);
    }

    if (classification_kind == PT_EVENT_NONE &&
        have_candidate &&
        (peak_i >= state->threshold_i1 || peak_f >= state->threshold_f1)) {
        uint32_t noise_sample = state->sample_count - ((uint32_t) current - (uint32_t) filtered_peak_index);
        if (noise_sample > state->artifact_guard_until_sample) {
            classification_kind = PT_EVENT_NOISE;
            pt_pending_push(state, noise_sample, 0u, peak_i, peak_f > 0.0f ? peak_f : 0.0f, PT_EVENT_NOISE);
        }
    }

    committed_hr = pt_commit_ready_events(state, state->sample_count);
    return committed_hr;
}

uint8_t pt_process(int32_t raw_signal[], uint16_t signal_len) {
    uint8_t last_hr = 0u;

    if (!raw_signal || signal_len == 0u) {
        errno = EINVAL;
        ERROR_CHECK("pt_process: bad args");
        return 0u;
    }

    for (uint16_t i = 0; i < signal_len; i++) {
        uint8_t hr = pt_process_sample(raw_signal[i]);
        if (hr > 0u) {
            last_hr = hr;
        }
    }

    return last_hr;
}

#ifdef PT_DEBUG
pt_intermediate_signals_t *pt_intermediate_signals_init(uint16_t signal_len) {
    pt_intermediate_signals_t *sig;

    if (signal_len == 0u) {
        return NULL;
    }

    sig = (pt_intermediate_signals_t *) calloc(1, sizeof(*sig));
    if (!sig) {
        return NULL;
    }

    sig->BW_filter = (int32_t *) calloc(signal_len, sizeof(int32_t));
    sig->derivative = (int32_t *) calloc(signal_len, sizeof(int32_t));
    sig->Squared = (int32_t *) calloc(signal_len, sizeof(int32_t));
    sig->MW = (int32_t *) calloc(signal_len, sizeof(int32_t));
    if (!sig->BW_filter || !sig->derivative || !sig->Squared || !sig->MW) {
        pt_intermediate_signals_free(sig);
        return NULL;
    }

    return sig;
}

void pt_intermediate_signals_free(pt_intermediate_signals_t *sig) {
    if (!sig) {
        return;
    }

    free(sig->BW_filter);
    free(sig->derivative);
    free(sig->Squared);
    free(sig->MW);
    free(sig);
}

int8_t pt_process_debug(int32_t sig_in[], uint16_t signal_len, pt_peaks_t *pt_peaks, pt_intermediate_signals_t *output) {
    (void) sig_in;
    (void) signal_len;
    (void) pt_peaks;
    (void) output;
    errno = ENOTSUP;
    ERROR_CHECK("PT_DEBUG path not implemented for streaming detector");
    return -1;
}
#endif
