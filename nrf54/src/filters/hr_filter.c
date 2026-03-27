#include "hr_filter.h"
#include "heart_rate_tuning_params.h"
#include <stdint.h>

#ifndef HR_FILTER_BPM_MIN
#define HR_FILTER_BPM_MIN 10.0f
#endif
#ifndef HR_FILTER_BPM_MAX
#define HR_FILTER_BPM_MAX 255.0f
#endif
#ifndef HR_FILTER_BOOTSTRAP_SAMPLES
#define HR_FILTER_BOOTSTRAP_SAMPLES 3u
#endif

static float s_state; /* Current filtered HR estimate */
static uint8_t s_initialized;
static float s_bootstrap[HR_FILTER_BOOTSTRAP_SAMPLES];
static uint8_t s_bootstrap_count;

static float clampf_local(float x, float lo, float hi) {
  if (x < lo)
    return lo;
  if (x > hi)
    return hi;
  return x;
}

static uint8_t  hr_filter_output_u8(void) {
  float y = clampf_local(s_state, HR_FILTER_BPM_MIN, HR_FILTER_BPM_MAX);
  return (uint8_t)(y + 0.5f);
}

static void hr_filter_sort(float *values, uint8_t count) {
  for (uint8_t i = 1; i < count; i++) {
    float key = values[i];
    uint8_t j = i;

    while (j > 0 && values[j - 1] > key) {
      values[j] = values[j - 1];
      j--;
    }
    values[j] = key;
  }
}

static float hr_filter_median(const float *values, uint8_t count) {
  float sorted[HR_FILTER_BOOTSTRAP_SAMPLES];

  for (uint8_t i = 0; i < count; i++) {
    sorted[i] = values[i];
  }
  hr_filter_sort(sorted, count);

  if ((count & 1u) != 0u) {
    return sorted[count / 2u];
  }

  return 0.5f * (sorted[(count / 2u) - 1u] + sorted[count / 2u]);
}

void hr_filter_reset(void) {
  s_state = 0.0f;
  s_initialized = 0;
  s_bootstrap_count = 0;
}

uint8_t hr_filter_update(uint8_t bpm) {
  float z = (float)bpm;

  /* Reject invalid measurements before they affect the filter state. */
  if (z < HR_FILTER_BPM_MIN || z > HR_FILTER_BPM_MAX) {
    if (!s_initialized) {
      return 0;
    }
    return hr_filter_output_u8();
  }

  /* Match the GUI display path: wait for a short bootstrap and seed from its median. */
  if (!s_initialized) {
    if (s_bootstrap_count < HR_FILTER_BOOTSTRAP_SAMPLES) {
      s_bootstrap[s_bootstrap_count++] = z;
    }
    if (s_bootstrap_count < HR_FILTER_BOOTSTRAP_SAMPLES) {
      return 0;
    }
    s_state = hr_filter_median(s_bootstrap, s_bootstrap_count);
    s_initialized = 1;
    s_bootstrap_count = 0;
    return hr_filter_output_u8();
  }

  s_state = (HR_FILTER_ALPHA * z) + ((1.0f - HR_FILTER_ALPHA) * s_state);

  return hr_filter_output_u8();
}
