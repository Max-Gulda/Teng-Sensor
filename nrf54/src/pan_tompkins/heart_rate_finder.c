#include "heart_rate_finder.h"
#include "error_handling.h"
#include "hr_filter.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(heart_rate_finder, LOG_LEVEL_INF);

uint8_t calculate_heart_from_peaks(uint16_t peak_indices[], uint16_t number_of_peaks, uint16_t sampling_rate) {
    if (!peak_indices) {
        ERROR_CHECK("Arguments are not correctly initialised");
        return 0;
    }

    if (number_of_peaks < 2) {
        ERROR_CHECK("Not enough peaks in the interval");
        return 0;
    }

    float sum_diffs = 0.0f;
    for (uint16_t i = 1; i < number_of_peaks; i++) {
        sum_diffs += (float)(peak_indices[i] - peak_indices[i - 1]);
    }
    float avg_dist = sum_diffs / (float)(number_of_peaks - 1);
    float avg_period_s = avg_dist / sampling_rate;

    if (avg_period_s <= 0.0f) {
        ERROR_CHECK("Invalid period (avg_period_s <= 0)");
        return 0;
    }

    float bpm_instant = 60.0f / avg_period_s;
    uint8_t bpm_raw = (bpm_instant > 255.0f) ? 255u : (uint8_t) (bpm_instant + 0.5f);
    LOG_DBG("raw_heart: %u", bpm_raw);
    return hr_filter_update(bpm_raw);
}
