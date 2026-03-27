#ifndef HR_FILTER_H
#define HR_FILTER_H

#include <stdint.h>

/**
 * @brief Feed one raw BPM sample into a fixed EWMA heart-rate filter.
 *
 * The first valid sample seeds the filter state and is returned unmodified.
 * Subsequent samples are smoothed with a fixed alpha:
 * `y[n] = alpha * x[n] + (1 - alpha) * y[n-1]`.
 *
 * @param bpm  Raw BPM estimate from Pan-Tompkins, as a uint8_t.
 * @return     Filtered BPM, as a uint8_t.
 */
uint8_t hr_filter_update(uint8_t bpm);

/**
 * @brief Reset all internal filter state.
 *
 * Call this if the sensor loses contact mid-session so that the next beat
 * re-seeds the filter cleanly instead of jumping from stale state.
 */
void hr_filter_reset(void);

#endif /* HR_FILTER_H */
