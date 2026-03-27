#ifndef HEART_RATE_H
#define HEART_RATE_H

#include <stdint.h>

/**
 * @brief Calculates heart rate (in beats per minute) from detected signal peaks.
 *
 * This function estimates the heart rate based on the indices of detected peaks
 * in a signal (e.g. ECG after running our pan tompkins implementation). The time intervals between
 * consecutive peaks are used to compute the average heart rate.
 *
 * @param[in] peak_indices      Array containing the indices of detected peaks in the signal.
 * @param[in] number_of_peaks   Total number of peaks detected in the signal.
 * @param[in] sampling_rate     Sampling rate of the signal in Hz (samples per second).
 *
 * @return The estimated heart rate in beats per minute (BPM). Returns 0 if not enough peaks are provided.
 *
 * @note A minimum of two peaks is required to compute the heart rate.
 *
 * @example
 * uint16_t peaks[] = {120, 380, 640, 900};
 * float bpm = calculate_heart_from_peaks(peaks, 4, 100);
 * // bpm will contain the estimated heart rate
 */

uint8_t calculate_heart_from_peaks(uint16_t peak_indices[], uint16_t number_of_peaks, uint16_t sampling_rate);


#endif 
