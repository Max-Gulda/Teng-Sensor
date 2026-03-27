#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <stdint.h>
#include <stdio.h>

/**
 * @brief Apply a simple FIR filter without frequency derivative compensation.
 *
 * This function filters an input signal using a fixed convolution kernel:
 *
 *     y[i] = (-x[i-2] - 2*x[i-1] + 2*x[i+1] + x[i+2]) / 8
 *
 * The filter introduces a delay and may produce transient effects at the
 * beginning of the signal. Optionally, the initial transient samples can
 * be suppressed by setting them to zero.
 *
 * @param[in]  signal_in   Pointer to the input signal array (int32_t).
 * @param[out] signal_out  Pointer to the output signal array (int32_t).
 * @param[in]  signal_length Number of samples in the signal.
 * @param[in]  suppress_initial_transient
 *                        If > 0, the first N samples are set to zero
 *                        and the last two samples are copied from
 *                        their predecessors to reduce artifacts.
 *
 * @return int8_t
 *         -  0 on success
 *         - -1 if input is invalid (signal_length too small, NULL pointer, etc.)
 *
 * @note
 *  - The caller must allocate `signal_out` with at least `signal_length` elements.
 *  - This filter is linear but not normalized for unity gain. The signal size need to be bigger than 4.
 */
int8_t sig_proc_derivative(int32_t signal_in[], int32_t signal_out[], uint16_t signal_length, uint8_t suppress_initial_transient);


/**
 * @brief Performs in-place moving window integration on a signal array.
 *
 * This function computes a moving average of the input signal over a window of size `win`.
 * The result replaces the original values in the input array. For the first `win-1` samples,
 * the average is computed using all available previous samples.
 *
 * @param[in,out] signal_in Pointer to the input signal array. The averaged values are written back here.
 * @param[in] signal_length  Number of elements in the input signal array.
 * @param[in] win          Window size for the moving average. Must be greater than 0.
 *
 * @return int8_t Status code:
 *         -  0: Success
 *         - -1: Invalid input
 *
 * @warning The input array will be modified in-place. If the original signal needs to be preserved,
 *          use a separate output array.
 */
int8_t sig_proc_mw_integration(int32_t signal_in[], uint16_t signal_length, uint32_t win, uint32_t gain);


#endif