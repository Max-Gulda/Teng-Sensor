#ifndef PAN_TOMPKINS_REF_H
#define PAN_TOMPKINS_REF_H

#include <stdint.h>

/**
 * @brief Structure representing detected peaks in a physiological signal.
 *
 * This structure stores information about the indices of detected peaks
 * in a sampled signal (such as a PPG or ECG waveform). It includes a pointer
 * to the array of peak indices, the allocated length of that array, and
 * the number of peaks actually detected.
 *
 * @typedef pt_peaks_t
 *
 * @struct pt_peaks
 * @var pt_peaks::peak_indices
 *      Pointer to an array containing the sample indices where peaks occur.
 *
 * @var pt_peaks::peak_indices_len
 *      Length of the allocated peak indices array (capacity).
 *
 * @var pt_peaks::nr_peaks
 *      Number of peaks actually detected and stored in the array.
 *
 *
 * @example
 * pt_peaks_t peaks_data;
 * peaks_data.peak_indices = detected_indices;
 * peaks_data.peak_indices_len = MAX_PEAKS;
 * peaks_data.nr_peaks = count_detected_peaks();
 */
typedef struct pt_peaks {
  uint16_t *peak_indices;     /**< Pointer to array of detected peak indices. */
  uint16_t  peak_indices_len; /**< Length of allocated peak array. */
  uint16_t  nr_peaks;         /**< Number of peaks actually detected. */
} pt_peaks_t;



#ifdef PT_DEBUG
/**
 * @brief Structure holding intermediate signal processing results.
 *
 * This structure contains pointers to intermediate data arrays generated
 * during signal processing — for example, when analyzing a physiological
 * signal such as ECG or PPG to detect peaks or compute heart rate.
 *
 * These arrays typically represent the step-by-step outputs of a processing
 * pipeline (e.g., filtering, differentiation, squaring, and moving window
 * integration).
 *
 * @warning This structure is **intended for debugging and analysis only**.
 *          It should **not be used in production code or embedded systems**
 *          due to increased memory usage and computational load.
 *
 * @typedef pt_intermediate_signals_t
 *
 * @struct pt_intermediate
 * @var pt_intermediate::BW_filter
 *      Pointer to the bandpass-filtered signal data.
 *
 * @var pt_intermediate::derivative
 *      Pointer to the first-derivative signal data.
 *
 * @var pt_intermediate::Squared
 *      Pointer to the squared signal data.
 *
 * @var pt_intermediate::MW
 *      Pointer to the moving window integrated signal data.
 */
typedef struct pt_intermediate {
  int32_t *BW_filter;   /**< Pointer to bandpass-filtered signal data. */
  int32_t *derivative;  /**< Pointer to derivative signal data. */
  int32_t *Squared;     /**< Pointer to squared signal data. */
  int32_t *MW;          /**< Pointer to moving window integrated signal data. */
} pt_intermediate_signals_t;
#endif



/**
 * @brief Number of initial samples to suppress during signal processing.
 *
 * This constant defines how many samples at the beginning of the signal
 * should be ignored to avoid startup artifacts (e.g., filter transients
 * or unstable signal regions).
 */
#ifndef INITIAL_SUPPRESSION
#define INITIAL_SUPPRESSION 16
#endif


 /**
  * @brief Maximum allowable heart rate in beats per minute (BPM).
  *
  * This constant defines an upper threshold for valid heart rate detection.
  * Any calculated value exceeding this limit is considered invalid or noise.
  */
#ifndef MAX_BPM
#define MAX_BPM 255
#endif
#ifndef PT_DEBUG_MAX_REJECTED_RR
#define PT_DEBUG_MAX_REJECTED_RR 6u
#endif

typedef struct pt_debug_snapshot {
  uint32_t sample_count;
  uint16_t current_index;
  int32_t raw_sample;
  float filtered_sample;
  int32_t lowpass_sample;
  int32_t highpass_sample;
  int32_t derivative_sample;
  int32_t squared_sample;
  int32_t integral_sample;
  float threshold_f1;
  float threshold_f2;
  float threshold_i1;
  float threshold_i2;
  float artifact_f;
  float artifact_i;
  int8_t signal_polarity;
  uint32_t artifact_guard_until_sample;
  uint32_t last_qrs_sample;
  uint32_t last_artifact_sample;
  uint16_t last_rr_ms;
  uint8_t rejected_rr_count;
  uint32_t rejected_rr_left[PT_DEBUG_MAX_REJECTED_RR];
  uint32_t rejected_rr_right[PT_DEBUG_MAX_REJECTED_RR];
} pt_debug_snapshot_t;



  /* --------------------------------------------------------------------------
   * Peak Management
   * -------------------------------------------------------------------------- */


   /**
    * @brief Initialize a pt_peaks_t structure.
    *
    * Allocates memory for a @ref pt_peaks_t instance and its peak index array.
    *
    * @param[in] peaks_indices_len  Length of the peak index array.
    * @return Pointer to the initialized structure, or NULL on failure.
    */
pt_peaks_t *pt_peaks_init(uint16_t peaks_indices_len);


/**
 * @brief Free a pt_peaks_t structure.
 *
 * Releases memory allocated by @ref pt_peaks_init().
 *
 * @param[in,out] pt_peaks_ptr  Pointer to the structure to free.
 * @return 0 on success, negative value on error.
 */
int8_t pt_peaks_free(pt_peaks_t *pt_peaks_ptr);



/* --------------------------------------------------------------------------
 * Initialization
 * -------------------------------------------------------------------------- */

 /**
  * @brief Initialize the peak tracking (Pan–Tompkins) module.
  *
  * Sets up filter parameters and processing configuration.
  *
  * @param[in] bp_order             Number of filter sections.
  * @param[in] sample_rate_hz       Sampling rate in Hz.
  * @param[in] low_cutoff_hz        Low cutoff frequency in Hz.
  * @param[in] high_cutoff_hz       High cutoff frequency in Hz.
  * @param[in] integration_time_ms  Moving window integration time in ms.
  * @param[in] window_len           Length of the processing window.
  *
  * @return 0 on success, negative value on failure.
  */
int8_t pt_init(uint8_t bp_order, uint32_t sample_rate_hz,
  float low_cutoff_hz, float high_cutoff_hz,
  uint32_t integration_time_ms, uint16_t window_len);



/* --------------------------------------------------------------------------
 * Signal Processing
 * -------------------------------------------------------------------------- */

/**
 * @brief Process a signal and detect peaks (production pipeline).
 *
 * Compatibility wrapper that feeds a batch of samples through the streaming
 * detector and returns the most recent non-zero BPM it produced.
 *
 * @param[in] raw_signal   Input raw signal array.
 * @param[in] signal_len   Length of the signal arrays.
 *
 * @return Calculated heart rate on success, negative value on error.
 */
uint8_t pt_process(int32_t raw_signal[], uint16_t signal_len);

/**
 * @brief Process one ECG sample through the streaming Pan-Tompkins detector.
 *
 * Returns a non-zero BPM only when a new QRS complex is confirmed and a valid
 * RR interval is available.
 *
 * @param[in] raw_sample One raw ECG sample.
 *
 * @return Filtered heart rate in BPM for a newly detected beat, or 0.
 */
uint8_t pt_process_sample(int32_t raw_sample);

/**
 * @brief Reset the detector state while keeping the current configuration.
 */
void pt_reset(void);

/**
 * @brief Get the most recent RR interval from the last detected beat.
 *
 * Returns the time between the last two detected R-peaks in milliseconds.
 *
 * @return RR interval in ms, or 0 if fewer than 2 peaks were detected.
 */
uint16_t pt_get_last_rr_ms(void);

/**
 * @brief Get the current internal streaming snapshot for GUI/debug use.
 */
void pt_get_debug_snapshot(pt_debug_snapshot_t *out);



/* --------------------------------------------------------------------------
 * DEBUG-ONLY API
 * -------------------------------------------------------------------------- */

#ifdef PT_DEBUG

 /**
  * @brief Initialize a pt_intermediate_signals_t structure.
  *
  * Allocates memory for a @ref pt_intermediate_signals_t instance and its internal arrays.
  *
  * @warning DEBUG ONLY — not for production use.
  *
  * @param[in] signal_len  Length of signal arrays to allocate.
  * @return Pointer to the allocated structure, or NULL on failure.
  */
pt_intermediate_signals_t *pt_intermediate_signals_init(uint16_t signal_len);


/**
 * @brief Free a pt_intermediate_signals_t structure.
 *
 * Releases memory allocated for the debug signal buffers.
 *
 * @warning DEBUG ONLY — not for production use.
 *
 * @param[in,out] inter_to_del Pointer to the structure to free.
 */
void pt_intermediate_signals_free(pt_intermediate_signals_t *inter_to_del);


/**
 * @brief Perform full signal processing and export intermediate results.
 *
 * Stores filtering, derivative, squaring, and moving window outputs.
 *
 * @warning DEBUG ONLY — not for production use.
 *
 * @param[in]  sig_in      Input raw signal array.
 * @param[in]  signal_len  Length of the signal.
 * @param[out] pt_peaks    Pointer to peak output structure.
 * @param[out] output      Pointer to intermediate result buffers.
 *
 * @return 0 on success, negative value on error.
 */
int8_t pt_process_debug(int32_t sig_in[],
  uint16_t signal_len,
  pt_peaks_t *pt_peaks,
  pt_intermediate_signals_t *output);

#endif /* PT_DEBUG */

#endif /* PAN_TOMPKINS_REF_H */
