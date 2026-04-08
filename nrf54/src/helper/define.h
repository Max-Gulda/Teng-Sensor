#define ECG_SIZE 516 /* Sets size of ECG circ buffer */
#define SAMPLING_RATE 200 /* Sets sampling rate for ADC */
#define PAN_TOMPKINS_SIZE 516 /* Sets amount of samples before running PT */

/* PCB mapping: CH0 is ECG/heart. CH1-CH3 are currently floating on this PCB. */
#define ECG_SOURCE_ADC_CHANNEL 0
