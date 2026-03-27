/**
 * @file sampling.h
 * @brief Time-synchronized ADC sampling system
 */

#ifndef SAMPLING_H
#define SAMPLING_H

#include <zephyr/kernel.h>
#include "ads131m02_spi.h"
#include "define.h"

/* Sampling configuration */
#define SAMPLING_FREQUENCY_HZ SAMPLING_RATE    /* Default sampling frequency */
#define SAMPLING_PERIOD_US    (1000000 / SAMPLING_FREQUENCY_HZ)
#define NON_SAMPLING_TIME_S 5 /* sets time that sampling idles at when not connected to BLE client */

/* Thread configuration */
#define SAMPLING_THREAD_PRIORITY   5      /* High priority for time-critical sampling */
#define SAMPLING_THREAD_STACK_SIZE 2048

/* Sampling statistics structure */
typedef struct {
    uint32_t total_samples;       /* Total samples taken */
    uint32_t missed_samples;      /* Number of missed deadlines */
    uint32_t max_jitter_us;       /* Maximum observed jitter in microseconds */
    uint64_t last_period_us;      /* Last measured sampling period */
} sampling_stats_t;

/**
 * @brief Initialize the sampling system
 * @param adc_config Pointer to ADC configuration
 * @return 0 on success, negative error code on failure
 */
int sampling_init(ads131m02_config_t *adc_config);

/**
 * @brief Start the sampling timer and thread
 * @return 0 on success, negative error code on failure
 */
int sampling_start(void);

/**
 * @brief Stop the sampling timer
 * @return 0 on success, negative error code on failure
 */
int sampling_stop(void);

/**
 * @brief Get current sampling statistics
 * @param stats Pointer to structure to store statistics
 */
void sampling_get_stats(sampling_stats_t *stats);

/**
 * @brief Reset sampling statistics counters
 */
void sampling_reset_stats(void);

/**
 * @brief Restore fast sampling rate (50 Hz)
 * Called by bluetooth module when connection is established
 */
void sampling_restore_fast_rate(void);

#endif /* SAMPLING_H */
