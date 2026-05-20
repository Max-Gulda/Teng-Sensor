/**
 * @file sampling.h
 * @brief DRDY-synchronized ADC sampling system
 */

#ifndef SAMPLING_H
#define SAMPLING_H

#include <zephyr/kernel.h>
#include "ads131m04_spi.h"
#include "define.h"

/* Sampling configuration */
#define SAMPLING_DRDY_TIMEOUT_US 1000000U
#define NON_SAMPLING_TIME_S 5 /* sets time that sampling idles at when not connected to BLE client */
#define ADS131M04_DEFAULT_SAMPLE_RATE_HZ 500U

/* Thread configuration */
#define SAMPLING_THREAD_PRIORITY   5      /* High priority for time-critical sampling */
#define SAMPLING_THREAD_STACK_SIZE 2048

/* Sampling statistics structure */
typedef struct {
    uint32_t total_samples;       /* Total samples taken */
    uint32_t missed_samples;      /* Number of DRDY wait timeouts */
    uint32_t max_jitter_us;       /* Longest observed inter-sample period in microseconds */
    uint64_t last_period_us;      /* Last measured sampling period */
} sampling_stats_t;

/**
 * @brief Initialize the sampling system
 * @param adc_config Pointer to ADC configuration
 * @return 0 on success, negative error code on failure
 */
int sampling_init(ads131m04_config_t *adc_config);

/**
 * @brief Start the DRDY-driven sampling thread
 * @return 0 on success, negative error code on failure
 */
int sampling_start(void);

/**
 * @brief Stop sampling
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
 * @brief Set ADS131M04 output data rate by selecting a standard OSR value.
 * @param sample_rate_hz Standard ADS131M04 sample rate in samples per second
 * @return 0 on success, negative error code on failure
 */
int sampling_set_ads131_sample_rate_hz(uint32_t sample_rate_hz);

/**
 * @brief Get current ADS131M04 output data rate.
 * @return Current ADS131M04 sample rate in samples per second
 */
uint32_t sampling_get_ads131_sample_rate_hz(void);

/**
 * @brief Wake DRDY-driven sampling after BLE connection
 * Called by bluetooth module when connection is established
 */
void sampling_restore_fast_rate(void);

#endif /* SAMPLING_H */
