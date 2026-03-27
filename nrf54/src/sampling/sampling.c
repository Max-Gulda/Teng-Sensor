/**
 * @file sampling.c
 * @brief Time-synchronized ADC sampling implementation
 */

/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jesper Sjöberg (@repsejs, KTH: @jessjobe)
 * Copyright (c) 2026 Max Gulda (@Max-Gulda, KTH: @gulda)
 */

#include "sampling.h"
#include "bluetooth.h"
#include "ecg.h"
#include "lsm6dso_spi.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <stdbool.h>

LOG_MODULE_REGISTER(sampling, LOG_LEVEL_INF);

/* Global ADC configuration */
static ads131m02_config_t *g_adc_config = NULL;
static lsm6dso_config_t *g_imu_config = NULL;

/* Sampling statistics */
static volatile uint32_t sample_count = 0;
static volatile uint64_t last_sample_time_cycles = 0;
static volatile uint32_t max_jitter_us = 0;
static volatile uint32_t missed_samples = 0;
static volatile uint32_t imu_read_failures = 0;
static volatile uint32_t adc_read_failures = 0;
static volatile uint32_t adc_frame_outliers = 0;
static bool have_last_valid_adc = false;
static ads131m02_data_t last_valid_adc = { 0 };

/* Synchronization: Timer signals thread via semaphore */
K_SEM_DEFINE(sampling_sem, 0, 1);

/* Timer for precise time synchronization */
static void sampling_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(sampling_timer, sampling_timer_handler, NULL);

static inline bool adc_sample_is_rail_value(const ads131m02_data_t *adc) {
    return (adc->ch0 == 8388607) || (adc->ch0 == -8388608) ||
           (adc->ch1 == 8388607) || (adc->ch1 == -8388608);
}

static inline int read_adc_frame(ads131m02_data_t *adc) {
    int err = ads131m02_read_adc(g_adc_config, adc);
    if (err < 0) {
        return err;
    }

    if (adc_sample_is_rail_value(adc)) {
        ads131m02_data_t retry;
        err = ads131m02_read_adc(g_adc_config, &retry);
        if (err == 0 && !adc_sample_is_rail_value(&retry)) {
            *adc = retry;
            return 0;
        }
        return -EIO;
    }

    return 0;
}

/**
 * @brief Read IMU data (accel + gyro)
 */
static inline int read_imu(lsm6dso_data_t *data) {
    if (g_imu_config == NULL || data == NULL) {
        return -ENODEV;
    }
    int err = lsm6dso_read_data(g_imu_config, data);
    if (err < 0) {
        imu_read_failures++;
        if ((imu_read_failures == 1) || (imu_read_failures % 50 == 0)) {
            LOG_WRN("IMU read failed (%u): %d", imu_read_failures, err);
        }
        return err;
    }
    return 0;
}

/**
 * @brief Timer ISR - fires at precise sampling frequency
 * Runs in interrupt context - just signals the sampling thread
 * When disconnected, slows to 1/5 Hz for power savings
 */
static void sampling_timer_handler(struct k_timer *timer) {
    if (bluetooth_is_connected()) {
        /* Connected: signal sampling thread at 50 Hz */
        k_sem_give(&sampling_sem);
    } else {
        /* Disconnected: slow down to 1/5 Hz to save power */
        k_timer_start(&sampling_timer, K_SECONDS(NON_SAMPLING_TIME_S), K_SECONDS(NON_SAMPLING_TIME_S));
    }
}

/**
 * @brief High-priority sampling thread
 * Waits for timer signal, then performs ADC sampling in thread context
 */
static void sampling_thread(void *arg1, void *arg2, void *arg3) {
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    ads131m02_data_t adc;
    LOG_INF("Sampling thread started (priority %d)", SAMPLING_THREAD_PRIORITY);

    while (1) {
        /* Wait for timer to signal (blocks until timer fires) */
        if (k_sem_take(&sampling_sem, K_FOREVER) != 0) {
            continue;
        }

        /* Check if we missed the deadline (semaphore count > 0 = missed sample) */
        if (k_sem_count_get(&sampling_sem) > 0) {
            missed_samples++;
            LOG_WRN("Missed sample! (total: %u)", missed_samples);
        }

        /* Perform ADC sampling (in thread context - SPI can work!) */
        if (read_adc_frame(&adc) == 0) {
            last_valid_adc = adc;
            have_last_valid_adc = true;
        } else {
            adc_read_failures++;
            LOG_WRN("ADC read failed (%u)", adc_read_failures);
            
            if (have_last_valid_adc) {
                adc = last_valid_adc;
                adc_frame_outliers++;
                LOG_WRN("ADC frame fallback used (%u)", adc_frame_outliers);
                
            } else {
                adc.ch1 = 0;
                adc.ch0 = 0;
            }
        }

        int32_t heart = adc.ch1;
        int32_t adc_aux = adc.ch0;

        /* Send data to Bluetooth thread - CRITICAL: Cannot drop samples! */
        (void) bluetooth_queue_sample(heart, adc_aux, sample_count);
        /* Note: bluetooth.c handles all error logging and warnings */
        lsm6dso_data_t imu;
        if (read_imu(&imu) == 0) {
            (void) bluetooth_queue_imu_sample(
                imu.accel_x, imu.accel_y, imu.accel_z,
                imu.gyro_x, imu.gyro_y, imu.gyro_z,
                sample_count
            );
        }
        //if ecg is not initialised this will fail and crash the program
        (void) ecg_queue_sample(heart);

        sample_count++;
    }
}

/* Define the sampling thread */
K_THREAD_DEFINE(sampling_thread_id, SAMPLING_THREAD_STACK_SIZE,
    sampling_thread, NULL, NULL, NULL,
    SAMPLING_THREAD_PRIORITY, 0, 0);

/* ========== Public API Implementation ========== */

int sampling_init(ads131m02_config_t *adc_config) {
    if (adc_config == NULL) {
        LOG_ERR("ADC configuration is NULL");
        return -EINVAL;
    }

    g_adc_config = adc_config;
    g_imu_config = lsm6dso_get_config();

    LOG_INF("Sampling system initialized @ %d Hz (%d us period)",
        SAMPLING_FREQUENCY_HZ, SAMPLING_PERIOD_US);

    return 0;
}

int sampling_start(void) {
    if (g_adc_config == NULL) {
        LOG_ERR("Sampling not initialized - call sampling_init() first");
        return -EINVAL;
    }

    adc_read_failures = 0;
    adc_frame_outliers = 0;
    have_last_valid_adc = false;
    last_valid_adc = (ads131m02_data_t){ 0 };

    LOG_INF("Starting time-synchronized sampling...");

    /* Start the timer - sampling thread is already running, waiting for semaphore */
    k_timer_start(&sampling_timer, K_USEC(SAMPLING_PERIOD_US), K_USEC(SAMPLING_PERIOD_US));

    LOG_INF("Sampling started successfully");
    return 0;
}

int sampling_stop(void) {
    LOG_INF("Stopping sampling...");
    k_timer_stop(&sampling_timer);
    LOG_INF("Sampling stopped");
    return 0;
}

void sampling_get_stats(sampling_stats_t *stats) {
    if (stats == NULL) {
        return;
    }

    stats->total_samples = sample_count;
    stats->missed_samples = missed_samples;
    stats->max_jitter_us = max_jitter_us;

    if (last_sample_time_cycles > 0) {
        uint64_t current = k_cycle_get_64();
        uint64_t period_cycles = current - last_sample_time_cycles;
        stats->last_period_us = k_cyc_to_us_floor64(period_cycles);
    } else {
        stats->last_period_us = 0;
    }
}

void sampling_reset_stats(void) {
    sample_count = 0;
    missed_samples = 0;
    max_jitter_us = 0;
    last_sample_time_cycles = 0;
    adc_read_failures = 0;
    adc_frame_outliers = 0;
    have_last_valid_adc = false;
    last_valid_adc = (ads131m02_data_t){ 0 };
    LOG_INF("Sampling statistics reset");
}

void sampling_restore_fast_rate(void) {
    /* Restore 50 Hz sampling rate when BLE connects */
    k_timer_start(&sampling_timer, K_USEC(SAMPLING_PERIOD_US), K_USEC(SAMPLING_PERIOD_US));
    LOG_INF("Sampling rate restored to %d Hz", SAMPLING_FREQUENCY_HZ);
}
