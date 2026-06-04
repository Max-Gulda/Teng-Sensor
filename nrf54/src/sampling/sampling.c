/**
 * @file sampling.c
 * @brief DRDY interrupt-synchronized ADC sampling implementation
 */

/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jesper Sjöberg (@repsejs, KTH: @jessjobe)
 * Copyright (c) 2026 Max Gulda (@Max-Gulda, KTH: @gulda)
 */

#include "sampling.h"
#include "bluetooth.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <errno.h>
#include <stdbool.h>

LOG_MODULE_REGISTER(sampling, LOG_LEVEL_INF);

/* Global ADC configuration */
static ads131m04_config_t *g_adc_config = NULL;
static volatile bool sampling_active = false;
static volatile uint32_t current_ads131_sample_rate_hz = ADS131M04_DEFAULT_SAMPLE_RATE_HZ;

/* Sampling statistics */
static volatile uint32_t sample_count = 0;
static volatile uint64_t last_sample_time_cycles = 0;
static volatile uint32_t max_jitter_us = 0;
static volatile uint32_t missed_samples = 0;
static volatile uint32_t adc_read_failures = 0;

/* Synchronization: lifecycle events wake the sampling thread, DRDY IRQs wake sampling */
K_SEM_DEFINE(sampling_sem, 0, 1);
K_SEM_DEFINE(drdy_sem, 0, 1);
K_MUTEX_DEFINE(adc_mutex);

static struct gpio_callback drdy_gpio_callback;
static atomic_t drdy_edge_count;
static bool drdy_interrupt_configured = false;

static void drdy_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    atomic_inc(&drdy_edge_count);
    k_sem_give(&drdy_sem);
}

static void reset_drdy_wait_state(void) {
    atomic_set(&drdy_edge_count, 0);
    k_sem_reset(&drdy_sem);
}

static void prime_drdy_wait_if_ready(void) {
    int drdy_ready = ads131m04_drdy_is_low(g_adc_config);

    if (drdy_ready > 0) {
        atomic_inc(&drdy_edge_count);
        k_sem_give(&drdy_sem);
    } else if (drdy_ready < 0) {
        LOG_WRN("Unable to read DRDY state while priming interrupt wait (%d)", drdy_ready);
    }
}

static int configure_drdy_interrupt(void) {
    int ret;

    if (drdy_interrupt_configured) {
        return 0;
    }

    if (g_adc_config == NULL || g_adc_config->drdy_gpio.port == NULL) {
        LOG_ERR("DRDY GPIO not configured");
        return -ENOTSUP;
    }

    if (!gpio_is_ready_dt(&g_adc_config->drdy_gpio)) {
        LOG_ERR("DRDY GPIO device is not ready");
        return -ENODEV;
    }

    gpio_init_callback(&drdy_gpio_callback, drdy_isr, BIT(g_adc_config->drdy_gpio.pin));

    ret = gpio_add_callback(g_adc_config->drdy_gpio.port, &drdy_gpio_callback);
    if (ret < 0) {
        LOG_ERR("Failed to add DRDY GPIO callback: %d", ret);
        return ret;
    }

    /*
     * DRDY is active-low, so EDGE_TO_ACTIVE maps to the physical falling edge
     * on P1.09 through the GPIO_ACTIVE_LOW flag in the ADS131M04 config.
     */
    ret = gpio_pin_interrupt_configure_dt(&g_adc_config->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        (void)gpio_remove_callback(g_adc_config->drdy_gpio.port, &drdy_gpio_callback);
        LOG_ERR("Failed to enable DRDY GPIO interrupt: %d", ret);
        return ret;
    }

    drdy_interrupt_configured = true;
    LOG_INF("DRDY interrupt configured on P1.09");
    return 0;
}

static inline int read_adc_frame(ads131m04_data_t *adc) {
    int ret;

    k_mutex_lock(&adc_mutex, K_FOREVER);
    ret = ads131m04_read_adc(g_adc_config, adc);
    k_mutex_unlock(&adc_mutex);

    return ret;
}

static int ads131_rate_to_osr(uint32_t sample_rate_hz, uint8_t *osr) {
    if (osr == NULL) {
        return -EINVAL;
    }

    switch (sample_rate_hz) {
        case 32000:
            *osr = ADS131M04_OSR_128;
            return 0;
        case 16000:
            *osr = ADS131M04_OSR_256;
            return 0;
        case 8000:
            *osr = ADS131M04_OSR_512;
            return 0;
        case 4000:
            *osr = ADS131M04_OSR_1024;
            return 0;
        case 2000:
            *osr = ADS131M04_OSR_2048;
            return 0;
        case 1000:
            *osr = ADS131M04_OSR_4096;
            return 0;
        case 500:
            *osr = ADS131M04_OSR_8192;
            return 0;
        case 250:
            *osr = ADS131M04_OSR_16384;
            return 0;
        default:
            return -EINVAL;
    }
}

/**
 * @brief High-priority sampling thread
 * Waits for ADS131M04 DRDY GPIO interrupts, then performs ADC sampling in thread context
 */
static void sampling_thread(void *arg1, void *arg2, void *arg3) {
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    ads131m04_data_t adc;
    LOG_INF("Sampling thread started (priority %d)", SAMPLING_THREAD_PRIORITY);

    while (1) {
        if (!sampling_active || !bluetooth_is_connected()) {
            (void)k_sem_take(&sampling_sem, K_SECONDS(NON_SAMPLING_TIME_S));
            continue;
        }

        int drdy_err = k_sem_take(&drdy_sem, K_USEC(SAMPLING_DRDY_TIMEOUT_US));
        if (drdy_err < 0) {
            missed_samples++;
            adc_read_failures++;
            LOG_WRN("DRDY interrupt wait failed (%u, err %d)", adc_read_failures, drdy_err);
            continue;
        }

        if (!sampling_active || !bluetooth_is_connected()) {
            continue;
        }

        atomic_val_t pending_edges = atomic_set(&drdy_edge_count, 0);
        if (pending_edges > 1) {
            missed_samples += (uint32_t)(pending_edges - 1);
        }

        uint64_t now_cycles = k_cycle_get_64();
        if (last_sample_time_cycles > 0) {
            uint64_t period_us = k_cyc_to_us_floor64(now_cycles - last_sample_time_cycles);
            if (period_us > max_jitter_us) {
                max_jitter_us = period_us;
            }
        }
        last_sample_time_cycles = now_cycles;

        /* Perform ADC sampling (in thread context - SPI can work!) */
        int adc_err = read_adc_frame(&adc);
        if (adc_err < 0) {
            adc_read_failures++;
            LOG_WRN("ADC read failed (%u, err %d)", adc_read_failures, adc_err);
            adc.ch0 = 0;
            adc.ch1 = 0;
            adc.ch2 = 0;
            adc.ch3 = 0;
        }

        /* Send the full ADS131M04 frame to Bluetooth - CRITICAL: Cannot drop samples! */
        (void) bluetooth_queue_sample(adc.ch0, adc.ch1, adc.ch2, adc.ch3, sample_count);
        /* Note: bluetooth.c handles all error logging and warnings */
        sample_count++;
    }
}

/* Define the sampling thread */
K_THREAD_DEFINE(sampling_thread_id, SAMPLING_THREAD_STACK_SIZE,
    sampling_thread, NULL, NULL, NULL,
    SAMPLING_THREAD_PRIORITY, 0, 0);

/* ========== Public API Implementation ========== */

int sampling_init(ads131m04_config_t *adc_config) {
    if (adc_config == NULL) {
        LOG_ERR("ADC configuration is NULL");
        return -EINVAL;
    }

    g_adc_config = adc_config;

    int ret = configure_drdy_interrupt();
    if (ret < 0) {
        return ret;
    }

    LOG_INF("Sampling system initialized (ADS131M04 DRDY interrupt-driven)");

    return 0;
}

int sampling_start(void) {
    if (g_adc_config == NULL) {
        LOG_ERR("Sampling not initialized - call sampling_init() first");
        return -EINVAL;
    }

    adc_read_failures = 0;
    reset_drdy_wait_state();
    prime_drdy_wait_if_ready();

    LOG_INF("Starting DRDY interrupt-driven sampling...");
    sampling_active = true;
    k_sem_give(&sampling_sem);

    LOG_INF("Sampling started successfully");
    return 0;
}

int sampling_stop(void) {
    LOG_INF("Stopping sampling...");
    sampling_active = false;
    k_sem_give(&sampling_sem);
    k_sem_give(&drdy_sem);
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
    LOG_INF("Sampling statistics reset");
}

int sampling_set_ads131_sample_rate_hz(uint32_t sample_rate_hz) {
    uint8_t osr;
    int ret = ads131_rate_to_osr(sample_rate_hz, &osr);
    if (ret < 0) {
        LOG_WRN("Rejected ADS131M04 sample rate %u SPS", sample_rate_hz);
        return ret;
    }

    if (g_adc_config == NULL) {
        LOG_ERR("Sampling not initialized - cannot set ADS131M04 sample rate");
        return -EINVAL;
    }

    k_mutex_lock(&adc_mutex, K_FOREVER);
    ret = ads131m04_set_oversampling(g_adc_config, osr);
    k_mutex_unlock(&adc_mutex);
    if (ret < 0) {
        LOG_ERR("Failed to set ADS131M04 sample rate %u SPS (%d)", sample_rate_hz, ret);
        return ret;
    }

    current_ads131_sample_rate_hz = sample_rate_hz;
    sampling_reset_stats();
    LOG_INF("ADS131M04 sample rate set to %u SPS", sample_rate_hz);
    return 0;
}

uint32_t sampling_get_ads131_sample_rate_hz(void) {
    return current_ads131_sample_rate_hz;
}

void sampling_restore_fast_rate(void) {
    reset_drdy_wait_state();
    prime_drdy_wait_if_ready();
    k_sem_give(&sampling_sem);
    LOG_INF("DRDY interrupt-driven sampling resumed");
}
