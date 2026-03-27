/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jesper Sjöberg (@repsejs, KTH: @jessjobe)
 * Copyright (c) 2026 Max Gulda (@Max-Gulda, KTH: @gulda)
 */

#include <zephyr/kernel.h>
#include "ads131m02_spi.h"
#include "lsm6dso_spi.h"
#include "sampling.h"
#include "bluetooth.h"
#include "ble_log_backend.h"
#include "ecg.h"
#include <zephyr/logging/log.h>
#include <zephyr/debug/cpu_load.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void) {
    int err;
    ads131m02_config_t *adc_config;
    lsm6dso_config_t *imu_config;

    LOG_INF("=== ADS131M02 Task-Based Sampling @ %d Hz ===", SAMPLING_FREQUENCY_HZ);
    
    /* ------------ INIT BLUETOOTH START ------------*/

    /* Initialize Bluetooth */
    err = bluetooth_init();
    if (err < 0) {
        LOG_ERR("Failed to initialize Bluetooth (%d)", err);
        return 0;
    }

    /* Start Bluetooth advertising */
    err = bluetooth_start();
    if (err < 0) {
        LOG_ERR("Failed to start Bluetooth (%d)", err);
        return 0;
    }

    /* Enable BLE log backend — messages stream to any NUS-connected terminal */
    ble_log_backend_enable();

    //k_msleep(20000);
    /* ------------ INIT BLUETOOTH END ------------*/

    /* ------------ INIT ADS131M02 START -------------*/
    /* Get ADC configuration */
    adc_config = ads131m02_get_config();

    /* Perform full ADC setup */
    err = ads131m02_full_setup(adc_config);
    if (err < 0) {
        LOG_ERR("Failed to setup ADC");
        return 0;
    }

    /* ------------ INIT ADS131M02 END --------------*/

    /* ------------ INIT LSM6DSO START -------------*/

    /* Get IMU configuration */
    imu_config = lsm6dso_get_config();

    /* Perform full IMU setup */
    err = lsm6dso_full_setup(imu_config);
    if (err < 0) {
        LOG_ERR("Failed to setup IMU (%d)", err);
        return 0;
    }

    /* ------------ INIT LSM6DSO END ---------------*/

    /* ------------ INIT ECG START ------------*/
    err = ecg_init();
    if (err < 0) {
        LOG_ERR("Failed to initialize ecg (%d)", err);
        return 0;
    }

    err = ecg_start();
    if (err < 0) {
        LOG_ERR("Failed to start ecg (%d)", err);
        return 0;
    }

    /* ------------ INIT ECG END ------------*/


    /* ------------ INIT SAMPLING START ------------*/

    /* Initialize sampling system */
    err = sampling_init(adc_config);
    if (err < 0) {
        LOG_ERR("Failed to initialize sampling system");
        return 0;
    }

    /* Start sampling */
    err = sampling_start();
    if (err < 0) {
        LOG_ERR("Failed to start sampling");
        return 0;
    }

    /* ------------ INIT BLUETOOTH END ------------*/
    int cpu_load = 0;

    /* Main thread monitors CPU load */
    while (1) {
        k_msleep(10000);
        cpu_load = cpu_load_get(false);
        if (cpu_load >= 0) {
            LOG_INF("CPU Usage: %d.%d%%", cpu_load / 10, cpu_load % 10);
        } else {
            LOG_ERR("CPU Usage: Error %d", cpu_load);
        }
    }

    return 0;
}
