/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jesper Sjöberg (@repsejs, KTH: @jessjobe)
 * Copyright (c) 2026 Max Gulda (@Max-Gulda, KTH: @gulda)
 */

#include <zephyr/kernel.h>
#include "ecg.h"
#include "pan_tompkins.h"
#include "define.h"
#include <zephyr/logging/log.h>
#include "bluetooth.h"

LOG_MODULE_REGISTER(ecg, LOG_LEVEL_INF);

K_MSGQ_DEFINE(ecg_data_queue, sizeof(int32_t), MSGQ_SIZE, 4);

int ecg_queue_sample(int32_t heart) {
    int err;

    err = k_msgq_put(&ecg_data_queue, &heart, K_MSEC(ECG_QUEUE_TIMEOUT_MS));
    if (err != 0) {
        LOG_ERR("CRITICAL: ECG queue full! First sample dropped. ECG thread may be blocked.");
        return -ENOMEM;
    }

    return 0;
}

static void ecg_thread(void *arg1, void *arg2, void *arg3) {
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    int32_t heart = 0;
    int err = 0;
    uint8_t hr = 0;

    LOG_INF("ECG thread started (priority %d)", ECG_THREAD_PRIORITY);

    while (1) {
        err = k_msgq_get(&ecg_data_queue, &heart, K_MSEC(ECG_QUEUE_TIMEOUT_MS));
        if (err == 0) {
            hr = pt_process_sample(heart);

            if (hr > 0) {
                uint16_t rr_ms = pt_get_last_rr_ms();
                LOG_DBG("HR: %u bpm", hr);
                (void) bluetooth_notify_hrv(rr_ms);
                if (rr_ms > 0) {
                    LOG_DBG("HRV RR: %u ms", rr_ms);
                }
                (void) bluetooth_notify_heart_rate(hr);
            }
        } else if (err == -EAGAIN) {
            /* timeout - no samples, nothing to do */
        } else {
            LOG_ERR("ECG queue error: %d", err);
        }
    }

}

K_THREAD_DEFINE(ecg_thread_id, ECG_THREAD_STACK_SIZE,
    ecg_thread, NULL, NULL, NULL,
    ECG_THREAD_PRIORITY, 0, 0);


int ecg_init(void) {
    int8_t err;

    LOG_INF("Initializing ECG...");

    err = pt_init(ECG_BP_ORDER, SAMPLING_RATE, ECG_LOW_CUTOFF, ECG_HIGH_CUTOFF, ECG_INTEGRATION_TIME_MS, ECG_SIZE);
    if (err < 0) {
        LOG_ERR("pt init failed (err %d)", err);
        return err;
    }
    (void) bluetooth_notify_hrv(0);
    LOG_INF("ECG initialized successfully");
    return 0;
}

int ecg_start(void) {
    return 0;
}
