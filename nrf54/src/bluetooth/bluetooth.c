/**
 * @file bluetooth.c
 * @brief Bluetooth core: GATT service, connection management, advertising, thread.
 *
 * Channel-specific logic lives in bt_ecg.c.
 * Generic pipeline (batching, DLQ, notify) lives in bt_channel.c.
 *
 * Adding a new data channel requires:
 *   1. Define packed sample struct + batch buffer in a new bt_<name>.c
 *   2. K_MSGQ_DEFINE for input queue and DLQ
 *   3. Fill a bt_channel_t descriptor
 *   4. Add GATT characteristic to ecg_service below
 *   5. Wire process/flush helpers into bluetooth_thread
 */

/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jesper Sjöberg (@repsejs, KTH: @jessjobe)
 * Copyright (c) 2026 Max Gulda (@Max-Gulda, KTH: @gulda)
 */

#include "bluetooth.h"
#include "bt_ecg.h"
#include "sampling.h"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <bluetooth/services/nus.h>
#include <stdlib.h>
#include <string.h>

LOG_MODULE_REGISTER(bluetooth, LOG_LEVEL_INF);

/* Compile-time checks: every channel batch must fit inside BT_MAX_BATCH_PAYLOAD. */
_Static_assert(BT_BATCH_SIZE * sizeof(adc_data_t) <= BT_MAX_BATCH_PAYLOAD,
    "ADC batch payload exceeds BT_MAX_BATCH_PAYLOAD");
/* ========== UUIDs ========== */

/* Custom 128-bit UUIDs for ADC service */
#define BT_UUID_ECG_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define BT_UUID_ECG_DATA_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

#define BT_UUID_ECG_SERVICE    BT_UUID_DECLARE_128(BT_UUID_ECG_SERVICE_VAL)
#define BT_UUID_ECG_DATA       BT_UUID_DECLARE_128(BT_UUID_ECG_DATA_VAL)

/* ========== Connection State ========== */

/* Non-static: extern'd by bt_channel.c for fast connection-state checks */
volatile bool connected = false;
static atomic_t active_conn_count = ATOMIC_INIT(0);

static volatile uint32_t bt_disconnects = 0;

/* ========== Advertising Data ========== */

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    /* Custom ADC service UUID */
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_ECG_SERVICE_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ========== NUS Command Handling ========== */

static void handle_ads131_rate_command(const char *value) {
    char *end = NULL;
    unsigned long sample_rate_hz = strtoul(value, &end, 10);

    if (end == value || sample_rate_hz > UINT32_MAX) {
        LOG_WRN("Invalid ads131_rate command value: %s", value);
        return;
    }

    int err = sampling_set_ads131_sample_rate_hz((uint32_t)sample_rate_hz);
    if (err < 0) {
        LOG_WRN("Failed to set ADS131M04 sample rate to %lu SPS (err %d)",
            sample_rate_hz, err);
    }
}

static void nus_received(struct bt_conn *conn, const uint8_t *const data, uint16_t len) {
    ARG_UNUSED(conn);

    char command[64];
    size_t copy_len = MIN(len, sizeof(command) - 1);
    memcpy(command, data, copy_len);
    command[copy_len] = '\0';

    while (copy_len > 0 && (command[copy_len - 1] == '\n' || command[copy_len - 1] == '\r')) {
        command[--copy_len] = '\0';
    }

    if (strncmp(command, "ads131_rate ", strlen("ads131_rate ")) == 0) {
        handle_ads131_rate_command(command + strlen("ads131_rate "));
    } else {
        LOG_WRN("Unknown NUS command: %s", command);
    }
}

/* ========== Advertising Restart ========== */

static void restart_advertising_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(restart_adv_work, restart_advertising_work_handler);

static void restart_advertising_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    int conn_count = atomic_get(&active_conn_count);
    if (conn_count >= CONFIG_BT_MAX_CONN) {
        LOG_DBG("Skip advertising restart (%d/%d links active)", conn_count, CONFIG_BT_MAX_CONN);
        return;
    }

    struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
        BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY,
        BT_GAP_ADV_FAST_INT_MIN_2,
        BT_GAP_ADV_FAST_INT_MAX_2,
        NULL
    );

    int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err == -EALREADY) {
        LOG_DBG("Advertising already running");
    } else if (err) {
        LOG_ERR("Failed to restart advertising (err %d)", err);
    } else {
        LOG_INF("Advertising restarted, ready for new connections");
    }
}

/* ========== Connection Callbacks ========== */

static void connected_callback(struct bt_conn *conn, uint8_t err) {
    struct bt_conn_info info;

    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }

    k_work_cancel_delayable(&restart_adv_work);

    int conn_count = atomic_inc(&active_conn_count) + 1;
    connected = true;

    bt_conn_get_info(conn, &info);
    LOG_INF("Connected (interval=%u, latency=%u, timeout=%u, links=%d/%d)",
        info.le.interval, info.le.latency, info.le.timeout,
        conn_count, CONFIG_BT_MAX_CONN);

    if (conn_count == 1) {
        bt_ecg_reset_on_connect();
        sampling_restore_fast_rate();
    }

    if (conn_count < CONFIG_BT_MAX_CONN) {
        k_work_schedule(&restart_adv_work, K_NO_WAIT);
    }
}

static void disconnected_callback(struct bt_conn *conn, uint8_t reason) {
    ARG_UNUSED(conn);

    int conn_count = atomic_get(&active_conn_count);
    if (conn_count > 0) {
        conn_count = atomic_dec(&active_conn_count) - 1;
    } else {
        conn_count = 0;
    }

    connected = (conn_count > 0);
    LOG_INF("Disconnected (reason %u, links=%d/%d)",
        reason, conn_count, CONFIG_BT_MAX_CONN);

    bt_disconnects++;

    if (conn_count == 0) {
        bt_ecg_on_disconnect();
    }

    if (conn_count < CONFIG_BT_MAX_CONN) {
        k_work_schedule(&restart_adv_work, K_MSEC(100));
        LOG_INF("Scheduling advertising restart in 100ms...");
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected_callback,
    .disconnected = disconnected_callback,
};

/* ========== GATT Service Definitions ========== */

/*
 * Custom ADC service attribute indices:
 *   [0]  Primary Service
 *   [1]  ADC Char Declaration  -> ecg_channel.gatt_attr_idx = 1
 *   [2]  ADC Char Value
 *   [3]  ADC CCC
 */
BT_GATT_SERVICE_DEFINE(ecg_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_ECG_SERVICE),

    BT_GATT_CHARACTERISTIC(BT_UUID_ECG_DATA,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        read_ecg_data, NULL, NULL),
    BT_GATT_CCC(data_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

/* ========== Bluetooth Thread ========== */

/**
 * @brief Bluetooth transmission thread.
 *
 * Blocks on the ADC queue with a timeout and flushes partial batches.
 */
static void bluetooth_thread(void *arg1, void *arg2, void *arg3) {
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    LOG_INF("Bluetooth thread started (priority %d, adc_batch=%d)",
        BT_THREAD_PRIORITY, BT_BATCH_SIZE);

    /* Initialise batch timers before entering the loop */
    bt_ecg_reset_on_connect();

    while (1) {
        /* Block on ADC queue with timeout */
        bt_sample_data_t adc_sample;
        int err = k_msgq_get(ecg_channel.queue, &adc_sample, K_MSEC(BT_BATCH_TIMEOUT_MS));

        if (err == 0) {
            bt_ecg_process_sample(&adc_sample);
        } else if (err == -EAGAIN) {
            bt_ecg_flush_if_timeout();
        } else {
            LOG_ERR("Failed to get ADC sample from queue: %d", err);
        }
    }
}

K_THREAD_DEFINE(bluetooth_thread_id, BT_THREAD_STACK_SIZE,
    bluetooth_thread, NULL, NULL, NULL,
    BT_THREAD_PRIORITY, 0, 0);

/* ========== Public API ========== */

int bluetooth_init(void) {
    LOG_INF("Initializing Bluetooth...");

    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }

    static struct bt_nus_cb nus_cb = {
        .received = nus_received,
    };
    err = bt_nus_init(&nus_cb);
    if (err) {
        LOG_ERR("NUS init failed (err %d)", err);
        return err;
    }

    LOG_INF("Bluetooth initialized successfully");
    return 0;
}

int bluetooth_start(void) {
    struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
        BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY,
        BT_GAP_ADV_FAST_INT_MIN_2,
        BT_GAP_ADV_FAST_INT_MAX_2,
        NULL
    );

    LOG_INF("Starting Bluetooth advertising...");

    int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return err;
    }

    LOG_INF("Bluetooth advertising started");
    LOG_INF("Device name: %s", CONFIG_BT_DEVICE_NAME);
    return 0;
}

int bluetooth_stop(void) {
    LOG_INF("Stopping Bluetooth advertising...");

    int err = bt_le_adv_stop();
    if (err) {
        LOG_ERR("Failed to stop advertising (err %d)", err);
        return err;
    }

    LOG_INF("Bluetooth advertising stopped");
    return 0;
}

void bluetooth_get_stats(bt_stats_t *stats) {
    if (!stats) {
        return;
    }

    bt_ecg_get_stats(&stats->adc);
    stats->bt_disconnects = bt_disconnects;
    stats->connected = connected;
}

void bluetooth_print_stats(bt_stats_t *stats) {
    if (!stats) {
        return;
    }

    LOG_INF("------ BT STATS ------");
    LOG_INF("  ADC sent: %u", stats->adc.samples_sent);
    LOG_INF("  ADC overflow: %u", stats->adc.queue_overflows);
    LOG_INF("  ADC retries: %u", stats->adc.batch_retries);
    LOG_INF("  ADC stale: %u", stats->adc.samples_discarded_stale);
    LOG_INF("  ADC dlq: %u/%u overflows", stats->adc.dlq_count, stats->adc.dlq_overflows);
    LOG_INF("  ADC queue: %u/%u", stats->adc.current_queue_used, stats->adc.max_queue_used);
    LOG_INF("  disconnects: %u", stats->bt_disconnects);
    LOG_INF("  connected: %d", stats->connected);
    LOG_INF("----------------------");
}

void bluetooth_reset_stats(void) {
    bt_ecg_reset_stats();
    LOG_INF("Bluetooth statistics reset");
}

bool bluetooth_is_connected(void) {
    return connected;
}
