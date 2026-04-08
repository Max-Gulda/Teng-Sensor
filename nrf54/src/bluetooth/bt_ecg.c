/**
 * @file bt_ecg.c
 * @brief Main ADC Bluetooth channel: queue, batching, DLQ, and stats.
 */

/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jesper Sjöberg (@repsejs, KTH: @jessjobe)
 * Copyright (c) 2026 Max Gulda (@Max-Gulda, KTH: @gulda)
 */

#include "bt_ecg.h"
#include "bt_ecg_raw.h"
#include "bt_channel.h"
#include "bluetooth.h"
#include "define.h"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(bluetooth, LOG_LEVEL_INF);

/* ========== Static State ========== */

static volatile bool data_notify_enabled = false;
static adc_data_t adc_data_value = { 0 };
static adc_data_t adc_batch_buffer[BT_BATCH_SIZE];
static uint8_t ecg_batch_count = 0;
static int64_t ecg_last_batch_time = 0;
static bt_chan_stats_t ecg_stats = { 0 };

#define DLQ_SIZE 3
K_MSGQ_DEFINE(bt_data_queue, sizeof(bt_sample_data_t), BT_DATA_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(ecg_dlq, sizeof(bt_failed_batch_t), DLQ_SIZE, 4);

bt_channel_t ecg_channel = {
    .name = "ADC",
    .gatt_attr_idx = 1,
    .notify_enabled = &data_notify_enabled,
    .current_value = &adc_data_value,
    .queue = &bt_data_queue,
    .queue_size = BT_DATA_QUEUE_SIZE,
    .queue_warn_level = BT_QUEUE_WARN_LEVEL,
    .queue_timeout_ms = BT_QUEUE_TIMEOUT_MS,
    .batch_buf = adc_batch_buffer,
    .batch_count = &ecg_batch_count,
    .last_batch_time = &ecg_last_batch_time,
    .max_batch_size = BT_BATCH_SIZE,
    .batch_timeout_ms = BT_BATCH_TIMEOUT_MS,
    .batch_elem_size = sizeof(adc_data_t),
    .dlq = &ecg_dlq,
    .dlq_size = DLQ_SIZE,
    .stale_age = BT_DATA_QUEUE_SIZE,
    .stats = &ecg_stats,
};

static int32_t get_ecg_source_sample(const bt_sample_data_t *sample) {
    switch (ECG_SOURCE_ADC_CHANNEL) {
        case 0:
            return sample->ch0;
        case 1:
            return sample->ch1;
        case 2:
            return sample->ch2;
        case 3:
            return sample->ch3;
        default:
            return sample->ch0;
    }
}

/* ========== GATT Callbacks ========== */

void data_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    data_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("ADC data notifications %s", data_notify_enabled ? "enabled" : "disabled");
}

ssize_t read_ecg_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
    void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
        &adc_data_value, sizeof(adc_data_value));
}

/* ========== Channel Helpers ========== */

void bt_ecg_process_sample(const bt_sample_data_t *sample) {
    channel_retry_failed(&ecg_channel, sample->timestamp);

    adc_data_t *slot = adc_batch_buffer + ecg_batch_count;
    slot->sample_count = sample->timestamp;
    slot->ch0 = sample->ch0;
    slot->ch1 = sample->ch1;
    slot->ch2 = sample->ch2;
    slot->ch3 = sample->ch3;
    ecg_batch_count++;

    if (ecg_batch_count >= BT_BATCH_SIZE) {
        channel_send_batch(&ecg_channel);
    }
}

void bt_ecg_flush_if_timeout(void) {
    int64_t now = k_uptime_get();
    if (ecg_batch_count > 0 && (now - ecg_last_batch_time) >= BT_BATCH_TIMEOUT_MS) {
        channel_send_batch(&ecg_channel);
    }
}

void bt_ecg_get_stats(bt_channel_stats_t *out) {
    out->samples_sent = ecg_stats.samples_sent;
    out->queue_overflows = ecg_stats.queue_overflows;
    out->max_queue_used = ecg_stats.max_queue_used;
    out->current_queue_used = k_msgq_num_used_get(&bt_data_queue);
    out->batch_retries = ecg_stats.batch_retries;
    out->samples_discarded_stale = ecg_stats.samples_discarded_stale;
    out->dlq_overflows = ecg_stats.dlq_overflows;
    out->dlq_count = k_msgq_num_used_get(&ecg_dlq);
}

void bt_ecg_reset_stats(void) {
    ecg_stats = (bt_chan_stats_t){ 0 };
}

void bt_ecg_reset_on_connect(void) {
    ecg_batch_count = 0;
    ecg_last_batch_time = k_uptime_get();
    k_msgq_purge(&ecg_dlq);
}

void bt_ecg_on_disconnect(void) {
    data_notify_enabled = false;
}

/* ========== Public API ========== */

int bluetooth_queue_sample(int32_t ch0, int32_t ch1, int32_t ch2, int32_t ch3, uint32_t timestamp) {
    bt_sample_data_t sample = {
        .ch0 = ch0,
        .ch1 = ch1,
        .ch2 = ch2,
        .ch3 = ch3,
        .timestamp = timestamp,
    };
    int err = channel_queue_put(&ecg_channel, &sample);
#if BT_ENABLE_RAW_ECG_CHAR
    if (err == 0) {
        (void) bluetooth_queue_raw_ecg_sample(get_ecg_source_sample(&sample), timestamp);
    }
#endif
    return err;
}
