/**
 * @file bt_ecg_raw.c
 * @brief Optional raw ECG Bluetooth channel: queue, batching, DLQ, and stats.
 */

#include "bt_ecg_raw.h"
#include "bt_channel.h"
#include "bluetooth.h"
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(bluetooth, LOG_LEVEL_INF);

#if BT_ENABLE_RAW_ECG_CHAR

static volatile bool raw_ecg_notify_enabled = false;
static ecg_raw_data_t ecg_raw_value = { 0 };
static ecg_raw_data_t ecg_raw_batch_buffer[BT_ECG_RAW_BATCH_SIZE];
static uint8_t ecg_raw_batch_count = 0;
static int64_t ecg_raw_last_batch_time = 0;
static bt_chan_stats_t ecg_raw_stats = { 0 };

#define ECG_RAW_DLQ_SIZE 3
K_MSGQ_DEFINE(bt_raw_ecg_queue, sizeof(bt_ecg_raw_sample_data_t), BT_ECG_RAW_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(ecg_raw_dlq, sizeof(bt_failed_batch_t), ECG_RAW_DLQ_SIZE, 4);

bt_channel_t ecg_raw_channel = {
    .name = "ECG_RAW",
    .gatt_attr_idx = 7,
    .notify_enabled = &raw_ecg_notify_enabled,
    .current_value = &ecg_raw_value,
    .queue = &bt_raw_ecg_queue,
    .queue_size = BT_ECG_RAW_QUEUE_SIZE,
    .queue_warn_level = BT_ECG_RAW_QUEUE_WARN_LEVEL,
    .queue_timeout_ms = BT_ECG_RAW_QUEUE_TIMEOUT_MS,
    .batch_buf = ecg_raw_batch_buffer,
    .batch_count = &ecg_raw_batch_count,
    .last_batch_time = &ecg_raw_last_batch_time,
    .max_batch_size = BT_ECG_RAW_BATCH_SIZE,
    .batch_timeout_ms = BT_ECG_RAW_BATCH_TIMEOUT_MS,
    .batch_elem_size = sizeof(ecg_raw_data_t),
    .dlq = &ecg_raw_dlq,
    .dlq_size = ECG_RAW_DLQ_SIZE,
    .stale_age = BT_ECG_RAW_QUEUE_SIZE,
    .stats = &ecg_raw_stats,
};

void raw_ecg_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    raw_ecg_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Raw ECG notifications %s", raw_ecg_notify_enabled ? "enabled" : "disabled");
}

ssize_t read_ecg_raw_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
    void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
        &ecg_raw_value, sizeof(ecg_raw_value));
}

void bt_ecg_raw_process_sample(const bt_ecg_raw_sample_data_t *sample) {
    channel_retry_failed(&ecg_raw_channel, sample->timestamp);

    ecg_raw_data_t *slot = ecg_raw_batch_buffer + ecg_raw_batch_count;
    slot->sample_count = sample->timestamp;
    slot->heart = sample->heart;
    ecg_raw_batch_count++;

    if (ecg_raw_batch_count >= BT_ECG_RAW_BATCH_SIZE) {
        channel_send_batch(&ecg_raw_channel);
    }
}

void bt_ecg_raw_flush_if_timeout(void) {
    int64_t now = k_uptime_get();
    if (ecg_raw_batch_count > 0 &&
        (now - ecg_raw_last_batch_time) >= BT_ECG_RAW_BATCH_TIMEOUT_MS) {
        channel_send_batch(&ecg_raw_channel);
    }
}

void bt_ecg_raw_reset_on_connect(void) {
    ecg_raw_batch_count = 0;
    ecg_raw_last_batch_time = k_uptime_get();
    k_msgq_purge(&ecg_raw_dlq);
}

void bt_ecg_raw_on_disconnect(void) {
    raw_ecg_notify_enabled = false;
}

int bluetooth_queue_raw_ecg_sample(int32_t heart, uint32_t timestamp) {
    bt_ecg_raw_sample_data_t sample = {
        .heart = heart,
        .timestamp = timestamp,
    };
    return channel_queue_put(&ecg_raw_channel, &sample);
}

#else

void raw_ecg_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    ARG_UNUSED(value);
}

ssize_t read_ecg_raw_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
    void *buf, uint16_t len, uint16_t offset) {
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(buf);
    ARG_UNUSED(len);
    ARG_UNUSED(offset);
    return 0;
}

void bt_ecg_raw_process_sample(const bt_ecg_raw_sample_data_t *sample) {
    ARG_UNUSED(sample);
}

void bt_ecg_raw_flush_if_timeout(void) {}

void bt_ecg_raw_reset_on_connect(void) {}

void bt_ecg_raw_on_disconnect(void) {}

int bluetooth_queue_raw_ecg_sample(int32_t heart, uint32_t timestamp) {
    ARG_UNUSED(heart);
    ARG_UNUSED(timestamp);
    return 0;
}

#endif
