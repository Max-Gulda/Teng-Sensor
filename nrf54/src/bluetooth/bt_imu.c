/**
 * @file bt_imu.c
 * @brief IMU Bluetooth channel: queue, batching, DLQ, and stats.
 */

#include "bt_imu.h"
#include "bt_channel.h"
#include "bluetooth.h"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(bluetooth, LOG_LEVEL_INF);

/* ========== Static State ========== */

static volatile bool imu_notify_enabled = false;
static imu_data_t imu_data_value = { 0 };
static imu_data_t imu_batch_buffer[BT_IMU_BATCH_SIZE];
static uint8_t imu_batch_count = 0;
static int64_t imu_last_batch_time = 0;
static bt_chan_stats_t imu_stats = { 0 };

#define IMU_DLQ_SIZE 3
K_MSGQ_DEFINE(bt_imu_queue, sizeof(bt_imu_sample_data_t), BT_IMU_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(imu_dlq, sizeof(bt_failed_batch_t), IMU_DLQ_SIZE, 4);

bt_channel_t imu_channel = {
    .name = "IMU",
    .gatt_attr_idx = 4,
    .notify_enabled = &imu_notify_enabled,
    .current_value = &imu_data_value,
    .queue = &bt_imu_queue,
    .queue_size = BT_IMU_QUEUE_SIZE,
    .queue_warn_level = BT_IMU_QUEUE_WARN_LEVEL,
    .queue_timeout_ms = BT_IMU_QUEUE_TIMEOUT_MS,
    .batch_buf = imu_batch_buffer,
    .batch_count = &imu_batch_count,
    .last_batch_time = &imu_last_batch_time,
    .max_batch_size = BT_IMU_BATCH_SIZE,
    .batch_timeout_ms = BT_IMU_BATCH_TIMEOUT_MS,
    .batch_elem_size = sizeof(imu_data_t),
    .dlq = &imu_dlq,
    .dlq_size = IMU_DLQ_SIZE,
    .stale_age = BT_IMU_QUEUE_SIZE,
    .stats = &imu_stats,
};

/* ========== GATT Callbacks ========== */

void imu_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    imu_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("IMU data notifications %s", imu_notify_enabled ? "enabled" : "disabled");
}

ssize_t read_imu_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
    void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
        &imu_data_value, sizeof(imu_data_value));
}

/* ========== Channel Helpers ========== */

void bt_imu_process_sample(const bt_imu_sample_data_t *sample) {
    channel_retry_failed(&imu_channel, sample->timestamp);

    imu_data_t *slot = imu_batch_buffer + imu_batch_count;
    slot->sample_count = sample->timestamp;
    slot->accel_x = sample->accel_x;
    slot->accel_y = sample->accel_y;
    slot->accel_z = sample->accel_z;
    slot->gyro_x = sample->gyro_x;
    slot->gyro_y = sample->gyro_y;
    slot->gyro_z = sample->gyro_z;
    imu_batch_count++;

    if (imu_batch_count >= BT_IMU_BATCH_SIZE) {
        channel_send_batch(&imu_channel);
    }
}

void bt_imu_flush_if_timeout(void) {
    int64_t now = k_uptime_get();
    if (imu_batch_count > 0 && (now - imu_last_batch_time) >= BT_IMU_BATCH_TIMEOUT_MS) {
        channel_send_batch(&imu_channel);
    }
}

void bt_imu_get_stats(bt_channel_stats_t *out) {
    out->samples_sent = imu_stats.samples_sent;
    out->queue_overflows = imu_stats.queue_overflows;
    out->max_queue_used = imu_stats.max_queue_used;
    out->current_queue_used = k_msgq_num_used_get(&bt_imu_queue);
    out->batch_retries = imu_stats.batch_retries;
    out->samples_discarded_stale = imu_stats.samples_discarded_stale;
    out->dlq_overflows = imu_stats.dlq_overflows;
    out->dlq_count = k_msgq_num_used_get(&imu_dlq);
}

void bt_imu_reset_stats(void) {
    imu_stats = (bt_chan_stats_t){ 0 };
}

void bt_imu_reset_on_connect(void) {
    imu_batch_count = 0;
    imu_last_batch_time = k_uptime_get();
    k_msgq_purge(&imu_dlq);
}

void bt_imu_on_disconnect(void) {
    imu_notify_enabled = false;
}

/* ========== Public API ========== */

int bluetooth_queue_imu_sample(int16_t ax, int16_t ay, int16_t az,
    int16_t gx, int16_t gy, int16_t gz, uint32_t timestamp) {
    bt_imu_sample_data_t sample = {
        .accel_x = ax, .accel_y = ay, .accel_z = az,
        .gyro_x = gx, .gyro_y = gy, .gyro_z = gz,
        .timestamp = timestamp,
    };
    return channel_queue_put(&imu_channel, &sample);
}
