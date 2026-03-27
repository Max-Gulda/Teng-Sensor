/**
 * @file bt_imu.h
 * @brief IMU Bluetooth channel — internal header for src/bluetooth/.
 */

#ifndef BT_IMU_H
#define BT_IMU_H

#include "bt_channel.h"
#include "bluetooth.h"
#include <zephyr/bluetooth/gatt.h>
#include <stdint.h>

 /* IMU queue and batching configuration */
#define BT_IMU_QUEUE_SIZE        512    /* Buffer 512 IMU samples */
#define BT_IMU_QUEUE_WARN_LEVEL  384    /* Warn when 75% full */
#define BT_IMU_QUEUE_TIMEOUT_MS  100    /* Max wait time if queue full */
#define BT_IMU_BATCH_SIZE        15     /* 15 samples × 16 bytes = 240 bytes, fits in MTU 247 */
#define BT_IMU_BATCH_TIMEOUT_MS  50     /* Max time to wait for a full batch */

/* IMU sample queue entry — filled by the sampling thread */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint32_t timestamp;
} bt_imu_sample_data_t;

/* IMU packed BLE payload (16 bytes per sample) */
typedef struct {
    uint32_t sample_count;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} __attribute__((packed)) imu_data_t;

/* Channel descriptor — used by bluetooth.c thread loop */
extern bt_channel_t imu_channel;

/* GATT callbacks — non-static, referenced by BT_GATT_SERVICE_DEFINE in bluetooth.c */
void imu_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
ssize_t read_imu_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
    void *buf, uint16_t len, uint16_t offset);

/* Channel helpers — called by bluetooth.c */
void bt_imu_process_sample(const bt_imu_sample_data_t *sample);
void bt_imu_flush_if_timeout(void);
void bt_imu_get_stats(bt_channel_stats_t *out);
void bt_imu_reset_stats(void);
void bt_imu_reset_on_connect(void);
void bt_imu_on_disconnect(void);

#endif /* BT_IMU_H */
