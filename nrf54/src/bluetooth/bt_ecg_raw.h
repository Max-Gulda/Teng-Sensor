/**
 * @file bt_ecg_raw.h
 * @brief Optional raw ECG Bluetooth channel — internal header for src/bluetooth/.
 */

#ifndef BT_ECG_RAW_H
#define BT_ECG_RAW_H

#include "bt_channel.h"
#include "bt_ecg.h"
#include "bluetooth.h"
#include <stdint.h>
#include <zephyr/bluetooth/gatt.h>

#define BT_ECG_RAW_QUEUE_SIZE       512
#define BT_ECG_RAW_QUEUE_WARN_LEVEL 384
#define BT_ECG_RAW_QUEUE_TIMEOUT_MS 100
#define BT_ECG_RAW_BATCH_SIZE       20
#define BT_ECG_RAW_BATCH_TIMEOUT_MS 50

typedef struct {
    int32_t heart;
    uint32_t timestamp;
} bt_ecg_raw_sample_data_t;

typedef struct {
    uint32_t sample_count;
    int32_t heart;
} __attribute__((packed)) ecg_raw_data_t;

#if BT_ENABLE_RAW_ECG_CHAR
extern bt_channel_t ecg_raw_channel;
#endif

void raw_ecg_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
ssize_t read_ecg_raw_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
    void *buf, uint16_t len, uint16_t offset);

void bt_ecg_raw_process_sample(const bt_ecg_raw_sample_data_t *sample);
void bt_ecg_raw_flush_if_timeout(void);
void bt_ecg_raw_reset_on_connect(void);
void bt_ecg_raw_on_disconnect(void);
int bluetooth_queue_raw_ecg_sample(int32_t heart, uint32_t timestamp);

#endif /* BT_ECG_RAW_H */
