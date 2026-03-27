/**
 * @file bt_ecg.h
 * @brief Primary ECG Bluetooth channel — internal header.
 */

#ifndef BT_ECG_H
#define BT_ECG_H

#include "bt_channel.h"
#include "bluetooth.h"
#include <zephyr/bluetooth/gatt.h>
#include <stdint.h>

/*
 * Build-time switch for optional raw ECG characteristic.
 * 0: send only primary ECG channel on the custom data characteristic (default).
 * 1: also expose a separate raw ECG characteristic.
 */
#define BT_ENABLE_RAW_ECG_CHAR 1

 /* Primary ECG queue and batching configuration */
#define BT_DATA_QUEUE_SIZE   512    /* Buffer 512 samples = 10.24 seconds @ 50Hz */
#define BT_QUEUE_WARN_LEVEL  384    /* Warn when 75% full */
#define BT_QUEUE_TIMEOUT_MS  100    /* Max wait time if queue full */
#define BT_BATCH_SIZE        20     /* 20 samples × 8 bytes = 160 bytes */
#define BT_BATCH_TIMEOUT_MS  50     /* Max time to wait for a full batch */

/* Sample queue entry — filled by the sampling thread */
typedef struct {
    int32_t heart;        /* Raw ECG sample (optional BLE characteristic) */
    int32_t ecg_aux;      /* Secondary ADC channel (default BLE payload) */
    uint32_t timestamp;   /* Sample number */
} bt_sample_data_t;

/* Primary ECG packed BLE payload (8 bytes per sample) */
typedef struct {
    uint32_t sample_count;
    int32_t ecg_aux;
} __attribute__((packed)) ecg_data_t;

/* Channel descriptor — used by bluetooth.c thread loop */
extern bt_channel_t ecg_channel;

/* GATT callbacks — non-static, referenced by BT_GATT_SERVICE_DEFINE in bluetooth.c */
void data_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
ssize_t read_ecg_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
    void *buf, uint16_t len, uint16_t offset);

/* Channel helpers — called by bluetooth.c */
void bt_ecg_process_sample(const bt_sample_data_t *sample);
void bt_ecg_flush_if_timeout(void);
void bt_ecg_get_stats(bt_channel_stats_t *out);
void bt_ecg_reset_stats(void);
void bt_ecg_reset_on_connect(void);
void bt_ecg_on_disconnect(void);

#endif /* BT_ECG_H */
