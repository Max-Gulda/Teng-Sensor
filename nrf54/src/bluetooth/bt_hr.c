/**
 * @file bt_hr.c
 * @brief Bluetooth Heart Rate Service (0x180D) implementation.
 *
 * Implements the Heart Rate Measurement characteristic (0x2A37) per the
 * Bluetooth GATT Heart Rate Service specification v1.0.
 *
 * Packet format (Heart Rate Measurement value):
 *   Byte 0: Flags
 *     Bit 0: HR Value Format    — 0 = uint8
 *     Bit 1-2: Sensor Contact   — 0b00 = not supported
 *     Bit 3: Energy Expended    — 0 = not present
 *     Bit 4: RR-Interval        — 1 = present (when available)
 *   Byte 1: Heart Rate (uint8, BPM)
 *   Bytes 2-3: RR-Interval (uint16, units of 1/1024 s) — present when Bit 4 set
 */

#include "bt_hr.h"
#include "bt_channel.h"
#include "bluetooth.h"
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(bluetooth, LOG_LEVEL_INF);

/* ========== Static State ========== */

static volatile bool hr_notify_enabled = false;
static uint8_t last_hr_bpm = 0;
static uint16_t last_rr_ms = 0;          /* latest RR interval, 0 if unavailable */

/* Body Sensor Location value: 1 = Chest (BT spec Table 3.4) */
static const uint8_t body_sensor_location = 1;

/* HRM flags byte — bit 4 set when RR-interval is included */
#define HRM_FLAG_RR_PRESENT BIT(4)

/* hrs_service is defined by BT_GATT_SERVICE_DEFINE in bluetooth.c */
extern const struct bt_gatt_service_static hrs_service;

/* ========== GATT Callbacks ========== */

void hr_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    hr_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("HRM notifications %s", hr_notify_enabled ? "enabled" : "disabled");
}

ssize_t read_body_sensor_location(struct bt_conn *conn, const struct bt_gatt_attr *attr,
    void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
        &body_sensor_location, sizeof(body_sensor_location));
}

/* ========== Channel Lifecycle ========== */

void bt_hr_on_disconnect(void) {
    hr_notify_enabled = false;
    last_rr_ms = 0;
}

/* ========== Internal Helpers ========== */

/**
 * @brief Build and send a Heart Rate Measurement notification.
 *
 * Packs flags + HR + optional RR-interval into the HRM characteristic value
 * and notifies the connected client.
 */
static int send_hrm_notification(void) {
    uint8_t pkt[4];
    uint8_t len;

    if (last_rr_ms > 0) {
        /* RR-interval in units of 1/1024 second */
        uint16_t rr_1024 = (uint32_t)last_rr_ms * 1024U / 1000U;
        pkt[0] = HRM_FLAG_RR_PRESENT;
        pkt[1] = last_hr_bpm;
        pkt[2] = (uint8_t)(rr_1024 & 0xFFU);
        pkt[3] = (uint8_t)(rr_1024 >> 8);
        len = 4;
    } else {
        pkt[0] = 0x00;
        pkt[1] = last_hr_bpm;
        len = 2;
    }

    /* hrs_service.attrs[1] = HRM characteristic declaration (matches Zephyr notify convention) */
    return notify_simple(&hrs_service.attrs[1], &hr_notify_enabled, pkt, len, "HRM");
}

/* ========== Public API ========== */

/**
 * @brief Store the latest RR-interval for inclusion in the next HRM notification.
 *
 * Call this before bluetooth_notify_heart_rate() so the current RR is bundled
 * in the same HRM packet. No BLE notification is sent by this function.
 */
int bluetooth_notify_hrv(uint16_t rr_ms) {
    last_rr_ms = rr_ms;
    return 0;
}

/**
 * @brief Send a Heart Rate Measurement notification to the connected client.
 *
 * Includes the last RR-interval stored via bluetooth_notify_hrv() if available.
 */
int bluetooth_notify_heart_rate(uint8_t bpm) {
    last_hr_bpm = bpm;
    int err = send_hrm_notification();
    if (err == 0) {
        LOG_DBG("HRM notified: %u bpm, rr=%u ms", bpm, last_rr_ms);
    }
    return err;
}
