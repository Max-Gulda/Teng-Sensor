/**
 * @file bt_hr.h
 * @brief Heart Rate Service (0x180D) — internal header for src/bluetooth/.
 */

#ifndef BT_HR_H
#define BT_HR_H

#include <zephyr/bluetooth/gatt.h>
#include <stdint.h>

/* GATT callbacks — referenced by BT_GATT_SERVICE_DEFINE in bluetooth.c */
void hr_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
ssize_t read_body_sensor_location(struct bt_conn *conn, const struct bt_gatt_attr *attr,
    void *buf, uint16_t len, uint16_t offset);

/* Channel lifecycle */
void bt_hr_on_disconnect(void);

#endif /* BT_HR_H */
