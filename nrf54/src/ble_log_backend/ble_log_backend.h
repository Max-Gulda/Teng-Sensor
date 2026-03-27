/**
 * @file ble_log_backend.h
 * @brief Zephyr log backend that outputs over BLE Nordic UART Service (NUS)
 */

#ifndef BLE_LOG_BACKEND_H
#define BLE_LOG_BACKEND_H

/**
 * @brief Enable the BLE log backend.
 *
 * Call this after bluetooth_init() and bt_nus_init() are complete.
 * Log messages are silently dropped when no BLE client is connected.
 */
void ble_log_backend_enable(void);

/**
 * @brief Disable the BLE log backend.
 */
void ble_log_backend_disable(void);

#endif /* BLE_LOG_BACKEND_H */
