/**
 * @file ble_log_backend.c
 * @brief Zephyr log backend that sends log output over BLE Nordic UART Service (NUS).
 *
 * Log messages are formatted as plain text and sent as NUS notifications.
 * Messages are silently dropped when no BLE client is connected or the BLE
 * stack is congested. This backend does not block the calling thread.
 *
 * Usage:
 *   1. Call ble_log_backend_enable() after bluetooth_init() in main.
 *   2. Connect with nRF Connect app (or any NUS-compatible terminal) and
 *      subscribe to the NUS TX characteristic (6E400003-...).
 */

#include "ble_log_backend.h"
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_output.h>
#include <bluetooth/services/nus.h>

/* Output buffer size. BLE MTU is 247 bytes; ATT overhead is 3 bytes, so the
 * max payload is 244 bytes. Keep the buffer at 244 to avoid splitting a single
 * formatted log line across multiple BLE packets. */
#define BLE_LOG_BUF_SIZE 244

static uint8_t ble_log_buf[BLE_LOG_BUF_SIZE];

static int ble_char_out(uint8_t *data, size_t length, void *ctx)
{
    ARG_UNUSED(ctx);
    /* bt_nus_send returns an error when not connected or congested;
     * we silently discard in that case to avoid blocking. */
    (void)bt_nus_send(NULL, data, length);
    return (int)length;
}

LOG_OUTPUT_DEFINE(ble_log_output, ble_char_out, ble_log_buf, sizeof(ble_log_buf));

static void ble_backend_process(const struct log_backend *const backend,
                                union log_msg_generic *msg)
{
    uint32_t flags = LOG_OUTPUT_FLAG_LEVEL | LOG_OUTPUT_FLAG_TIMESTAMP;

    log_output_msg_process(&ble_log_output, &msg->log, flags);
}

static void ble_backend_panic(struct log_backend const *const backend)
{
    log_output_flush(&ble_log_output);
}

static void ble_backend_dropped(const struct log_backend *const backend, uint32_t cnt)
{
    ARG_UNUSED(backend);
    ARG_UNUSED(cnt);
}

static const struct log_backend_api ble_log_backend_api = {
    .process = ble_backend_process,
    .dropped = ble_backend_dropped,
    .panic   = ble_backend_panic,
};

/* Start disabled; call ble_log_backend_enable() to activate. */
LOG_BACKEND_DEFINE(ble_log_backend, ble_log_backend_api, false);

void ble_log_backend_enable(void)
{
    log_backend_enable(&ble_log_backend, NULL, CONFIG_LOG_DEFAULT_LEVEL);
}

void ble_log_backend_disable(void)
{
    log_backend_disable(&ble_log_backend);
}
