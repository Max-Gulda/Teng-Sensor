/**
 * @file bt_channel.c
 * @brief Generic Bluetooth channel pipeline: batching, DLQ, and GATT notify.
 *
 * Operates entirely through bt_channel_t descriptors defined in each channel
 * file. References connection state and the GATT service from bluetooth.c.
 */

#include "bt_channel.h"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_DECLARE(bluetooth, LOG_LEVEL_INF);

/* Owned by bluetooth.c */
extern volatile bool connected;
extern const struct bt_gatt_service_static ecg_service; /* used by channel_send_internal */

/**
 * @brief Send a buffer of samples via GATT notification.
 *
 * @param ch       Channel descriptor
 * @param buffer   Pointer to array of packed samples
 * @param count    Number of samples in buffer
 * @param is_retry True if this is a retry from the DLQ
 * @return 0 on success, negative errno on failure
 */
int channel_send_internal(bt_channel_t *ch, void *buffer, uint8_t count, bool is_retry) {
    if (count == 0) {
        return 0;
    }

    if (!connected) {
        return -ENOTCONN;
    }

    size_t bytes = count * ch->batch_elem_size;
    int err = bt_gatt_notify(NULL, &ecg_service.attrs[ch->gatt_attr_idx], buffer, bytes);
    if (err < 0) {
        LOG_WRN("%s batch notify failed (size=%zu, err=%d, retry=%d)",
            ch->name, bytes, err, is_retry);
        return err;
    }

    ch->stats->samples_sent += count;
    if (is_retry) {
        ch->stats->batch_retries++;
        LOG_DBG("%s retry successful, sent %u samples", ch->name, count);
    }

    /* Update the current value cache used by GATT reads */
    memcpy(ch->current_value,
        (uint8_t *) buffer + (count - 1) * ch->batch_elem_size,
        ch->batch_elem_size);

    return 0;
}

/**
 * @brief Retry the oldest failed batch from the DLQ, discarding stale entries.
 *
 * @param ch                Channel descriptor
 * @param current_timestamp Current sample count (for age calculation)
 * @return 0 on success or empty DLQ, negative errno on send failure
 */
int channel_retry_failed(bt_channel_t *ch, uint32_t current_timestamp) {
    bt_failed_batch_t oldest;

    if (k_msgq_num_used_get(ch->dlq) == 0) {
        return 0;
    }

    int err = k_msgq_peek(ch->dlq, &oldest);
    if (err != 0) {
        LOG_ERR("Failed to peek %s DLQ: %d", ch->name, err);
        return err;
    }

    uint32_t age = current_timestamp - oldest.first_timestamp;
    if (age > ch->stale_age) {
        LOG_WRN("Discarding stale %s DLQ batch: %u samples, age=%u",
            ch->name, oldest.count, age);
        ch->stats->samples_discarded_stale += oldest.count;
        k_msgq_get(ch->dlq, &oldest, K_NO_WAIT);

        if (k_msgq_num_used_get(ch->dlq) > 0) {
            return channel_retry_failed(ch, current_timestamp);
        }
        return 0;
    }

    err = channel_send_internal(ch, oldest.data, oldest.count, true);
    if (err == 0) {
        k_msgq_get(ch->dlq, &oldest, K_NO_WAIT);
        LOG_DBG("%s DLQ retry success, %u batches remaining",
            ch->name, k_msgq_num_used_get(ch->dlq));
    }

    return err;
}

/**
 * @brief Flush the current batch buffer via GATT notification.
 * On failure, saves the batch to the DLQ for later retry.
 *
 * @param ch Channel descriptor
 * @return 0 on success, negative errno on failure
 */
int channel_send_batch(bt_channel_t *ch) {
    if (*ch->batch_count == 0) {
        return 0;
    }

    if (!connected) {
        *ch->batch_count = 0;
        return -ENOTCONN;
    }

    int err = channel_send_internal(ch, ch->batch_buf, *ch->batch_count, false);
    if (err < 0 && err != -ENOTCONN) {
        bt_failed_batch_t failed;
        failed.count = *ch->batch_count;
        /* sample_count is the first field in all packed BLE payload structs. */
        failed.first_timestamp = *(uint32_t *) ch->batch_buf;
        memcpy(failed.data, ch->batch_buf, *ch->batch_count * ch->batch_elem_size);

        int put_err = k_msgq_put(ch->dlq, &failed, K_NO_WAIT);
        if (put_err == 0) {
            LOG_WRN("%s batch failed, saved %u samples to DLQ (queue=%u/%u, timestamp=%u)",
                ch->name, *ch->batch_count,
                k_msgq_num_used_get(ch->dlq), ch->dlq_size,
                failed.first_timestamp);
        } else {
            LOG_ERR("CRITICAL: %s DLQ full (%u batches), dropping %u samples",
                ch->name, ch->dlq_size, *ch->batch_count);
            ch->stats->queue_overflows += *ch->batch_count;
            ch->stats->dlq_overflows++;
        }
    }

    *ch->batch_count = 0;
    *ch->last_batch_time = k_uptime_get();
    return err;
}

/**
 * @brief Enqueue one sample and update fill-level statistics.
 * Logs warnings when the queue is filling up or has recovered.
 *
 * @param ch     Channel descriptor
 * @param sample Pointer to the typed sample struct
 * @return 0 on success, -ENOMEM if queue is full
 */
int channel_queue_put(bt_channel_t *ch, const void *sample) {
    int err = k_msgq_put(ch->queue, sample, K_MSEC(ch->queue_timeout_ms));
    if (err != 0) {
        ch->stats->queue_overflows++;
        if (ch->stats->queue_overflows == 1) {
            LOG_ERR("CRITICAL: %s BT queue full! First sample dropped.", ch->name);
        } else if (ch->stats->queue_overflows % 10 == 0) {
            LOG_ERR("CRITICAL: %s dropped %u samples total!",
                ch->name, ch->stats->queue_overflows);
        }
        return -ENOMEM;
    }

    uint32_t used = k_msgq_num_used_get(ch->queue);
    if (used > ch->stats->max_queue_used) {
        ch->stats->max_queue_used = used;
    }

    if (used >= ch->queue_warn_level && !ch->stats->queue_warning_shown) {
        uint32_t pct = (used * 100) / ch->queue_size;
        LOG_WRN("%s BT queue filling up: %u/%u samples (%u%%).",
            ch->name, used, ch->queue_size, pct);
        ch->stats->queue_warning_shown = true;
    }

    if (used < (ch->queue_size / 2) && ch->stats->queue_warning_shown) {
        LOG_INF("%s BT queue recovered: %u/%u samples", ch->name, used, ch->queue_size);
        ch->stats->queue_warning_shown = false;
    }

    return 0;
}

/**
 * @brief Send a GATT notification for a small scalar value.
 * Used for HRM which doesn't need batching or a DLQ.
 */
int notify_simple(const struct bt_gatt_attr *attr, volatile bool *enabled,
    void *value, size_t size, const char *name) {
    ARG_UNUSED(enabled);

    if (!connected) {
        return -ENOTCONN;
    }

    int err = bt_gatt_notify(NULL, attr, value, size);
    if (err < 0) {
        LOG_WRN("%s notify failed (err=%d)", name, err);
    }

    return err;
}
