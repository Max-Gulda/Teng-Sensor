/**
 * @file bt_channel.h
 * @brief Generic Bluetooth channel pipeline — internal to src/bluetooth/.
 *
 * Defines the shared types and pipeline function declarations used by all
 * data channels (ECG, IMU). Not part of the public API.
 */

#ifndef BT_CHANNEL_H
#define BT_CHANNEL_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

 /* Fixed payload size for DLQ entries.
 *   ECG:    20 samples × 8 bytes  = 160 bytes
  *   RAW ECG: 20 samples × 8 bytes = 160 bytes (optional)
  *   IMU: 15 samples × 16 bytes = 240 bytes
  * A _Static_assert in bluetooth.c validates this at compile time.
  */
#define BT_MAX_BATCH_PAYLOAD 240

  /* DLQ entry: fixed-size payload buffer sized for the largest channel batch */
typedef struct {
    uint8_t data[BT_MAX_BATCH_PAYLOAD];
    uint8_t count;
    uint32_t first_timestamp;
} bt_failed_batch_t;

/* Per-channel internal statistics */
typedef struct {
    volatile uint32_t samples_sent;
    volatile uint32_t queue_overflows;
    volatile uint32_t max_queue_used;
    volatile bool queue_warning_shown;
    volatile uint32_t batch_retries;
    volatile uint32_t samples_discarded_stale;
    volatile uint32_t dlq_overflows;
} bt_chan_stats_t;

/**
 * @brief Generic channel descriptor.
 *
 * All runtime state for one data channel (ecg/raw-ecg/imu).
 * The generic send/retry/batch/queue functions operate entirely through this struct.
 */
typedef struct {
    const char *name;

    /* GATT notification */
    uint8_t gatt_attr_idx;         /* Index in ecg_service.attrs[] */
    volatile bool *notify_enabled; /* Flag set by CCC callback */
    void *current_value;           /* Last-sent sample cache (for GATT reads) */

    /* Input queue */
    struct k_msgq *queue;
    uint32_t queue_size;
    uint32_t queue_warn_level;
    uint32_t queue_timeout_ms;

    /* Batching */
    void *batch_buf;               /* Static buffer for pending batch */
    uint8_t *batch_count;          /* Current number of samples in batch */
    int64_t *last_batch_time;
    uint8_t max_batch_size;
    uint32_t batch_timeout_ms;
    size_t batch_elem_size;        /* size of one packed sample element */

    /* Dead letter queue */
    struct k_msgq *dlq;
    uint32_t dlq_size;
    uint32_t stale_age;            /* DLQ entries older than this sample count are discarded */

    /* Statistics */
    bt_chan_stats_t *stats;
} bt_channel_t;

/* Pipeline functions (implemented in bt_channel.c) */
int channel_queue_put(bt_channel_t *ch, const void *sample);
int channel_send_internal(bt_channel_t *ch, void *buffer, uint8_t count, bool is_retry);
int channel_send_batch(bt_channel_t *ch);
int channel_retry_failed(bt_channel_t *ch, uint32_t current_timestamp);
int notify_simple(const struct bt_gatt_attr *attr, volatile bool *enabled,
    void *value, size_t size, const char *name);

#endif /* BT_CHANNEL_H */
