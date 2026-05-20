/**
 * @file bluetooth.h
 * @brief Bluetooth Low Energy communication for ADC data.
 */

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

 /* Thread configuration */
#define BT_THREAD_PRIORITY   7      /* Lower priority than sampling (higher number) */
#define BT_THREAD_STACK_SIZE 2048

/* Per-channel Bluetooth statistics */
typedef struct {
    uint32_t samples_sent;            /* Total samples transmitted */
    uint32_t queue_overflows;         /* Samples dropped due to full queue */
    uint32_t max_queue_used;          /* Maximum queue usage observed */
    uint32_t current_queue_used;      /* Current queue fill level */
    uint32_t batch_retries;           /* Successful batch retries from DLQ */
    uint32_t samples_discarded_stale; /* Samples discarded as too old */
    uint32_t dlq_overflows;           /* Batches dropped due to full DLQ */
    uint8_t dlq_count;                /* Current number of batches in DLQ */
} bt_channel_stats_t;

/* Bluetooth statistics */
typedef struct {
    bt_channel_stats_t adc;           /* Primary ADC channel statistics */
    uint32_t bt_disconnects;          /* Number of disconnections */
    bool connected;                   /* Current connection status */
} bt_stats_t;

/**
 * @brief Initialize the Bluetooth system
 * @return 0 on success, negative error code on failure
 */
int bluetooth_init(void);

/**
 * @brief Start Bluetooth advertising
 * @return 0 on success, negative error code on failure
 */
int bluetooth_start(void);

/**
 * @brief Stop Bluetooth advertising
 * @return 0 on success, negative error code on failure
 */
int bluetooth_stop(void);

/**
 * @brief Queue one ADS131M04 sample frame for Bluetooth transmission.
 *
 * The main custom data characteristic carries CH0-CH3 from the ADS131M04.
 *
 * @param ch0 ADC channel 0 sample
 * @param ch1 ADC channel 1 sample
 * @param ch2 ADC channel 2 sample
 * @param ch3 ADC channel 3 sample
 * @param timestamp Sample number
 * @return 0 on success, -ENOMEM if queue is full
 */
int bluetooth_queue_sample(int32_t ch0, int32_t ch1, int32_t ch2, int32_t ch3,
    uint32_t timestamp);

/**
 * @brief Get current Bluetooth statistics
 * @param stats Pointer to structure to store statistics
 */
void bluetooth_get_stats(bt_stats_t *stats);

/**
 * @brief Print Bluetooth statistics to log
 * @param stats Pointer to statistics structure
 */
void bluetooth_print_stats(bt_stats_t *stats);

/**
 * @brief Reset Bluetooth statistics counters
 */
void bluetooth_reset_stats(void);

/**
 * @brief Check if a Bluetooth client is connected
 * @return true if connected, false otherwise
 */
bool bluetooth_is_connected(void);

#endif /* BLUETOOTH_H */
