/**
 * @file bluetooth.h
 * @brief Bluetooth Low Energy communication for ADC/raw-ECG/IMU data.
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
    bt_channel_stats_t imu;           /* IMU channel statistics */
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
 * The optional raw ECG characteristic and the HR pipeline use CH0 as the
 * ECG/heart source on this PCB.
 *
 * @param ch0 ADC channel 0 sample (ECG/heart on this PCB)
 * @param ch1 ADC channel 1 sample
 * @param ch2 ADC channel 2 sample
 * @param ch3 ADC channel 3 sample
 * @param timestamp Sample number
 * @return 0 on success, -ENOMEM if queue is full
 */
int bluetooth_queue_sample(int32_t ch0, int32_t ch1, int32_t ch2, int32_t ch3,
    uint32_t timestamp);

/**
 * @brief Queue an IMU sample for Bluetooth transmission
 * @param ax Accelerometer X
 * @param ay Accelerometer Y
 * @param az Accelerometer Z
 * @param gx Gyroscope X
 * @param gy Gyroscope Y
 * @param gz Gyroscope Z
 * @param timestamp Sample number
 * @return 0 on success, -ENOMEM if queue is full
 */
int bluetooth_queue_imu_sample(int16_t ax, int16_t ay, int16_t az,
    int16_t gx, int16_t gy, int16_t gz, uint32_t timestamp);

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

/**
 * @brief Send a Heart Rate Measurement notification to the connected client.
 *
 * Transmits a BT Heart Rate Service (0x180D) HRM notification containing the
 * BPM value and the last RR-interval stored via bluetooth_notify_hrv().
 * Call bluetooth_notify_hrv() first if an RR-interval is available.
 *
 * @param bpm Heart rate in beats per minute
 * @return 0 on success, -ENOTCONN if not connected/subscribed
 */
int bluetooth_notify_heart_rate(uint8_t bpm);

/**
 * @brief Store the latest RR-interval for bundling into the next HRM notification.
 *
 * This does not send a BLE notification on its own. The stored value is
 * included in the next bluetooth_notify_heart_rate() call as an RR-Interval
 * field per the BT Heart Rate Measurement characteristic format.
 *
 * @param rr_ms RR interval in milliseconds
 * @return Always 0
 */
int bluetooth_notify_hrv(uint16_t rr_ms);

#endif /* BLUETOOTH_H */
