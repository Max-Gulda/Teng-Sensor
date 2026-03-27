#include "heart_rate_tuning_params.h"

/* Thread configuration */
#ifndef ECG_THREAD_PRIORITY
#define ECG_THREAD_PRIORITY   6
#endif
#ifndef ECG_THREAD_STACK_SIZE
#define ECG_THREAD_STACK_SIZE 2048
#endif
#ifndef ECG_BP_ORDER
#define ECG_BP_ORDER 4
#endif
#ifndef ECG_INTEGRATION_TIME_MS
#define ECG_INTEGRATION_TIME_MS 150
#endif
#ifndef MSGQ_SIZE
#define MSGQ_SIZE 16
#endif
#ifndef ECG_QUEUE_TIMEOUT_MS
#define ECG_QUEUE_TIMEOUT_MS 100
#endif


int ecg_queue_sample(int32_t heart);

int ecg_init(void);

int ecg_start(void);
