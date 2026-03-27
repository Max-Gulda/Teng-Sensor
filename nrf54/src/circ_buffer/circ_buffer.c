#include "circ_buffer.h"
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <error_handling.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(circ_buffer, LOG_LEVEL_INF);

CircularBuffer *cb_init(uint32_t capacity) {

  CircularBuffer *cb = (CircularBuffer *) k_malloc(sizeof(CircularBuffer));
  if (!cb) {
    errno = ENOMEM;
    ERROR_CHECK("Failed to allocate cb");
    return NULL;
  }

  if (capacity == 0) {
    errno = EINVAL;
    ERROR_CHECK("Capacity cannot be set to 0");
    return NULL;
  }
  LOG_INF("capacity: %d\n", capacity);

  cb->buf = (int32_t *) k_malloc(capacity * sizeof(int32_t));
  if (!cb->buf) {
    errno = ENOMEM;
    ERROR_CHECK("Failed to allocate cb->buf");
    return NULL;
  }

  memset(cb->buf, 0, capacity * sizeof(int32_t));
  cb->cap = capacity;
  cb->head = 0;
  cb->count = 0;
  LOG_INF("cb->cap = %d\n", cb->cap);
  return cb;
}

void cb_init_static(CircularBuffer *cb, int32_t *buf, uint32_t capacity) {
  cb->buf = buf;
  cb->cap = capacity;
  cb->head = 0;
  cb->count = 0;
}

void cb_free(CircularBuffer *cb) {
  if (!cb) {
    errno = ENXIO;
    ERROR_CHECK("cb does not exist or NULL has been passed");
    return;
  }

  free(cb->buf);
  cb->buf = NULL;
  cb->cap = cb->head = cb->count = 0;
  free(cb);
}

inline void cb_push(CircularBuffer *cb, int32_t data) {
  if (!cb) {
    errno = ENXIO;
    ERROR_CHECK("cb does not exist or NULL has been passed");
    return;
  }

  if (!cb->buf) {
    errno = ENXIO;
    ERROR_CHECK("cb->buf does not exist");
    return;
  }
  cb->buf[cb->head] = data;
  cb->head = (cb->head + 1) % cb->cap;


  if (cb->count < cb->cap) {
    cb->count++;
  }
}

inline uint32_t cb_count(CircularBuffer *cb) {
  if (!cb) {
    errno = ENXIO;
    ERROR_CHECK("cb does not exist or NULL has been passed");
    return -1;
  }
  return cb->count;
}

inline uint32_t cb_cap(CircularBuffer *cb) {
  return cb->cap;
}

uint32_t cb_copy_all_oldest_first(CircularBuffer *cb, int32_t *out) {
  if (!cb || !out) {
    errno = ENXIO;
    ERROR_CHECK("cb or out does not exist or NULL has been passed");
    return 0;
  }

  if (!cb->buf) {
    errno = ENXIO;
    ERROR_CHECK("cb->buf does not exist");
    return 0;
  }

  uint32_t n = cb->count;
  if (n == 0) {
    return 0;
  }

  uint32_t start = (cb->head + cb->cap - n) % cb->cap;
  uint32_t first = (start + n <= cb->cap) ? n : (cb->cap - start);

  memcpy(out, cb->buf + start, first * sizeof(int32_t));
  if (first < n) {
    memcpy(out + first, cb->buf, (n - first) * sizeof(int32_t));
  }
  return n;
}

void inline cb_print_buffer(const CircularBuffer *cb) {
  if (cb->count == 0) {
    LOG_INF("CB empty\n");
    return;
  }

  uint32_t n = cb->count;
  uint32_t start = (cb->head + cb->cap - n) % cb->cap;
  LOG_INF("start %d count %d \n", start, n);

  for (uint32_t i = 0; i < n; i++) {
    uint32_t idx = (start + i) % cb->cap;
    LOG_INF("%d, \n", idx);
  }
}
