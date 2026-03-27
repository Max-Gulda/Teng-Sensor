#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>

typedef struct {
    int32_t *buf;   // storage
    uint32_t cap;   // capacity
    uint32_t head;  // next write index
    uint32_t count; // number of valid items (0..cap)
} CircularBuffer;

/* Create with given capacity. Returns 0 on success. */
CircularBuffer *cb_init(uint32_t capacity);

/* Initialize a caller-provided CircularBuffer struct with a caller-provided
   backing buffer.  No heap allocation is performed. */
void cb_init_static(CircularBuffer *cb, int32_t *buf, uint32_t capacity);

/* Free memory */
void cb_free(CircularBuffer *cb);

/* Push one value. Overwrites oldest when full. */
void cb_push(CircularBuffer *cb, int32_t v);

/* How many valid samples are in the buffer */
uint32_t cb_count(CircularBuffer *cb);

/* Copy all samples to out[] in chronological order (oldest -> newest).
   out must have space for at least cb->count items.
   Returns number of samples copied. */
uint32_t cb_copy_all_oldest_first(CircularBuffer *cb, int32_t *out);

/* Prints the content of the circular buffer*/
void cb_print_buffer(const CircularBuffer *cb);

uint32_t cb_cap(CircularBuffer *cb);

#endif // CIRCULAR_BUFFER_H