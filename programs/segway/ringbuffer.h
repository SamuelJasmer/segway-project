/*
 * Collin Janke
 * CE 2812 021
 * Float RingBuffer
 */

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
    float* data;
    uint8_t headIndex;
    uint8_t count;
    uint8_t capacity;
} RingBuffer;

RingBuffer* ringbuffer_new(uint8_t capacity);
void ringbuffer_destroy(RingBuffer* buf);
void ringbuffer_fill(RingBuffer* buf, float val);
void ringbuffer_clear(RingBuffer* buf);
void ringbuffer_clear_secure(RingBuffer* buf);
float ringbuffer_at(RingBuffer* buf, uint8_t index);
void ringbuffer_enqueue(RingBuffer* buf, float c);
float ringbuffer_dequeue(RingBuffer* buf);
float ringbuffer_dequeue_last(RingBuffer* buf);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
