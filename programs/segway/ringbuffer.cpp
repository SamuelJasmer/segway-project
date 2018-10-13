/*
 * Collin Janke
 * Float RingBuffer
 */

//#include "ArduinoSTL.h"
#include "ringbuffer.h"
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Converts the given virtual index (relative to startIndex)
 *             to the corresponding real index where data is stored in the
 *             underlying array.
 * @note   This function does no bounds checking, but indexes are calculated
 *             mod capacity, so the returned index will always be within the
 *             bounds of the underlying array.
 * @param  buf:   The buffer to use for conversion
 * @param  index: The virtual index to convert
 * @retval The real array index
 */
static uint8_t ringbuffer_get_real_index(RingBuffer* buf, uint8_t index)
{
    return (index + buf->headIndex) % buf->capacity;
}

/**
 * @brief  Creates a new RingBuffer with the given capacity
 * @note   RingBuffer should be free'd using ringbuffer_destroy()
 * @param  capacity: Capacity of RingBuffer. If zero, no array will be created.
 * @retval Pointer to new RingBuffer
 */
RingBuffer* ringbuffer_new(uint8_t capacity)
{
    RingBuffer* buf = (RingBuffer*)malloc(sizeof(RingBuffer));
	if (buf == NULL) return NULL;

    buf->data = (capacity == 0 ? 0 : (float*)malloc(capacity << 2));
	if (buf->data == NULL && capacity != 0)
	{
		free(buf);
		return NULL;
	}

    buf->headIndex = 0;
    buf->count = 0;
    buf->capacity = capacity;
    return buf;
}

/**
 * @brief  Destroys the given RingBuffer
 * @note   If a data array was manually set, data should be set to null before
 *             calling destroy, otherwise the array will be freed
 * @param  buf: RingBuffer to destroy
 * @retval None
 */
void ringbuffer_destroy(RingBuffer* buf)
{
	if (buf == NULL) return;
    if (buf->data != NULL)
        free(buf->data);

    free(buf);
}

/**
 * @brief  Expands the RingBuffer to its maximum capacity and replaces all
 *         values with the specified value
 * @param  buf: Target RingBuffer
 * @param  val: Value to put in all entries
 * @retval None
 */
void ringbuffer_fill(RingBuffer* buf, float val)
{
	if (buf == NULL || buf->data == NULL) return;

	for (uint8_t i = 0; i < buf->capacity; ++i)
	{
		buf->data[i] = val;
	}
	buf->count = buf->capacity;
}

/**
 * @brief  Clears all content from the given RingBuffer
 * @note   Data will not be zeroed.  Use ringbuffer_clear_secure()
 *             to zero all data.
 * @param  buf: Target RingBuffer
 * @retval None
 */
void ringbuffer_clear(RingBuffer* buf)
{
	if (buf == NULL || buf->data == NULL) return;
    buf->headIndex = 0;
    buf->count = 0;
}

/**
 * @brief  Clears and zeros all content from the given RingBuffer
 * @param  buf: Target RingBuffer
 * @retval None
 */
void ringbuffer_clear_secure(RingBuffer* buf)
{
	if (buf == NULL || buf->data == NULL) return;

    for (uint8_t i = 0; i < buf->capacity; ++i)
    {
        buf->data[i] = 0;
    }
    ringbuffer_clear(buf);
}

/**
 * @brief  Returns the float at the given index.
 * @param  buf:   Target RingBuffer
 * @param  index: Virtual index
 * @retval The float at the given index
 */
float ringbuffer_at(RingBuffer* buf, uint8_t index)
{
	if (buf == NULL || buf->data == NULL || index >= buf->count)
        return 0.0f;

    return buf->data[ringbuffer_get_real_index(buf, index)];
}

/**
 * @brief  Adds a float to the end of the RingBuffer.
 * @note   If the buffer is full, the head entry will be overwritten.
 * @param  buf: Target RingBuffer
 * @param  f:   Float to add
 * @retval None
 */
void ringbuffer_enqueue(RingBuffer* buf, float f)
{
	if (buf == NULL || buf->data == NULL) return;

    uint8_t nextIndex = ringbuffer_get_real_index(buf, buf->count);
    buf->data[nextIndex] = f;

    if (buf->count == buf->capacity)
        buf->headIndex = (nextIndex + 1) % buf->capacity;
    else
        ++(buf->count);
}

/**
 * @brief  Removes and returns the float at the head of the RingBuffer.
 * @param  buf: Target RingBuffer
 * @retval The float at the head of the RingBuffer
 */
float ringbuffer_dequeue(RingBuffer* buf)
{
	if (buf == NULL || buf->data == NULL || buf->count == 0)
        return 0.0f;

    float f = buf->data[buf->headIndex];
    buf->headIndex = ringbuffer_get_real_index(buf, 1);
    --(buf->count);
    return f;
}

/**
 * @brief  Removes and returns the float at the tail of the RingBuffer.
 * @param  buf: Target RingBuffer
 * @retval The float at the tail of the RingBuffer
 */
float ringbuffer_dequeue_last(RingBuffer* buf)
{
    if (buf == NULL || buf->data == NULL || buf->count == 0)
        return 0.0f;

    --(buf->count);
    return ringbuffer_at(buf, buf->count);
}

#ifdef __cplusplus
} // extern "C"
#endif
