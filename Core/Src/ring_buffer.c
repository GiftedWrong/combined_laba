#include "ring_buffer.h"
#include <string.h>

void buffer_init(RingBuffer_t *buf, uint8_t *storage, uint16_t size) {
    buf->buf = storage;
    buf->capacity = size;
    buf->head = 0;
    buf->tail = 0;
}

void buffer_addToEnd(RingBuffer_t *buf, uint8_t data) {
    uint16_t next = (buf->head + 1u) % buf->capacity;
    if (next == buf->tail) return;  /* overflow — drop */
    buf->buf[buf->head] = data;
    __asm volatile ("" ::: "memory");
    buf->head = next;
}

bool buffer_getFromFront(RingBuffer_t *buf, uint8_t *data) {
    if (buf->head == buf->tail) return false;
    *data = buf->buf[buf->tail];
    buf->tail = (buf->tail + 1u) % buf->capacity;
    return true;
}

uint16_t buffer_count(const RingBuffer_t *buf) {
    int32_t d = (int32_t)buf->head - (int32_t)buf->tail;
    if (d < 0) d += buf->capacity;
    return (uint16_t)d;
}

bool buffer_isEmpty(const RingBuffer_t *buf) {
    return buf->head == buf->tail;
}
