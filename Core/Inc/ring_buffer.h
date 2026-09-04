#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    uint16_t capacity;
    uint8_t *buf;
} RingBuffer_t;

void    buffer_init(RingBuffer_t *buf, uint8_t *storage, uint16_t size);
void    buffer_addToEnd(RingBuffer_t *buf, uint8_t data);
bool    buffer_getFromFront(RingBuffer_t *buf, uint8_t *data);
uint16_t buffer_count(const RingBuffer_t *buf);
bool    buffer_isEmpty(const RingBuffer_t *buf);

#endif /* RING_BUFFER_H */
