#include "protocol.h"

#if (RX_RING_SIZE & (RX_RING_SIZE - 1)) != 0
#error "RX_RING_SIZE must be a power of two"
#endif
#if (TX_RING_SIZE & (TX_RING_SIZE - 1)) != 0
#error "TX_RING_SIZE must be a power of two"
#endif

static uint8_t s_rx_buf[RX_RING_SIZE];
static uint8_t s_tx_buf[TX_RING_SIZE];

ring_t g_rx_ring;
ring_t g_tx_ring;

void ring_init(ring_t *r, uint8_t *buf, uint16_t size) {
    r->buf = buf;
    r->size = size;
    r->head = 0;
    r->tail = 0;
}

bool ring_put(ring_t *r, uint8_t b) {
    uint16_t next = (uint16_t)((r->head + 1u) & (r->size - 1u));
    if (next == r->tail) return false;
    r->buf[r->head] = b;
    __asm volatile ("" ::: "memory");
    r->head = next;
    return true;
}

bool ring_get(ring_t *r, uint8_t *b) {
    if (r->head == r->tail) return false;
    *b = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1u) & (r->size - 1u));
    return true;
}

uint16_t ring_count(const ring_t *r) {
    int32_t d = (int32_t)r->head - (int32_t)r->tail;
    if (d < 0) d += r->size;
    return (uint16_t)d;
}

bool ring_empty(const ring_t *r) {
    return r->head == r->tail;
}

uint8_t proto_xor(uint8_t cmd, const uint8_t data[5]) {
    uint8_t x = cmd;
    for (int i = 0; i < 5; ++i) x ^= data[i];
    return x;
}

void frame_init(void) {
    ring_init(&g_rx_ring, s_rx_buf, RX_RING_SIZE);
    ring_init(&g_tx_ring, s_tx_buf, TX_RING_SIZE);
}

void proto_feed(uint8_t byte) {
    ring_put(&g_rx_ring, byte);
}

void proto_send_frame(uint8_t cmd,
                      uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
    uint8_t data[5] = {d0, d1, d2, d3, d4};
    uint8_t x = proto_xor(cmd, data);
    ring_put(&g_tx_ring, PROTO_SYNC);
    ring_put(&g_tx_ring, cmd);
    ring_put(&g_tx_ring, d0);
    ring_put(&g_tx_ring, d1);
    ring_put(&g_tx_ring, d2);
    ring_put(&g_tx_ring, d3);
    ring_put(&g_tx_ring, d4);
    ring_put(&g_tx_ring, x);
    hw_uart_kick_tx();
}

void frame_send_short(uint8_t cmd) {
    proto_send_frame(cmd, 0, 0, 0, 0, 0);
}

typedef enum {
    ST_WAIT_SYNC = 0,
    ST_CMD,
    ST_DATA,
    ST_XOR
} parse_state_t;

void proto_task(void) {
    static parse_state_t st = ST_WAIT_SYNC;
    static uint8_t cmd = 0;
    static uint8_t data[5];
    static uint8_t di = 0;

    uint8_t b;
    while (ring_get(&g_rx_ring, &b)) {
        switch (st) {
            case ST_WAIT_SYNC:
                if (b == PROTO_SYNC) st = ST_CMD;
                break;
            case ST_CMD:
                cmd = b;
                di = 0;
                st = ST_DATA;
                break;
            case ST_DATA:
                data[di++] = b;
                if (di >= 5) st = ST_XOR;
                break;
            case ST_XOR: {
                uint8_t x = proto_xor(cmd, data);
                if (x == b) frame_dispatch(cmd, data);
                st = ST_WAIT_SYNC;
                break;
            }
        }
    }
}
