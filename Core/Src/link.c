#include "protocol.h"
#include "link.h"

typedef enum {
    DMA_IDLE = 0,
    DMA_TX_TRANSFER,
    DMA_RX_TRANSFER,
} dma_state_t;

static volatile dma_state_t s_dma_state = DMA_IDLE;
static volatile uint16_t    s_dma_size  = 0;

static volatile uint8_t s_dma_buf[512];
static volatile uint8_t s_cfg_buf[CFG_BUF_MAX];
static volatile uint8_t s_cfg_records = 0;

void link_init(void) {
    s_dma_state = DMA_IDLE;
    s_dma_size  = 0;
    s_cfg_records = 0;
}

bool proto_dma_active(void) {
    return s_dma_state != DMA_IDLE;
}

static void send_marker(uint8_t type, uint16_t size) {
    proto_send_frame(CMD_DMA_MARK, type, 0, 0,
                     (uint8_t)(size & 0xFFu),
                     (uint8_t)((size >> 8) & 0xFFu));
}

static void dma_window_open_tx(uint16_t size) {
    s_dma_size = size;
    send_marker(1, size);
    hw_uart_tx_flush();
    s_dma_state = DMA_TX_TRANSFER;
}

static void dma_window_close_tx(void) {
    s_dma_state = DMA_IDLE;
    send_marker(0, s_dma_size);
}

static void dma_window_open_rx(uint16_t size) {
    s_dma_size = size;
    send_marker(1, size);
    hw_uart_tx_flush();
    s_dma_state = DMA_RX_TRANSFER;
}

static void dma_window_close_rx(void) {
    s_dma_state = DMA_IDLE;
    send_marker(0, s_dma_size);
}

/* Вариант 1: [0..127], [128..64], [65..255], циклически.
 * Период = 128 + 65 + 191 = 384 байт.  Значение 64 не дублируется. */
static uint8_t variant1_byte(uint16_t i) {
    uint16_t p = (uint16_t)(i % 384u);
    if (p < 128u)  return (uint8_t)p;            /* 0..127    */
    if (p <= 192u) return (uint8_t)(256u - p);    /* 128..64   */
    return (uint8_t)(p - 128u);                   /* 65..255   */
}

void handle_dma_on(const uint8_t data[5]) {
    uint16_t size = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
    if (size == 0) size = 16;
    if (size > sizeof(s_dma_buf)) size = sizeof(s_dma_buf);
    for (uint16_t i = 0; i < size; ++i) s_dma_buf[i] = variant1_byte(i);

    dma_window_open_tx(size);
    if (!hw_dma_send((const uint8_t *)s_dma_buf, size)) {
        dma_window_close_tx();
    }
}

void handle_dma_off(const uint8_t data[5]) {
    (void)data;
    if (s_dma_state == DMA_TX_TRANSFER) {
        hw_dma_abort();
        dma_window_close_tx();
    } else if (s_dma_state == DMA_RX_TRANSFER) {
        hw_uart_disarm_rx_dma();
        s_cfg_records = 0;
        dma_window_close_rx();
    }
}

void handle_set_baud(const uint8_t data[5]) {
    uint32_t baud = (uint32_t)data[0]
                  | ((uint32_t)data[1] << 8)
                  | ((uint32_t)data[2] << 16);
    if (baud < 300) return;
    proto_send_frame(CMD_SET_BAUD, data[0], data[1], data[2], data[3], data[4]);
    hw_uart_tx_flush();
    hw_uart_apply_baud(baud);
}

void handle_config_push(const uint8_t data[5]) {
    uint8_t  n    = data[0];
    uint16_t size = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
    if (n == 0 || n > CFG_MAX_RECORDS) return;
    if (size != (uint16_t)(n * CFG_REC_SIZE)) return;

    s_cfg_records = n;
    dma_window_open_rx(size);
    if (!hw_uart_arm_rx_dma((uint8_t *)s_cfg_buf, size)) {
        s_cfg_records = 0;
        dma_window_close_rx();
    }
}

void proto_on_dma_complete(void) {
    if (s_dma_state != DMA_TX_TRANSFER) return;
    dma_window_close_tx();
}

void proto_on_dma_send_failed(void) {
    if (s_dma_state != DMA_TX_TRANSFER) return;
    dma_window_close_tx();
}

void proto_on_config_rx_complete(void) {
    if (s_dma_state != DMA_RX_TRANSFER) return;
    config_apply_buf((const uint8_t *)s_cfg_buf, s_cfg_records);
    s_cfg_records = 0;
    hw_uart_disarm_rx_dma();
    dma_window_close_rx();
}
