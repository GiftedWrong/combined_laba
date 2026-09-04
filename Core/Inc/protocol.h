#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "hal_shim.h"

#define PROTO_SYNC          0x7Cu
#define PROTO_FRAME_LEN     8u

#define CMD_PING            0x00u
#define CMD_SET_FREQ        0x01u
#define CMD_SET_DUTY        0x02u
#define CMD_SET_STATE       0x03u
#define CMD_SWEEP_SET       0x04u
#define CMD_SWEEP_CTRL      0x05u
#define CMD_SET_INVERT      0x06u
#define CMD_CONFIG_PUSH     0x09u
#define CMD_GET_FREQ        0x81u
#define CMD_GET_DUTY        0x82u
#define CMD_GET_STATE       0x83u
#define CMD_SNAP_FREQ_LO    0x84u
#define CMD_SNAP_FREQ_HI    0x85u
#define CMD_SNAP_DC_LO      0x86u
#define CMD_SNAP_DC_HI      0x87u
#define CMD_DMA_ON          0xACu
#define CMD_DMA_OFF         0xABu
#define CMD_SET_BAUD        0xADu
#define CMD_BTN_SET         0x7Bu
#define CMD_DMA_MARK        0x0Cu

#define PWM_CHANNELS        8u
#define BTN_SEQ_LEN         5u

#define SWEEP_MODE_FREQ     0u
#define SWEEP_MODE_DUTY     1u

typedef struct {
    uint32_t freq_hz;
    uint8_t  duty_pct;
    bool     enabled;
} pwm_channel_t;

typedef struct {
    uint8_t channel;
    uint8_t enable;
    uint8_t order;
    bool    used;
} btn_action_t;

typedef struct {
    bool     active;
    uint8_t  mode;
    uint16_t v_start;
    uint16_t v_stop;
    uint16_t step;
    uint16_t period_ms;
    uint16_t current;
    uint32_t last_tick;
    int16_t  dir;
} sweep_t;

extern pwm_channel_t g_pwm[PWM_CHANNELS];
extern btn_action_t  g_btn_seq[BTN_SEQ_LEN];
extern sweep_t       g_sweep[PWM_CHANNELS];

#ifndef RX_RING_SIZE
#define RX_RING_SIZE 256u
#endif
#ifndef TX_RING_SIZE
#define TX_RING_SIZE 256u
#endif

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    uint16_t size;
    uint8_t *buf;
} ring_t;

void ring_init(ring_t *r, uint8_t *buf, uint16_t size);
bool ring_put(ring_t *r, uint8_t b);
bool ring_get(ring_t *r, uint8_t *b);
uint16_t ring_count(const ring_t *r);
bool ring_empty(const ring_t *r);

extern ring_t g_rx_ring;
extern ring_t g_tx_ring;

void proto_init(void);
void frame_init(void);
void proto_feed(uint8_t byte);
void proto_task(void);
void proto_send_frame(uint8_t cmd,
                      uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4);
void frame_send_short(uint8_t cmd);
uint8_t proto_xor(uint8_t cmd, const uint8_t data[5]);
void frame_dispatch(uint8_t cmd, const uint8_t data[5]);

void proto_on_button(void);
void proto_sweep_tick(void);
void config_apply_buf(const uint8_t *buf, uint8_t n_records);

bool proto_dma_active(void);
void proto_on_dma_complete(void);
void proto_on_dma_send_failed(void);
void proto_on_config_rx_complete(void);

#endif
