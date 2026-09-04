/**
  * @file    pwm_app.c
  * @brief   Протокол верхнего уровня: обработка команд, sweep, кнопки,
  *          конфигурация (CONFIG_PUSH).
  *
  * Лабораторная работа 4 — реализация:
  *   - blink 10 Гц (mode=2, CONFIG_PUSH)
  *   - sweep duty 5%/100 мс (mode=3, CONFIG_PUSH)
  */
#include "protocol.h"
#include "link.h"
#include <string.h>

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

pwm_channel_t g_pwm[PWM_CHANNELS];
btn_action_t  g_btn_seq[BTN_SEQ_LEN];
sweep_t       g_sweep[PWM_CHANNELS];

static uint8_t s_btn_pos = 0;

void proto_init(void) {
    frame_init();
    link_init();
    memset(g_pwm, 0, sizeof(g_pwm));
    memset(g_btn_seq, 0, sizeof(g_btn_seq));
    memset(g_sweep, 0, sizeof(g_sweep));
    for (int i = 0; i < PWM_CHANNELS; ++i) {
        g_pwm[i].freq_hz = 1000;
        g_pwm[i].duty_pct = 50;
        g_pwm[i].enabled = false;
    }
    s_btn_pos = 0;
}

/* ---------- PWM set/get ---------- */

static void handle_set_freq(const uint8_t data[5]) {
    uint32_t f = (uint32_t)data[0]
               | ((uint32_t)data[1] << 8)
               | ((uint32_t)data[2] << 16)
               | ((uint32_t)data[3] << 24);
    uint8_t ch = data[4];
    if (ch < 1 || ch > PWM_CHANNELS) return;
    g_pwm[ch - 1].freq_hz = f;
    hw_pwm_apply_freq(ch);
}

static void handle_set_duty(const uint8_t data[5]) {
    uint8_t d = data[0];
    uint8_t ch = data[4];
    if (ch < 1 || ch > PWM_CHANNELS) return;
    if (d > 100) d = 100;
    g_pwm[ch - 1].duty_pct = d;
    hw_pwm_apply_duty(ch);
}

static void apply_state_field(uint8_t v) {
    uint8_t ch     = v & 0x0F;
    uint8_t state  = (v >> 4) & 0x01;
    uint8_t modify = (v >> 5) & 0x01;
    if (ch < 1 || ch > PWM_CHANNELS) return;
    if (!modify) return;
    g_pwm[ch - 1].enabled = state ? true : false;
    hw_pwm_apply_state(ch);
}

static void handle_set_state(const uint8_t data[5]) {
    for (int i = 0; i < 5; ++i) apply_state_field(data[i]);
}

static void handle_set_invert(const uint8_t data[5]) {
    uint8_t ch     = data[0];
    bool    invert = (data[1] != 0);
    if (ch >= 1 && ch <= PWM_CHANNELS) hw_pwm_apply_invert(ch, invert);
    proto_send_frame(CMD_SET_INVERT, data[0], data[1], 0, 0, 0);
}

static void handle_get_freq(const uint8_t data[5]) {
    uint8_t ch = data[4];
    if (ch < 1 || ch > PWM_CHANNELS) {
        proto_send_frame(CMD_GET_FREQ & 0x7F, 0, 0, 0, 0, ch);
        return;
    }
    uint32_t f = g_pwm[ch - 1].freq_hz;
    proto_send_frame(CMD_GET_FREQ & 0x7F,
                     (uint8_t)(f & 0xFF),
                     (uint8_t)((f >> 8) & 0xFF),
                     (uint8_t)((f >> 16) & 0xFF),
                     (uint8_t)((f >> 24) & 0xFF),
                     ch);
}

static void handle_get_duty(const uint8_t data[5]) {
    uint8_t ch = data[4];
    if (ch < 1 || ch > PWM_CHANNELS) {
        proto_send_frame(CMD_GET_DUTY & 0x7F, 0, 0, 0, 0, ch);
        return;
    }
    proto_send_frame(CMD_GET_DUTY & 0x7F, g_pwm[ch - 1].duty_pct, 0, 0, 0, ch);
}

static uint8_t pack_state_field(uint8_t ch) {
    uint8_t v = ch & 0x0F;
    if (g_pwm[ch - 1].enabled) v |= (1u << 4);
    v |= (1u << 5);
    return v;
}

static void handle_get_state(const uint8_t data[5]) {
    uint8_t ch_nibbles[6];
    int n = 0;
    for (int i = 0; i < 3 && n < 5; ++i) {
        uint8_t lo = data[i] & 0x0F;
        uint8_t hi = (data[i] >> 4) & 0x0F;
        if (lo >= 1 && lo <= PWM_CHANNELS) ch_nibbles[n++] = lo;
        if (n < 5 && hi >= 1 && hi <= PWM_CHANNELS) ch_nibbles[n++] = hi;
    }
    uint8_t out[5] = {0};
    for (int i = 0; i < n && i < 5; ++i) out[i] = pack_state_field(ch_nibbles[i]);
    proto_send_frame(CMD_GET_STATE & 0x7F, out[0], out[1], out[2], out[3], out[4]);
}

/* ---------- snap ---------- */

static void send_freq_for(uint8_t ch, uint8_t cmd) {
    if (ch < 1 || ch > PWM_CHANNELS) return;
    uint32_t f = g_pwm[ch - 1].freq_hz;
    proto_send_frame(cmd,
                     (uint8_t)(f & 0xFF),
                     (uint8_t)((f >> 8) & 0xFF),
                     (uint8_t)((f >> 16) & 0xFF),
                     (uint8_t)((f >> 24) & 0xFF),
                     ch);
}

static void snap_freq_group(uint8_t cmd, uint8_t ch_from) {
    for (uint8_t ch = ch_from; ch < ch_from + 4 && ch <= PWM_CHANNELS; ++ch)
        send_freq_for(ch, cmd);
}

static void snap_dc_group(uint8_t cmd, uint8_t ch_from) {
    uint8_t out[5] = {0};
    uint8_t mask = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        uint8_t ch = ch_from + i;
        if (ch < 1 || ch > PWM_CHANNELS) continue;
        out[i] = g_pwm[ch - 1].duty_pct;
        if (g_pwm[ch - 1].enabled) mask |= (uint8_t)(1u << i);
    }
    out[4] = mask;
    proto_send_frame(cmd, out[0], out[1], out[2], out[3], out[4]);
}

static void handle_snap_freq_lo(const uint8_t data[5]) { (void)data; snap_freq_group(CMD_SNAP_FREQ_LO, 1); }
static void handle_snap_freq_hi(const uint8_t data[5]) { (void)data; snap_freq_group(CMD_SNAP_FREQ_HI, 5); }
static void handle_snap_dc_lo  (const uint8_t data[5]) { (void)data; snap_dc_group  (CMD_SNAP_DC_LO,   1); }
static void handle_snap_dc_hi  (const uint8_t data[5]) { (void)data; snap_dc_group  (CMD_SNAP_DC_HI,   5); }

/* ---------- ping ---------- */

static void handle_ping(const uint8_t data[5]) {
    proto_send_frame(CMD_PING, data[0], data[1], data[2], data[3], data[4]);
}

/* ---------- sweep ---------- */

static void handle_sweep_set(const uint8_t data[5]) {
    uint8_t ch   = data[0] & 0x0F;
    uint8_t mode = (data[0] >> 6) & 0x03;
    if (ch < 1 || ch > PWM_CHANNELS) {
        proto_send_frame(CMD_SWEEP_SET, data[0], data[1], data[2], data[3], data[4]);
        return;
    }
    sweep_t *s = &g_sweep[ch - 1];
    s->mode    = mode;
    s->v_start = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
    s->v_stop  = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
    s->current = s->v_start;
    s->dir     = 0;
    proto_send_frame(CMD_SWEEP_SET, data[0], data[1], data[2], data[3], data[4]);
}

static void handle_sweep_ctrl(const uint8_t data[5]) {
    uint8_t ch     = data[0] & 0x0F;
    uint8_t action = (data[0] >> 4) & 0x0F;
    if (ch < 1 || ch > PWM_CHANNELS) {
        proto_send_frame(CMD_SWEEP_CTRL, data[0], data[1], data[2], data[3], data[4]);
        return;
    }
    sweep_t *s = &g_sweep[ch - 1];
    s->step      = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
    s->period_ms = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
    if (s->step == 0) s->step = 1;
    if (s->period_ms == 0) s->period_ms = 1;
    if (action == 1) {
        s->active = true;
        s->current = s->v_start;
        s->dir = (s->v_stop >= s->v_start) ? (int16_t)s->step : -(int16_t)s->step;
        s->last_tick = hw_tick_ms();
    } else {
        s->active = false;
    }
    proto_send_frame(CMD_SWEEP_CTRL, data[0], data[1], data[2], data[3], data[4]);
}

static void sweep_apply(uint8_t ch, sweep_t *s) {
    if (s->mode == SWEEP_MODE_FREQ) {
        g_pwm[ch - 1].freq_hz = s->current;
        hw_pwm_apply_freq(ch);
    } else {
        uint16_t d = s->current;
        if (d > 100) d = 100;
        g_pwm[ch - 1].duty_pct = (uint8_t)d;
        hw_pwm_apply_duty(ch);
    }
}

void proto_sweep_tick(void) {
    uint32_t now = hw_tick_ms();
    for (uint8_t ch = 1; ch <= PWM_CHANNELS; ++ch) {
        sweep_t *s = &g_sweep[ch - 1];
        if (!s->active) continue;
        if ((uint32_t)(now - s->last_tick) < s->period_ms) continue;
        s->last_tick = now;
        int32_t nxt = (int32_t)s->current + s->dir;
        if (nxt >= (int32_t)s->v_stop) {
            s->current = s->v_stop;
            s->dir = -(int16_t)s->step;
        } else if (nxt <= (int32_t)s->v_start) {
            s->current = s->v_start;
            s->dir = +(int16_t)s->step;
        } else {
            s->current = (uint16_t)nxt;
        }
        sweep_apply(ch, s);
    }
}

/* ---------- config push ---------- *
 * Запись (8 байт):
 *   0      channel  1..8
 *   1      mode     0=off, 1=static, 2=blink, 3=sweep_duty, 4=sweep_freq
 *   2..3   freq     u16 LE, Гц
 *   4      duty     0..100
 *   5      step     шаг sweep
 *   6..7   period   u16 LE, мс
 */
#define CFG_MODE_OFF        0u
#define CFG_MODE_STATIC     1u
#define CFG_MODE_BLINK      2u
#define CFG_MODE_SWEEP_DUTY 3u
#define CFG_MODE_SWEEP_FREQ 4u

static void config_apply_record(const uint8_t *r) {
    uint8_t  ch     = r[0];
    uint8_t  mode   = r[1];
    uint16_t freq   = (uint16_t)r[2] | ((uint16_t)r[3] << 8);
    uint8_t  duty   = r[4];
    uint8_t  step   = r[5];
    uint16_t period = (uint16_t)r[6] | ((uint16_t)r[7] << 8);

    if (ch < 1 || ch > PWM_CHANNELS) return;
    if (duty > 100) duty = 100;

    sweep_t *s = &g_sweep[ch - 1];

    switch (mode) {
    case CFG_MODE_OFF:
        s->active = false;
        g_pwm[ch - 1].enabled = false;
        hw_pwm_apply_state(ch);
        break;

    case CFG_MODE_STATIC:
        s->active = false;
        if (freq > 0) g_pwm[ch - 1].freq_hz = freq;
        g_pwm[ch - 1].duty_pct = duty;
        g_pwm[ch - 1].enabled  = true;
        hw_pwm_apply_freq(ch);
        hw_pwm_apply_duty(ch);
        hw_pwm_apply_state(ch);
        break;

    case CFG_MODE_BLINK:
        /* Мерцание freq Гц = ШИМ с заданной частотой и duty=50% (ЛР4 п.1.2) */
        s->active = false;
        if (freq > 0) g_pwm[ch - 1].freq_hz = freq;
        g_pwm[ch - 1].duty_pct = 50;
        g_pwm[ch - 1].enabled  = true;
        hw_pwm_apply_freq(ch);
        hw_pwm_apply_duty(ch);
        hw_pwm_apply_state(ch);
        break;

    case CFG_MODE_SWEEP_DUTY:
        /* Линейный sweep duty 0..100 с заданным шагом и периодом (ЛР4 п.2.1) */
        if (freq > 0) g_pwm[ch - 1].freq_hz = freq;
        g_pwm[ch - 1].enabled = true;
        hw_pwm_apply_freq(ch);
        hw_pwm_apply_state(ch);
        s->mode      = SWEEP_MODE_DUTY;
        s->v_start   = 0;
        s->v_stop    = 100;
        s->step      = step ? step : 1;
        s->period_ms = period ? period : 1;
        s->current   = 0;
        s->dir       = (int16_t)s->step;
        s->active    = true;
        s->last_tick = hw_tick_ms();
        break;

    case CFG_MODE_SWEEP_FREQ:
        g_pwm[ch - 1].duty_pct = duty ? duty : 50;
        g_pwm[ch - 1].enabled  = true;
        hw_pwm_apply_duty(ch);
        hw_pwm_apply_state(ch);
        s->mode      = SWEEP_MODE_FREQ;
        s->v_start   = freq ? freq : 100;
        s->v_stop    = (uint16_t)((uint32_t)s->v_start * 10u > 0xFFFFu
                                  ? 0xFFFFu
                                  : (uint32_t)s->v_start * 10u);
        s->step      = step ? step : 100;
        s->period_ms = period ? period : 50;
        s->current   = s->v_start;
        s->dir       = (int16_t)s->step;
        s->active    = true;
        s->last_tick = hw_tick_ms();
        break;

    default:
        break;
    }
}

void config_apply_buf(const uint8_t *buf, uint8_t n_records) {
    for (uint8_t i = 0; i < n_records; ++i)
        config_apply_record(&buf[i * 8u]);
}

/* ---------- button ---------- */

static void handle_btn_set(const uint8_t data[5]) {
    for (int i = 0; i < BTN_SEQ_LEN; ++i) {
        uint8_t v = data[i];
        g_btn_seq[i].channel = v & 0x0F;
        g_btn_seq[i].enable  = (v >> 4) & 0x01;
        g_btn_seq[i].order   = (v >> 5) & 0x07;
        g_btn_seq[i].used    = (v != 0);
    }
    s_btn_pos = 0;
}

void proto_on_button(void) {
    /* Если все data[n] == 0 — нажатие кнопки не приводит ни к каким действиям */
    bool any_used = false;
    for (int i = 0; i < BTN_SEQ_LEN; ++i) {
        if (g_btn_seq[i].used) { any_used = true; break; }
    }
    if (!any_used) return;

    /* Выполнить очередной шаг последовательности */
    uint8_t tries = 0;
    while (tries < BTN_SEQ_LEN) {
        btn_action_t *a = &g_btn_seq[s_btn_pos];
        s_btn_pos = (uint8_t)((s_btn_pos + 1) % BTN_SEQ_LEN);
        tries++;
        if (!a->used || a->channel == 0 || a->channel > PWM_CHANNELS) continue;
        g_pwm[a->channel - 1].enabled = a->enable ? true : false;
        hw_pwm_apply_state(a->channel);
        break;
    }

    /* Отправить кадр [0x7C][0x7B][data0..4][xor] обратно ПК */
    uint8_t data[5];
    for (int i = 0; i < BTN_SEQ_LEN; ++i) {
        btn_action_t *a = &g_btn_seq[i];
        data[i] = (uint8_t)((a->order << 5) | ((a->enable & 1u) << 4) | (a->channel & 0x0Fu));
    }
    proto_send_frame(CMD_BTN_SET, data[0], data[1], data[2], data[3], data[4]);
}

/* ---------- command table & dispatch ---------- */

typedef enum {
    F_NONE      = 0,
    F_ECHO_FULL = 1u << 0,
    F_BLOCK_DMA = 1u << 1,
} cmd_flag_t;

typedef struct {
    uint8_t  cmd;
    uint8_t  flags;
    void   (*handler)(const uint8_t data[5]);
} cmd_entry_t;

static const cmd_entry_t CMD_TABLE[] = {
    { CMD_PING,         F_NONE,      handle_ping         },
    { CMD_SET_FREQ,     F_ECHO_FULL, handle_set_freq     },
    { CMD_SET_DUTY,     F_ECHO_FULL, handle_set_duty     },
    { CMD_SET_STATE,    F_ECHO_FULL, handle_set_state    },
    { CMD_SWEEP_SET,    F_NONE,      handle_sweep_set    },
    { CMD_SWEEP_CTRL,   F_NONE,      handle_sweep_ctrl   },
    { CMD_SET_INVERT,   F_NONE,      handle_set_invert   },
    { CMD_GET_FREQ,     F_NONE,      handle_get_freq     },
    { CMD_GET_DUTY,     F_NONE,      handle_get_duty     },
    { CMD_GET_STATE,    F_NONE,      handle_get_state    },
    { CMD_SNAP_FREQ_LO, F_NONE,      handle_snap_freq_lo },
    { CMD_SNAP_FREQ_HI, F_NONE,      handle_snap_freq_hi },
    { CMD_SNAP_DC_LO,   F_NONE,      handle_snap_dc_lo   },
    { CMD_SNAP_DC_HI,   F_NONE,      handle_snap_dc_hi   },
    { CMD_DMA_ON,       F_BLOCK_DMA, handle_dma_on       },
    { CMD_DMA_OFF,      F_NONE,      handle_dma_off      },
    { CMD_CONFIG_PUSH,  F_BLOCK_DMA, handle_config_push  },
    { CMD_SET_BAUD,     F_BLOCK_DMA, handle_set_baud     },
    { CMD_BTN_SET,      F_ECHO_FULL, handle_btn_set      },
};

void frame_dispatch(uint8_t cmd, const uint8_t data[5]) {
    for (size_t i = 0; i < ARRAY_LEN(CMD_TABLE); ++i) {
        if (CMD_TABLE[i].cmd != cmd) continue;
        if ((CMD_TABLE[i].flags & F_BLOCK_DMA) && proto_dma_active()) return;
        CMD_TABLE[i].handler(data);
        if (CMD_TABLE[i].flags & F_ECHO_FULL)
            proto_send_frame(cmd, data[0], data[1], data[2], data[3], data[4]);
        return;
    }
    proto_send_frame(cmd, data[0], data[1], data[2], data[3], data[4]);
}
