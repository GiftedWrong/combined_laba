#ifndef HAL_SHIM_H
#define HAL_SHIM_H

#include <stdint.h>
#include <stdbool.h>

void hw_pwm_apply_freq(uint8_t ch);
void hw_pwm_apply_duty(uint8_t ch);
void hw_pwm_apply_state(uint8_t ch);
void hw_pwm_apply_invert(uint8_t ch, bool invert);

void hw_uart_apply_baud(uint32_t baud);
void hw_uart_kick_tx(void);
void hw_uart_tx_flush(void);

bool hw_dma_send(const uint8_t *data, uint16_t size);
void hw_dma_abort(void);

bool hw_uart_arm_rx_dma(uint8_t *buf, uint16_t size);
void hw_uart_disarm_rx_dma(void);

uint32_t hw_tick_ms(void);

#endif
