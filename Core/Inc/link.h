#ifndef LINK_H
#define LINK_H

#include <stdint.h>

#define CFG_REC_SIZE    8u
#define CFG_MAX_RECORDS 16u
#define CFG_BUF_MAX     (CFG_REC_SIZE * CFG_MAX_RECORDS)

void link_init(void);

void handle_dma_on(const uint8_t data[5]);
void handle_dma_off(const uint8_t data[5]);
void handle_set_baud(const uint8_t data[5]);
void handle_config_push(const uint8_t data[5]);

void config_apply_buf(const uint8_t *buf, uint8_t n_records);

#endif
