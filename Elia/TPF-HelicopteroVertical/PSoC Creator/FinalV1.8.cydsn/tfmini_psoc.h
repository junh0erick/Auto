#ifndef TFMINI_PSOC_H
#define TFMINI_PSOC_H

#include <stdint.h>
#include <stdbool.h>

#include "cytypes.h"
#include "CyLib.h"
#include "uart_TFminiPlus.h"
#include "isr_rx_TFminiPlus.h"

#define TFMINI_FRAME_SIZE (9u)

/* “hay muestra nueva lista” */
extern volatile uint8_t tfmini_sample_pending;

void  tfmini_init(void);
void  tfmini_clear_flags(void);

bool  tfmini_enable(void);
bool  tfmini_disable(void);
bool  tfmini_set_fps(uint16_t fps_hz);

/* Consume una muestra calibrada */
bool  tfmini_pop_cm(uint16_t *y_cm);

/* Calibración: SOLO distancia */
uint16_t tfmini_calibrate_cm(uint16_t dist_cm);

#endif /* TFMINI_PSOC_H */
