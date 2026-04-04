#ifndef TFMINI_PSOC_H
#define TFMINI_PSOC_H

#include <stdint.h>
#include <stdbool.h>

#include "cytypes.h"
#include "CyLib.h"
#include "uart_TFminiPlus.h"
#include "isr_rx_TFminiPlus.h"

#define TFMINI_FRAME_SIZE (9u)

/* Rango útil (interno) en cm */
#define TFMINI_MIN_CM   (10u)
#define TFMINI_MAX_CM   (130u)

/* Conversión cm <-> m */
#define TFMINI_CM_TO_M  (0.01f)
#define TFMINI_M_TO_CM  (100.0f)

/* Rango útil en metros (equivalente) */
#define TFMINI_MIN_M    ((float)TFMINI_MIN_CM * TFMINI_CM_TO_M)
#define TFMINI_MAX_M    ((float)TFMINI_MAX_CM * TFMINI_CM_TO_M)

/* “hay muestra nueva lista” */
extern volatile uint8_t tfmini_sample_pending;

void  tfmini_init(void);
void  tfmini_clear_flags(void);

bool  tfmini_enable(void);
bool  tfmini_disable(void);
bool  tfmini_set_fps(uint16_t fps_hz);

/* Consume una muestra calibrada (cm) */
bool  tfmini_pop_cm(uint16_t *y_cm);

/* Consume una muestra calibrada (m) */
bool  tfmini_pop_m(float *y_m);

/* Calibración: SOLO distancia (cm -> cm calibrado) */
uint16_t tfmini_calibrate_cm(uint16_t dist_cm);

#endif /* TFMINI_PSOC_H */
