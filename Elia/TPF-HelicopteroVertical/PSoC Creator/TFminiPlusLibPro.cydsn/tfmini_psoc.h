#ifndef TFMINI_PSOC_H
#define TFMINI_PSOC_H

#include <stdint.h>
#include <stdbool.h>

#include "cytypes.h"
#include "CyLib.h"

/* Solo lo que usa la librería (NO project.h) */
#include "uart_TFminiPlus.h"
#include "isr_rx_TFminiPlus.h"

#define TFMINI_FRAME_SIZE   (9u)

/* Rango default (cm) */
#ifndef TFMINI_MIN_DIST_CM
#define TFMINI_MIN_DIST_CM  (10u)
#endif

#ifndef TFMINI_MAX_DIST_CM
#define TFMINI_MAX_DIST_CM  (150u)
#endif

typedef struct
{
    uint16_t dist_cm;
    uint16_t strength;
    int16_t  temp_raw;   /* raw del frame (ver temp_c10_from_raw) */
    uint8_t  valid;      /* 1 si hay muestra válida */
} tfmini_data_t;

/* Flag estilo “sample_pending” (1 cuando llega frame nuevo) */
extern volatile uint8_t tfmini_sample_pending;

/* API */
void     tfmini_init(void);
void     tfmini_clear_flags(void);

bool     tfmini_enable(void);
bool     tfmini_disable(void);
bool     tfmini_set_fps(uint16_t fps_hz);
void     tfmini_set_range_cm(uint16_t min_cm, uint16_t max_cm);

uint8_t  tfmini_get(tfmini_data_t* out);

/* Temp: raw -> décimas de °C */
static inline int16_t tfmini_temp_c10_from_raw(int16_t raw)
{
    /* TFmini: temp = (raw >> 3) - 256  (en °C) */
    int16_t tC = (int16_t)((raw >> 3) - 256);
    return (int16_t)(tC * 10);
}

/* Hook de calibración: lo podés re-definir como quieras */
uint16_t tfmini_calibrate_cm(uint16_t dist_cm, uint16_t strength, uint16_t fps_hz, int16_t temp_raw);

#endif /* TFMINI_PSOC_H */
