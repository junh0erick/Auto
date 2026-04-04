#ifndef TFMINI_PSOC_H
#define TFMINI_PSOC_H

#include <stdint.h>
#include <stdbool.h>

#include "cytypes.h"
#include "CyLib.h"

/* Solo lo que usa la librería (NO project.h) */
#include "uart_TFminiPlus.h"
#include "isr_rx_TFminiPlus.h"

#define TFMINI_FRAME_SIZE (9u)

/* Rango default (cm) */
#ifndef TFMINI_MIN_DIST_CM
#define TFMINI_MIN_DIST_CM (10u)
#endif

#ifndef TFMINI_MAX_DIST_CM
#define TFMINI_MAX_DIST_CM (150u) /* 1.5 m */
#endif

typedef struct {
    uint16_t dist_cm;
    uint16_t strength;
    int16_t  temp_raw;      /* raw de sensor */
    uint32_t bytes;
    uint32_t frames_ok;
    uint32_t frames_bad;
    uint8_t  valid;
} tfmini_data_t;

void   tfmini_init(void);
void   tfmini_clear_flags(void);
bool   tfmini_enable(void);
bool   tfmini_disable(void);
bool   tfmini_set_fps(uint16_t fps_hz);
void   tfmini_set_range_cm(uint16_t min_cm, uint16_t max_cm);
uint8_t tfmini_get(tfmini_data_t* out); /* snapshot: NO consume "new sample" */

/* --- API limpia para main: consume una muestra nueva si existe --- */
bool tfmini_pop(tfmini_data_t* out);          /* devuelve true si había muestra nueva */
bool tfmini_pop_cm(uint16_t* dist_cm);        /* idem, solo distancia */

/* Temperatura helpers */
static inline float tfmini_temp_c_from_raw(uint16_t raw)
{
    return ((float)raw / 8.0f) - 256.0f;
}

/* Temp en décimas (evita float): (raw*10)/8 - 2560 */
static inline int16_t tfmini_temp_c10_from_raw(uint16_t raw)
{
    return (int16_t)(((int32_t)raw * 10) / 8 - 2560);
}

/* ÚNICA función que vos tocás para calibración */
uint16_t tfmini_calibrate_cm(uint16_t dist_cm,
                             uint16_t strength,
                             uint16_t fps_hz,
                             uint16_t temp_raw);

#endif /* TFMINI_PSOC_H */
