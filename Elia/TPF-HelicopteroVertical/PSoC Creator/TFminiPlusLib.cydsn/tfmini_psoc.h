#ifndef TFMINI_PSOC_H
#define TFMINI_PSOC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint16_t dist;         /* distancia (unidad según TFMini; normalmente cm) */
    uint16_t strength;     /* calidad/strength */
    int16_t  temp_raw;     /* crudo */
    float    temp_c;       /* °C (temp_raw/8 - 256) */

    uint32_t frames_ok;
    uint32_t frames_bad;
    uint32_t bytes;

    uint8_t  valid;        /* 1 si el último frame guardado es válido */
} tfmini_data_t;

/* Inicializa estado interno (no arranca UART, eso lo hacés vos) */
void tfmini_init(void);

/* Ajusta rango válido de distancia para filtrar outliers */
void tfmini_set_range(uint16_t min_dist, uint16_t max_dist);

/* Lee todo lo disponible del UART, actualiza parser.
   Retorna 1 si capturó AL MENOS un frame válido (y guardó el último). */
uint8_t tfmini_poll(void);

/* Copia el último dato válido (si hay) a *out.
   Retorna 1 si out es válido, 0 si todavía no hay frame válido. */
uint8_t tfmini_get(tfmini_data_t *out);

/* Helpers */
float tfmini_temp_c_from_raw(int16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* TFMINI_PSOC_H */
