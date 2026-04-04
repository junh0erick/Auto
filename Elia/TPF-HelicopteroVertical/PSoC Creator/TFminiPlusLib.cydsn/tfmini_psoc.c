#include "tfmini_psoc.h"

/* Acople a tu instancia UART del PSoC Creator.
   Si tu componente no se llama "uart", cambiá esto. */
#include "uart.h"

/* ===== Estado interno ===== */
static uint8_t  s_buf[9];
static uint8_t  s_idx;

static tfmini_data_t s_last;

static uint16_t s_min_dist = 1;
static uint16_t s_max_dist = 1200;

static void parser_reset(void)
{
    s_idx = 0;
}

float tfmini_temp_c_from_raw(int16_t raw)
{
    return ((float)raw / 8.0f) - 256.0f;
}

void tfmini_init(void)
{
    parser_reset();

    s_last.dist       = 0;
    s_last.strength   = 0;
    s_last.temp_raw   = 0;
    s_last.temp_c     = 0.0f;
    s_last.frames_ok  = 0;
    s_last.frames_bad = 0;
    s_last.bytes      = 0;
    s_last.valid      = 0;
}

void tfmini_set_range(uint16_t min_dist, uint16_t max_dist)
{
    s_min_dist = min_dist;
    s_max_dist = max_dist;
}

/* Alimenta 1 byte al parser.
   Retorna 1 si cerró un frame válido y actualizó s_last. */
static uint8_t feed(uint8_t b)
{
    /* Sync con header 0x59 0x59 */
    if (s_idx == 0)
    {
        if (b == 0x59) { s_buf[s_idx++] = b; }
        return 0;
    }
    if (s_idx == 1)
    {
        if (b == 0x59) { s_buf[s_idx++] = b; }
        else { s_idx = 0; }
        return 0;
    }

    s_buf[s_idx++] = b;

    if (s_idx >= 9)
    {
        uint8_t sum = 0;
        for (uint8_t i = 0; i < 8; i++) sum += s_buf[i];

        parser_reset();

        if (sum != s_buf[8])
        {
            s_last.frames_bad++;
            return 0;
        }

        /* Decode */
        uint16_t dist     = (uint16_t)s_buf[2] | ((uint16_t)s_buf[3] << 8);
        uint16_t strength = (uint16_t)s_buf[4] | ((uint16_t)s_buf[5] << 8);
        int16_t  temp_raw = (int16_t)((uint16_t)s_buf[6] | ((uint16_t)s_buf[7] << 8));

        /* Filtro de plausibilidad */
        if (dist < s_min_dist || dist > s_max_dist)
        {
            /* Lo consideramos “frame válido” a nivel checksum, pero lo descartamos como medición */
            s_last.frames_bad++;
            return 0;
        }

        s_last.dist      = dist;
        s_last.strength  = strength;
        s_last.temp_raw  = temp_raw;
        s_last.temp_c    = tfmini_temp_c_from_raw(temp_raw);
        s_last.frames_ok++;
        s_last.valid = 1;

        return 1;
    }

    return 0;
}

uint8_t tfmini_poll(void)
{
    uint8_t got = 0;

    while (uart_GetRxBufferSize() != 0u)
    {
        uint8_t b = uart_ReadRxData();
        s_last.bytes++;
        if (feed(b)) got = 1;
    }

    return got;
}

uint8_t tfmini_get(tfmini_data_t *out)
{
    if (!out) return 0;
    if (s_last.valid == 0) return 0;

    *out = s_last;
    return 1;
}
