#include "project.h"
#include "uart.h"
#include "uart_dbg.h"
#include "tfmini_psoc.h"

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

/* =========================
   Debug printf por UART dbg
   ========================= */
static void dbg_printf(const char *fmt, ...)
{
    char buf[220];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uart_dbg_PutString(buf);
}

/* Temp en decimas de °C: (raw/8 - 256) * 10 = raw*10/8 - 2560 */
static int32_t temp_c10_from_raw(int16_t raw)
{
    return ((int32_t)raw * 10) / 8 - 2560;
}

int main(void)
{
    CyGlobalIntEnable;

    /* Arrancar UARTs */
    uart_Start();       /* UART que recibe del TFMini */
    uart_dbg_Start();   /* UART hacia PuTTY */

    /* Arrancar librería */
    tfmini_init();
    tfmini_set_range(0, 4000); /* ajustá según tu rango real */

    uart_dbg_PutString("\r\nTFMini lib test\r\n");

    uint32_t t_ms = 0;
    uint32_t last_meas = 0;
    uint32_t last_hb   = 0;

    for (;;)
    {
        CyDelay(1);
        t_ms++;

        /* IMPORTANTÍSIMO: drenar UART del TFMini lo más seguido posible */
        (void)tfmini_poll();

        /* ===== Heartbeat cada 1 s (siempre) ===== */
        if ((t_ms - last_hb) >= 1000u)
        {
            last_hb = t_ms;

            tfmini_data_t d = (tfmini_data_t){0};
            uint8_t valid = tfmini_get(&d);  /* si tu tfmini_get() aún no copia cuando no hay valid, d queda 0 por init */

            dbg_printf("hb valid=%u ok=%lu bad=%lu bytes=%lu\r\n",
                       (unsigned)valid,
                       (unsigned long)d.frames_ok,
                       (unsigned long)d.frames_bad,
                       (unsigned long)d.bytes);
        }

        /* ===== Medición cada 200 ms (solo si hay válido) ===== */
        if ((t_ms - last_meas) >= 200u)
        {
            last_meas = t_ms;

            tfmini_data_t d = (tfmini_data_t){0};
            uint8_t valid = tfmini_get(&d);

            if (valid)
            {
                int32_t tc10 = temp_c10_from_raw(d.temp_raw);

                dbg_printf("dist=%u strength=%u temp=%ld.%ldC\r\n",
                           (unsigned)d.dist,
                           (unsigned)d.strength,
                           (long)(tc10 / 10),
                           (long)((tc10 < 0) ? -(tc10 % 10) : (tc10 % 10)));
            }
        }
    }
}
