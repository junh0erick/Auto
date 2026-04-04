#include "project.h"
#include "uart_PC.h"
#include "tfmini_psoc.h"

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

static void pc_printf(const char *fmt, ...)
{
    char buf[220];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    uart_PC_PutString(buf);
}

int main(void)
{
    CyGlobalIntEnable;

    uart_PC_Start();
    uart_PC_PutString("\r\n"); /* opcional */

    tfmini_init();
    tfmini_set_range_cm(0u, 400u);

    (void)tfmini_disable();
    CyDelay(20u);

    (void)tfmini_set_fps(50u);     /* tu Freq_prom */
    CyDelay(20u);

    (void)tfmini_enable();
    CyDelay(20u);

    for (;;)
    {
        /* Estilo ADC: solo cuando llega frame nuevo */
        if (tfmini_sample_pending)
        {
            tfmini_sample_pending = 0u;

            tfmini_data_t d;
            if (tfmini_get(&d))
            {
                int16_t tc10 = tfmini_temp_c10_from_raw(d.temp_raw);

                /* Formato EXACTO para Excel:
                   Freq_prom dis_sensor temperatura intensidaddeluz */
                pc_printf("%u %u %d.%d %u\r\n",
                          (unsigned)50u,                 /* Freq_prom = lo que seteaste */
                          (unsigned)d.dist_cm,           /* dis_sensor */
                          (int)(tc10 / 10),              /* temperatura */
                          (int)((tc10 < 0) ? -(tc10 % 10) : (tc10 % 10)),
                          (unsigned)d.strength);         /* intensidad de luz (strength) */
            }
        }
    }
}
