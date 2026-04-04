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
    uint32_t t_ms = 0u, last_hb = 0u, last_meas = 0u;

    CyGlobalIntEnable;
    
    uart_PC_Start();
    uart_PC_PutString("\r\nTFMiniPlus ISR test\r\n");

    tfmini_init();
    tfmini_set_range_cm(0u, 400u);

    (void)tfmini_disable();
    CyDelay(20u);
    
    (void)tfmini_set_fps(1000u);
    CyDelay(20u);

   
    (void)tfmini_enable();
    CyDelay(20u);
   
    for (;;) {
        CyDelay(1u);
        t_ms++;

        if ((t_ms - last_hb) >= 1000u) {
            tfmini_data_t d;
            last_hb = t_ms;

            (void)tfmini_get(&d);
//            pc_printf("hb valid=%u ok=%lu bad=%lu bytes=%lu pend=%u\r\n",
//                      (unsigned)d.valid,
//                      (unsigned long)d.frames_ok,
//                      (unsigned long)d.frames_bad,
//                      (unsigned long)d.bytes,
//                      (unsigned)tfmini_sample_pending);
        }

        if ((t_ms - last_meas) >= 200u) {
            tfmini_data_t d;
            last_meas = t_ms;
            int16_t tc10 = tfmini_temp_c10_from_raw(d.temp_raw);


            if (tfmini_get(&d)) {
               pc_printf("%u %u %d.%d\r\n",
                  (unsigned)d.dist_cm,
                  (unsigned)d.strength,
                  (int)(tc10 / 10),
                  (int)((tc10 < 0) ? -(tc10 % 10) : (tc10 % 10)));
            }
        }

        /* Estilo control: solo cuando llega frame nuevo */
        if (tfmini_sample_pending) {
            tfmini_sample_pending = 0u;

            /* En tu proyecto real:
               - leer d dentro de una sección crítica o volver a tfmini_get()
               - usarlo como y(k)
             */
            /* tfmini_data_t d; tfmini_get(&d); control_sample_isr_push((float)d.dist_cm); */
        }
    }
}
