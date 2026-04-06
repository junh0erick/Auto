/* ============================================================
   main.c — Péndulo Invertido PSoC 5LP  v6
   Arquitectura: PSoC = control + sensor/actuador, MATLAB = display.

   Protocolo (uartp_pend):
     COMMAND mode:
       'r' → 'K'                          Reset
       'f' → 'R' → [Fs f32 4B handshake] → 'K'   Compatibilidad
       'p' → [len 2B LE][216B payload][xsum] → 'K'/'E'  Set params
       'i' → 'K'                          Iniciar STREAM mode
       's' → 'K'                          Stop

     STREAM mode (PSoC-driven, v6):
       PSoC → MATLAB: 9×field = 36 bytes/tick
       MATLAB → PSoC: 'u' + [ref f32]     Actualizar referencia inner
                      's'                  Stop

   Telemetría 36 bytes por tick:
       y1       = omega [rad/s]      (float32 LE, bytes  0- 3)
       y2       = theta [rad]        (float32 LE, bytes  4- 7)
       u1       = esfuerzo PWM       (float32 LE, bytes  8-11)
       u2       = ref inner          (float32 LE, bytes 12-15)
       x1i      = inner xhat[0]     (float32 LE, bytes 16-19)
       x2i      = inner xhat[1]     (float32 LE, bytes 20-23)
       x1o      = outer xhat[0]     (float32 LE, bytes 24-27)
       x2o      = outer xhat[1]     (float32 LE, bytes 28-31)
       elapsed  = ticks ctrl_step   (uint32  LE, bytes 32-35)
                  clock = 24 MHz → time_us = elapsed / 24.0
   ============================================================ */
#include "project.h"
#include "pendulo.h"
#include "motor.h"
#include "uartp_pend.h"
#include "ctrl_pend.h"

/* Tamaño de trama de telemetría (8×float32 + 1×uint32 = 36 bytes) */
#define TELEM_FRAME_SZ  36u

int main(void)
{
    uint8  frame[TELEM_FRAME_SZ];
    uint8  *p;
    float  t_y1, t_y2, t_u1, t_u2;
    float  t_x1i, t_x2i, t_x1o, t_x2o;
    uint32 cap_before, cap_after, elapsed;
    uint8  intr;

    CyGlobalIntEnable;

    ctrl_init();           /* inicializar módulo de control */
    pendulo_init();        /* QuadDec, PWM, Timer ISR */
    UARTP_Init();          /* UART_1_Start + SysMode = COMMAND */

    for (;;)
    {
        /* ══ Aplicar período de timer pendiente ══
           ctrl_apply_coeffs() calcula ctrl_period_ticks cuando recibe Fs.
           Aplicar aquí en contexto main (no ISR).                          */
        if (ctrl_period_pending)
        {
            ctrl_period_pending = 0u;
            Timer_1_Stop();
            Timer_1_WritePeriod(ctrl_period_ticks);
            Timer_1_Start();
        }

        if (UARTP_SysMode == UARTP_SYS_COMMAND)
        {
            /* ── COMMAND: procesar comandos de configuración ── */
            UARTP_ProcessCommand();
        }
        else /* UARTP_SYS_CONTROL (= STREAM mode) */
        {
            /* ── STREAM: tick de control + envío de telemetría ── */
            if (g_flag_control)
            {
                g_flag_control = 0u;

                /* 1. Ejecutar paso de control — capturar ticks antes y después */
                Timer_1_SoftwareCapture();
                cap_before = Timer_1_ReadCapture();
#if DEBUG_PINS_ENABLED
                pin_flag_Write(1u);
#endif
                ctrl_step();
#if DEBUG_PINS_ENABLED
                pin_flag_Write(0u);
#endif
                Timer_1_SoftwareCapture();
                cap_after = Timer_1_ReadCapture();
                /* Timer cuenta hacia abajo → elapsed = before - after */
                elapsed = (cap_before >= cap_after) ? (cap_before - cap_after) : 0u;

                /* 2. Leer telemetría con sección crítica mínima */
                intr = CyEnterCriticalSection();
                t_y1  = ctrl_telem_y1;
                t_y2  = ctrl_telem_y2;
                t_u1  = ctrl_telem_u1;
                t_u2  = ctrl_telem_u2;
                t_x1i = ctrl_telem_x1i;
                t_x2i = ctrl_telem_x2i;
                t_x1o = ctrl_telem_x1o;
                t_x2o = ctrl_telem_x2o;
                ctrl_telem_ready = 0u;
                CyExitCriticalSection(intr);

                /* 3. Serializar 8 × float32 LE en trama de 32 bytes */
                p = (uint8*)&t_y1;
                frame[0]=p[0];  frame[1]=p[1];  frame[2]=p[2];  frame[3]=p[3];
                p = (uint8*)&t_y2;
                frame[4]=p[0];  frame[5]=p[1];  frame[6]=p[2];  frame[7]=p[3];
                p = (uint8*)&t_u1;
                frame[8]=p[0];  frame[9]=p[1];  frame[10]=p[2]; frame[11]=p[3];
                p = (uint8*)&t_u2;
                frame[12]=p[0]; frame[13]=p[1]; frame[14]=p[2]; frame[15]=p[3];
                p = (uint8*)&t_x1i;
                frame[16]=p[0]; frame[17]=p[1]; frame[18]=p[2]; frame[19]=p[3];
                p = (uint8*)&t_x2i;
                frame[20]=p[0]; frame[21]=p[1]; frame[22]=p[2]; frame[23]=p[3];
                p = (uint8*)&t_x1o;
                frame[24]=p[0]; frame[25]=p[1]; frame[26]=p[2]; frame[27]=p[3];
                p = (uint8*)&t_x2o;
                frame[28]=p[0]; frame[29]=p[1]; frame[30]=p[2]; frame[31]=p[3];
                /* elapsed ticks de ctrl_step (uint32 LE) */
                frame[32]=(uint8)(elapsed);
                frame[33]=(uint8)(elapsed >> 8u);
                frame[34]=(uint8)(elapsed >> 16u);
                frame[35]=(uint8)(elapsed >> 24u);

                /* 4. Enviar por UART */
                UART_1_PutArray(frame, TELEM_FRAME_SZ);
            }

            /* ── RX en STREAM gestionado por isr_2 / UARTP_Rx_ISR ── */
        }
    }
}
