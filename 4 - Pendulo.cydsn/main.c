/* ============================================================
   main.c — Péndulo Invertido PSoC 5LP  v3
   Nueva arquitectura: PSoC = sensor/actuador únicamente.
   MATLAB ejecuta todo el control en doble precisión.

   Protocolo (uartp_pend):
     COMMAND mode:
       'r' → 'K'                         Reset
       'f' → 'R' → [Fs f32 4B handshake] → 'K'   Configurar Fs_inner
       'i' → 'K'                         Iniciar CONTROL mode
       's' → 'K'                         Stop

     CONTROL mode (ciclo por Ts_inner):
       PSoC → MATLAB : [theta int32 LE (4B)][delta_om int16 LE (2B)] = 6 bytes
       MATLAB → PSoC : [u_pwm int16 LE (2B)] = 2 bytes
       's' (byte solitario) → 'K'        Stop (vuelve a COMMAND)
   ============================================================ */
#include "project.h"
#include "pendulo.h"
#include "motor.h"
#include "uartp_pend.h"

int main(void)
{
    int32 theta_cnt;
    int16 delta_om;
    uint8 frame[6];

    CyGlobalIntEnable;

    pendulo_init();    /* QuadDec, PWM, Timer ISR, UART */
    UARTP_Init();      /* UART_1_Start + SysMode = COMMAND */

    for (;;)
    {
        if (UARTP_SysMode == UARTP_SYS_COMMAND)
        {
            /* ── COMMAND: procesar comandos de configuración ── */
            UARTP_ProcessCommand();
        }
        else /* UARTP_SYS_CONTROL */
        {
            /* ── CONTROL: en cada tick ISR → leer → enviar → aplicar u ── */
            if (g_flag_control)
            {
                g_flag_control = 0u;

                pendulo_read(&theta_cnt, &delta_om);

                /* Trama de medición: [theta int32 LE][delta_om int16 LE] */
                frame[0] = (uint8)( theta_cnt        & 0xFFu);
                frame[1] = (uint8)((theta_cnt >>  8) & 0xFFu);
                frame[2] = (uint8)((theta_cnt >> 16) & 0xFFu);
                frame[3] = (uint8)((theta_cnt >> 24) & 0xFFu);
                frame[4] = (uint8)((uint16)delta_om  & 0xFFu);
                frame[5] = (uint8)((uint16)delta_om  >> 8);
                UART_1_PutArray(frame, 6u);

                /* Aplicar último esfuerzo recibido de MATLAB */
                Motor_Control(g_last_u_pwm);
            }

            /* Sondear RX: actualiza g_last_u_pwm o gestiona stop */
            UARTP_ControlRxPoll();
        }
    }
}
