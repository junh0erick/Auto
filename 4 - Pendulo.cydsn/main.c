/* ============================================================
   main.c — Péndulo Invertido PSoC 5LP  v5
   Arquitectura: PSoC = sensor/actuador, MATLAB = control.

   Protocolo (uartp_pend):
     COMMAND mode:
       'r' → 'K'                          Reset
       'f' → 'R' → [Fs f32 4B handshake] → 'K'   Configurar timer
       'i' → 'K'                          Iniciar CONTROL mode
       's' → 'K'                          Stop (también desde CONTROL)

     CONTROL mode (request-response por muestra):
       MATLAB → PSoC : 'g'               Solicitar muestra
       PSoC → MATLAB : [theta int32 LE (4B)][delta_om int16 LE (2B)] = 6 bytes
       MATLAB → PSoC : [u_pwm int16 LE (2B)]  → Motor_Control()
       Stop: 's' en estado 0 → Motor_Free + 'K' + vuelve a COMMAND
   ============================================================ */
#include "project.h"
#include "pendulo.h"
#include "motor.h"
#include "uartp_pend.h"

int main(void)
{
    int32  theta_cnt;
    int16  delta_om;
    uint8  frame[6];
    uint8  ctrl_phase = 0u;   /* 0=wait 'g'/'s'  1=wait u_lsb  2=wait u_msb */
    uint8  u_lsb      = 0u;

    CyGlobalIntEnable;

    pendulo_init();   /* QuadDec, PWM, Timer ISR, UART */
    UARTP_Init();     /* UART_1_Start + SysMode = COMMAND */

    for (;;)
    {
        if (UARTP_SysMode == UARTP_SYS_COMMAND)
        {
            /* ── COMMAND: procesar comandos de configuración ── */
            UARTP_ProcessCommand();
            ctrl_phase = 0u;   /* estado limpio al entrar en CONTROL */
        }
        else /* UARTP_SYS_CONTROL */
        {
            /* ── CONTROL: ciclo request-response dirigido por MATLAB ── */
            while (UART_1_GetRxBufferSize() > 0u)
            {
                uint8 b = UART_1_ReadRxData();

                if (ctrl_phase == 0u)
                {
                    if (b == (uint8)'g')
                    {
                        /* Leer sensores y enviar trama de 6 bytes */
                        pendulo_read(&theta_cnt, &delta_om);
                        frame[0] = (uint8)( theta_cnt        & 0xFFu);
                        frame[1] = (uint8)((theta_cnt >>  8) & 0xFFu);
                        frame[2] = (uint8)((theta_cnt >> 16) & 0xFFu);
                        frame[3] = (uint8)((theta_cnt >> 24) & 0xFFu);
                        frame[4] = (uint8)((uint16)delta_om  & 0xFFu);
                        frame[5] = (uint8)((uint16)delta_om  >> 8);
                        UART_1_PutArray(frame, 6u);
                        ctrl_phase = 1u;
                    }
                    else if (b == (uint8)'s')
                    {
                        /* Stop: detener motor, volver a COMMAND mode */
                        Motor_Free();
                        g_last_u_pwm  = 0;
                        UARTP_SysMode = UARTP_SYS_COMMAND;
                        { uint8 k = (uint8)'K'; UART_1_PutArray(&k, 1u); }
                        ctrl_phase = 0u;
                        break;  /* salir del while → siguiente iteración en COMMAND */
                    }
                    /* otros bytes en estado 0: ignorar */
                }
                else if (ctrl_phase == 1u)
                {
                    /* Primer byte del par u_pwm (LSB) */
                    u_lsb      = b;
                    ctrl_phase = 2u;
                }
                else /* ctrl_phase == 2u */
                {
                    /* Segundo byte (MSB): completar int16 y aplicar al motor */
                    g_last_u_pwm = (int16)((uint16)u_lsb | ((uint16)b << 8u));
                    Motor_Control(g_last_u_pwm);
                    ctrl_phase = 0u;
                }
            }
        }
    }
}
