#include "project.h"
#include "motor.h"
#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    char  rx_buf[8];
    uint8 rx_idx = 0u;
    uint8 rx_char;
    char  tx_buf[64];

    CyGlobalIntEnable;

    PWM_Motor_Start();
    QuadDec_1_Start();
    UART_1_Start();
    VDAC8_u_Start();
    Timer_1_Start();
    isr_1_StartEx(Motor_Speed_ISR);

    UART_1_PutString("=== Motor PI Controller ===\r\n");
    UART_1_PutString("Ingrese omega setpoint en rad/s (ej: 10.5) + Enter\r\n");
    UART_1_PutString("SP[rad/s] | MED[rad/s] | PWM_out\r\n");
    UART_1_PutString("----------+-----------+--------\r\n");

    for(;;)
    {
        /*----------------------------------------------------------------------
        * Lazo de control — ejecutado cuando la ISR activa la bandera
        *---------------------------------------------------------------------*/
        if (flag_control)
        {
            flag_control = 0u;
            Motor_Controller_Run();

            /* Enviar esfuerzo de control cada muestra (5 ms) para MATLAB */
            int32 u_i = (int32)(motor_u * 100.0f);
            sprintf(tx_buf, "%ld\r\n", u_i);
            UART_1_PutString(tx_buf);
        }

        /*
        if (print_cnt >= 100u)
        {
            print_cnt = 0u;

            int32 sp_i  = (int32)(omega_setpoint  * 100.0f);
            int32 med_i = (int32)(motor_speed_rads * 100.0f);

            sprintf(tx_buf, "SP:%5ld.%02ld | MED:%5ld.%02ld | PWM:%6d\r\n",
                    sp_i  / 100L, sp_i  < 0 ? -sp_i  % 100L : sp_i  % 100L,
                    med_i / 100L, med_i < 0 ? -med_i % 100L : med_i % 100L,
                    (int)pi_motor_output);
            UART_1_PutString(tx_buf);
        }
        */

        /*----------------------------------------------------------------------
        * Recepción de setpoint por UART
        * Escribir el número en PuTTY y presionar Enter para aplicarlo.
        * Soporta negativos (ej: -150 + Enter → -15.0 rad/s)
        *---------------------------------------------------------------------*/
        rx_char = UART_1_GetChar();
        if (rx_char != 0u)
        {
            if (rx_char == '\r' || rx_char == '\n')
            {
                if (rx_idx > 0u)
                {
                    rx_buf[rx_idx] = '\0';
                    omega_setpoint = (float)atoi(rx_buf) / 10.0f;
                    rx_idx = 0u;
                }
            }
            else if (rx_idx < (sizeof(rx_buf) - 1u))
            {
                rx_buf[rx_idx++] = (char)rx_char;
            }
        }
    }
}
