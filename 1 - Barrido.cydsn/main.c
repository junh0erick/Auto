#include "project.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/*******************************************************************************
* DEFINICIONES
*******************************************************************************/

// Configuración del PWM
#define PWM_PERIOD          1264

// Límites de velocidad
#define MOTOR_MAX           PWM_PERIOD
#define MOTOR_MIN           -PWM_PERIOD

// Encoder del motor: cuentas por revolución mecánica
// QuadDec usa modo x4 → si el encoder tiene N PPR, CPR = 4*N
// Ajustar según el encoder físico utilizado
#define ENCODER_CPR         1040u

// Tiempo de muestreo en segundos.
// clk_Timer = 24 MHz, period = 240000 cuentas
//   T = 240000 / 24000000 = 0.01 s  (10 ms)
#define SAMPLE_TIME_S       0.01f

/*******************************************************************************
* VARIABLES GLOBALES — VELOCIDAD DEL MOTOR
*******************************************************************************/

volatile int32 encoder_prev_count = 0;
volatile float motor_speed_rpm    = 0.0f;   // Positivo = adelante, negativo = atrás

/*******************************************************************************
* ISR DEL TIMER_1 — Cálculo periódico de velocidad del motor
*
* Se ejecuta cada SAMPLE_TIME_S segundos al llegar el Timer_1 a terminal count.
* Fórmula: RPM = (delta_cuentas * 60) / (ENCODER_CPR * SAMPLE_TIME_S)
*******************************************************************************/

CY_ISR(Motor_Speed_ISR)
{
    int32 current_count;
    int32 delta_counts;

    // Leer posición actual del encoder
    current_count = QuadDec_1_GetCounter();

    // Delta con signo (positivo = adelante, negativo = atrás)
    delta_counts = current_count - encoder_prev_count;
    encoder_prev_count = current_count;

    // Calcular RPM
    motor_speed_rpm = ((float)delta_counts * 60.0f) /
                      ((float)ENCODER_CPR * SAMPLE_TIME_S);

    // Limpiar flag de interrupción del Timer_1
    Timer_1_ReadStatusRegister();
}

/*******************************************************************************
* FUNCIONES DE CONTROL DEL MOTOR
*******************************************************************************/

/**
 * @brief Controla el motor DC con el L298N
 * @param velocidad: -2524 (máx. atrás) a +2524 (máx. adelante)
 * 
 * Pines utilizados:
 *   - Pin_ENA (P2[7]): PWM para velocidad
 *   - Pin_IN1 (P2[6]): Control de dirección 1
 *   - Pin_IN2 (P2[5]): Control de dirección 2
 */
void Motor_Control(int16 velocidad) {
    
    // Saturación de velocidad
    if (velocidad > MOTOR_MAX) velocidad = MOTOR_MAX;
    if (velocidad < MOTOR_MIN) velocidad = MOTOR_MIN;
    
    if (velocidad > 0) {
        // Motor ADELANTE
        Pin_IN1_Write(1);
        Pin_IN2_Write(0);
        PWM_Motor_WriteCompare((uint16)velocidad);
    }
    else if (velocidad < 0) {
        // Motor ATRÁS
        Pin_IN1_Write(0);
        Pin_IN2_Write(1);
        PWM_Motor_WriteCompare((uint16)(-velocidad));
    }
    else {
        // DETENER (freno activo - cortocircuito)
        Pin_IN1_Write(0);
        Pin_IN2_Write(0);
        PWM_Motor_WriteCompare(0);
    }
}

/**
 * @brief Controla el motor con porcentaje
 * @param percent: -100 (máx. atrás) a +100 (máx. adelante)
 */
void Motor_Control_Percent(int8 percent) {
    
    int16 pwm_value;
    
    // Saturación
    if (percent > 100) percent = 100;
    if (percent < -100) percent = -100;
    
    // Convertir porcentaje a valor PWM
    pwm_value = (abs(percent) * PWM_PERIOD) / 100;
    
    if (percent > 0) {
        // Adelante
        Pin_IN1_Write(1);
        Pin_IN2_Write(0);
        PWM_Motor_WriteCompare((uint16)pwm_value);
    }
    else if (percent < 0) {
        // Atrás
        Pin_IN1_Write(0);
        Pin_IN2_Write(1);
        PWM_Motor_WriteCompare((uint16)pwm_value);
    }
    else {
        // Detener
        Pin_IN1_Write(0);
        Pin_IN2_Write(0);
        PWM_Motor_WriteCompare(0);
    }
}

/**
 * @brief Frena el motor activamente (cortocircuito)
 */
void Motor_Brake(void) {
    Pin_IN1_Write(0);
    Pin_IN2_Write(0);
    PWM_Motor_WriteCompare(PWM_PERIOD);  // PWM al 100%
}

/**
 * @brief Deja el motor libre (sin freno)
 */
void Motor_Free(void) {
    Pin_IN1_Write(0);
    Pin_IN2_Write(0);
    PWM_Motor_WriteCompare(0);  // PWM apagado
}

/*******************************************************************************
* FUNCIÓN PRINCIPAL
*******************************************************************************/

int main(void)
{
    char  rx_buf[8];
    uint8 rx_idx  = 0u;
    uint8 rx_char;
    int16 current_pwm = 0;
    char  tx_buf[40];
    uint16 print_cnt = 0u;

    CyGlobalIntEnable;

    // Inicializar componentes
    PWM_Motor_Start();
    QuadDec_1_Start();
    UART_1_Start();

    // Timer_1 para medición de velocidad (ISR cada 10 ms)
    Timer_1_Start();
    isr_1_StartEx(Motor_Speed_ISR);

    UART_1_PutString("=== Motor Control ===\r\n");
    UART_1_PutString("Ingrese PWM (-2524 a 2524) + Enter\r\n");
    UART_1_PutString("PWM:     0 | RPM:     0.0\r\n");

    for(;;)
    {
        /*----------------------------------------------------------------------
        * Recepción de PWM por UART
        * Escribir el número en PuTTY y presionar Enter para aplicarlo.
        * Soporta negativos (ej: -1200 + Enter)
        *---------------------------------------------------------------------*/
        rx_char = UART_1_GetChar();
        if (rx_char != 0u)
        {
            if (rx_char == '\r' || rx_char == '\n')
            {
                if (rx_idx > 0u)
                {
                    rx_buf[rx_idx] = '\0';
                    current_pwm = (int16)atoi(rx_buf);
                    Motor_Control(current_pwm);
                    rx_idx = 0u;
                    UART_1_PutString("\r\n");
                }
            }
            else if (rx_idx < (sizeof(rx_buf) - 1u))
            {
                UART_1_PutChar(rx_char);    // eco: ver lo que se escribe
                rx_buf[rx_idx++] = (char)rx_char;
            }
        }

        /*----------------------------------------------------------------------
        * Imprimir PWM y velocidad cada 500 ms
        *---------------------------------------------------------------------*/
        print_cnt++;
        if (print_cnt >= 50u)
        {
            print_cnt = 0u;
            sprintf(tx_buf, "PWM: %5d | RPM: %5d\r\n",
                    (int)current_pwm, (int)motor_speed_rpm);
            UART_1_PutString(tx_buf);
        }

        CyDelay(10);    // 10 ms por iteración → 50 iter = 500 ms
    }
}