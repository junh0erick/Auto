/* ============================================================
   pendulo.c — Implementación de sensado y timer de control
   Proyecto: Péndulo Invertido — PSoC 5LP (control en MATLAB)
   ============================================================ */
#include "pendulo.h"
#include "motor.h"

/* ============================================================
   Variables globales compartidas
   ============================================================ */
volatile uint8  g_flag_control     = 0u;
volatile int32  g_prev_motor_count = 0;
volatile int16  g_last_u_pwm       = 0;

/* ============================================================
   ISR del timer de control
   Dispara cada Ts_inner = 1 / Fs_inner.
   SOLO levanta la bandera — todo el trabajo se hace en main.
   ============================================================ */
CY_ISR(Control_Timer_ISR)
{
    Timer_1_ReadStatusRegister();   /* limpiar interrupción */

    g_flag_control = 1u;
}


/* ============================================================
   pendulo_init()
   Arranca los periféricos de hardware del péndulo.
   Debe llamarse antes del loop principal.
   ============================================================ */
void pendulo_init(void)
{
    /* ---- Motor ---- */
    PWM_Motor_Start();
    Motor_Free();                    /* estado seguro al arrancar */

    /* ---- Encoders ---- */
    QuadDec_1_Start();
    QuadDec_2_Start();
    g_prev_motor_count = (int32)QuadDec_1_GetCounter();

    /* ---- Timer de control ----
       Clock = 24 MHz  →  period = 24 000 000 / Fs - 1
       Ejemplos:
         200 Hz  (5.000 ms) → 119999
         500 Hz  (2.000 ms) →  47999
        1000 Hz  (1.000 ms) →  23999
       Cambiar DEFAULT_FS_HZ para ajustar la frecuencia de arranque. */
#define DEFAULT_FS_HZ  200u
    Timer_1_WritePeriod((TIMER_CLOCK_HZ / DEFAULT_FS_HZ) - 1u);
    Timer_1_Start();
    isr_1_StartEx(Control_Timer_ISR);

    /* ---- Estado inicial ---- */
    g_flag_control = 0u;
    g_last_u_pwm   = 0;
}

/* ============================================================
   pendulo_read()
   Lee ambos encoders y calcula el delta de velocidad del motor.
   Llamar UNA vez por tick de control.
   ============================================================ */
void pendulo_read(int32 *theta_counts, int16 *delta_omega_counts)
{
    int32 curr_motor;
    int32 delta;

    /* ángulo del péndulo (QuadDec_2) */
    *theta_counts = (int32)QuadDec_2_GetCounter();

    /* velocidad del motor: cuentas por Ts_inner (QuadDec_1) */
    curr_motor   = (int32)QuadDec_1_GetCounter();
    delta        = curr_motor - g_prev_motor_count;
    g_prev_motor_count = curr_motor;

    /* saturar a rango int16 (no debería ocurrir a velocidades normales) */
    if (delta >  32767) delta =  32767;
    if (delta < -32768) delta = -32768;
    *delta_omega_counts = (int16)delta;
}

/* ============================================================
   pendulo_timer_set_fs()
   Recalcula y escribe el periodo del Timer_1.
   Timer_1 debe estar configurado con Clock = 24 MHz en PSoC Creator.
   ============================================================ */
