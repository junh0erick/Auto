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

    /* ---- Estado inicial ---- */
    /* Timer no se usa en v5: el timing lo maneja MATLAB con solicitudes 'g' */
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
