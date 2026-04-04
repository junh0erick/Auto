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
    Timer_1_ReadStatusRegister();   /* limpia la bandera del timer */
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

    /* ---- Timer de control (periodo por defecto: 5 ms @ 24 MHz) ---- */
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
void pendulo_timer_set_fs(float fs_inner_hz)
{
    uint32 period;

    if (!(fs_inner_hz > 0.0f)) return;
    if (fs_inner_hz > 10000.0f) fs_inner_hz = 10000.0f; /* máx 10 kHz */

    period = (uint32)((float)TIMER_CLOCK_HZ / fs_inner_hz);
    if (period < 2u) period = 2u;   /* mínimo 2 cuentas */

    /* Timer_1_WritePeriod toma (period - 1) en PSoC 5LP */
    Timer_1_WritePeriod(period - 1u);
}
