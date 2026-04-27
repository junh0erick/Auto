/* ============================================================
   pendulo.h — Definiciones de hardware y API de sensado
   Proyecto: Péndulo Invertido — PSoC 5LP (control en MATLAB)
   ============================================================ */
#ifndef PENDULO_H
#define PENDULO_H

#include "project.h"

/* ============================================================
   ENCODER DEL PÉNDULO (QuadDec_2)
   Ajustar según hardware real:
     CPR = 4 × PPR_físico  (QuadDec en modo x4)
   ============================================================ */
#define PENDULO_CPR     10000u      /* cuentas/revolución (QuadDec x4, 2500 PPR físico) */

/* ============================================================
   ENCODER DEL MOTOR (QuadDec_1)
   Coincide con motor.h de la planta original.
   ============================================================ */
#define MOTOR_CPR       1040u       /* cuentas/revolución (QuadDec x4, 260 PPR físico) */

/* ============================================================
   TIMER DE CONTROL
   clk_Timer = 24 MHz  →  period_counts = 24e6 / Fs_inner_hz
   ============================================================ */
#define TIMER_CLOCK_HZ  24000000UL

/* ============================================================
   PINES DE DEBUG
     pin_flag  : Digital Output — HIGH durante la ejecución del paso de
                 control (ISR sube, main baja al terminar ctrl_step).
                 tc_timer está conectado por hardware al timer, sin código.
   ============================================================ */
#define DEBUG_PINS_ENABLED  1u   /* 0 = deshabilitar sin borrar código */

/* ============================================================
   VARIABLES COMPARTIDAS (escritas en main, ISR y uartp_pend)
   ============================================================ */
extern volatile uint8  g_flag_control;       /* ISR → main: tick listo    */
extern volatile int32  g_prev_motor_count;   /* conteo anterior de motor  */
extern volatile int16  g_last_u_pwm;         /* último esfuerzo de MATLAB */

/* ============================================================
   API
   ============================================================ */

/* Inicializa encoders, PWM y timer (con periodo por defecto 5 ms). */
void pendulo_init(void);

/* Lee encoders y calcula delta de motor.
   theta_counts     : QuadDec_2 (péndulo, int32)
   delta_omega_counts: delta QuadDec_1 (motor, int16, cuentas/Ts_inner) */
void pendulo_read(int32 *theta_counts, int16 *delta_omega_counts);

/* Resetea el encoder del péndulo (QuadDec_2) a 0 y resincroniza el
   conteo previo del motor (QuadDec_1) para evitar un delta espurio en
   el primer tick. Llamar en ctrl_start tras posicionar el péndulo. */
void pendulo_reset_encoders(void);

/* Reconfigura el periodo del Timer_1 para la frecuencia indicada. */
void pendulo_timer_set_fs(float fs_inner_hz);

/* Prototipo de ISR del timer de control. */
CY_ISR_PROTO(Control_Timer_ISR);

#endif /* PENDULO_H */
