/* ============================================================
   motor.h — Driver del motor DC vía L298N (simplificado)
   Proyecto: Péndulo Invertido — PSoC 5LP (control en MATLAB)

   Sin controlador PI embebido: MATLAB calcula todo el esfuerzo.
   El PSoC solo aplica Motor_Control(u_pwm) con int16 en [-1264, +1264].
   ============================================================ */
#ifndef MOTOR_H
#define MOTOR_H

#include "project.h"

/* ============================================================
   PARÁMETROS DE HARDWARE
   ============================================================ */

/* PWM — coincide con período configurado en PSoC Creator */
#define PWM_PERIOD      1264

/* Requerido por pid.c (legado — no se usa en este proyecto) */
#define SAMPLE_TIME_S   0.005f
#define MOTOR_MAX       ( PWM_PERIOD)
#define MOTOR_MIN       (-PWM_PERIOD)

/* ============================================================
   PROTOTIPOS
   Motor_Control(velocidad):
     velocidad en [-1264, +1264]
     signo → dirección (IN1/IN2), magnitud → PWM compare
   ============================================================ */
void Motor_Control(int16 velocidad);
void Motor_Brake(void);
void Motor_Free(void);

#endif /* MOTOR_H */
