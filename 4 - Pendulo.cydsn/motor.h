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
   CONVERSIÓN VOLTAJE → PWM
   Polinomio Horner grado 4: PWM = f(V), V en [0, 12]
   Coeficientes calibrados en el banco de pruebas.
   ============================================================ */
#define PWM_P1  -1.101764e-01f
#define PWM_P2   4.057681e+00f
#define PWM_P3  -3.073966e+01f
#define PWM_P4   1.321242e+02f
#define PWM_P5   1.250859e+02f

/* ============================================================
   PROTOTIPOS
   Motor_Control(velocidad):
     velocidad en [-1264, +1264]
     signo → dirección (IN1/IN2), magnitud → PWM compare
   PWM_Desde_Voltaje(v):
     v en [-12, +12] V → PWM en [-1264, +1264]
     preserva signo para marcha atrás
   ============================================================ */
void Motor_Control(int16 velocidad);
void Motor_Brake(void);
void Motor_Free(void);
int16 PWM_Desde_Voltaje(float v);

#endif /* MOTOR_H */
