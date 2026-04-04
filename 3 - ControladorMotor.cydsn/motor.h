#ifndef MOTOR_H
#define MOTOR_H

#include "project.h"
#include "pid.h"

/*******************************************************************************
* CONFIGURACIÓN DE HARDWARE
*******************************************************************************/

/* PWM */
#define PWM_PERIOD          1264
#define MOTOR_MAX           PWM_PERIOD
#define MOTOR_MIN           -PWM_PERIOD

/* Encoder del motor: cuentas por revolución mecánica
   QuadDec usa modo x4 → CPR = 4 × PPR del encoder físico */
#define ENCODER_CPR         1040u

/* Tiempo de muestreo en segundos
   clk_Timer = 24 MHz, period = 120000 cuentas → T = 0.005 s (5 ms) */
#define SAMPLE_TIME_S       0.005f

/*******************************************************************************
* COEFICIENTES POLINOMIALES (Poly4, método de Horner)
*******************************************************************************/

/* RPM = f(PWM) */
#define RPM_P1   2.264e-11f
#define RPM_P2  -6.042e-08f
#define RPM_P3   4.570e-05f
#define RPM_P4   1.611e-03f
#define RPM_P5  -8.532e-01f

/* PWM = f(Voltaje) */
#define PWM_P1  -1.101764e-01f
#define PWM_P2   4.057681e+00f
#define PWM_P3  -3.073966e+01f
#define PWM_P4   1.321242e+02f
#define PWM_P5   1.250859e+02f

/*******************************************************************************
* GANANCIAS DEL CONTROLADOR PI — LAZO INTERNO (VELOCIDAD DEL MOTOR)
*
* Esquema de control en cascada:
*   [Setpoint péndulo] → [PID externo] → omega_setpoint → [PI motor] → PWM → Motor
*******************************************************************************/

#define PI_MOTOR_KP         2.4972f
#define PI_MOTOR_KI         0.05542f
#define PI_MOTOR_OUT_MAX    12.0f    /* voltaje máximo de salida [V] */
#define PI_MOTOR_OUT_MIN   -12.0f   /* voltaje mínimo de salida [V] */

/*******************************************************************************
* VARIABLES COMPARTIDAS
*******************************************************************************/

extern volatile float  omega_setpoint;    /* setpoint [rad/s] — escrito por main */
extern volatile float  motor_speed_rads;  /* velocidad medida [rad/s] */
extern volatile int16  pi_motor_output;   /* salida PI en cuentas PWM */
extern volatile float  motor_u;           /* esfuerzo de control [V] */
extern volatile uint8  flag_control;      /* bandera de la ISR */
extern volatile uint16 print_cnt;         /* contador de muestras para print */

/*******************************************************************************
* PROTOTIPOS
*******************************************************************************/

float RPM_Estimado(int16 pwm);
int16 PWM_Desde_Voltaje(float v);

void Motor_Control(int16 velocidad);
void Motor_Brake(void);
void Motor_Free(void);
void Motor_Stop(void);   /* detiene el motor y resetea el PI */

void Motor_Controller_Run(void);
CY_ISR_PROTO(Motor_Speed_ISR);

#endif /* MOTOR_H */
