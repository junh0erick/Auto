/* ============================================================
   motor.c — Driver del motor DC vía L298N
   Proyecto: Péndulo Invertido — PSoC 5LP (control en MATLAB)
   ============================================================ */
#include "motor.h"

/* ============================================================
   Motor_Control()
   Aplica el esfuerzo calculado por MATLAB al puente H L298N.

   velocidad: int16 en [-1264, +1264]
     > 0  → IN1=1, IN2=0 (sentido horario)
     < 0  → IN1=0, IN2=1 (sentido antihorario)
     = 0  → IN1=0, IN2=0 (motor libre)
   ============================================================ */
void Motor_Control(int16 velocidad)
{
    /* saturación de seguridad */
    if (velocidad > MOTOR_MAX) velocidad =  MOTOR_MAX;
    if (velocidad < MOTOR_MIN) velocidad =  MOTOR_MIN;

    if (velocidad > 0)
    {
        Pin_IN1_Write(1u);
        Pin_IN2_Write(0u);
        PWM_Motor_WriteCompare((uint16)velocidad);
    }
    else if (velocidad < 0)
    {
        Pin_IN1_Write(0u);
        Pin_IN2_Write(1u);
        PWM_Motor_WriteCompare((uint16)(-velocidad));
    }
    else
    {
        Pin_IN1_Write(0u);
        Pin_IN2_Write(0u);
        PWM_Motor_WriteCompare(0u);
    }
}

/* ============================================================
   Motor_Brake()
   Freno activo: cortocircuita las bobinas del motor.
   ============================================================ */
void Motor_Brake(void)
{
    Pin_IN1_Write(0u);
    Pin_IN2_Write(0u);
    PWM_Motor_WriteCompare((uint16)PWM_PERIOD);
}

/* ============================================================
   Motor_Free()
   Libera el motor (sin corriente, sin freno).
   Estado seguro al arrancar o detener el control.
   ============================================================ */
void Motor_Free(void)
{
    Pin_IN1_Write(0u);
    Pin_IN2_Write(0u);
    PWM_Motor_WriteCompare(0u);
}
