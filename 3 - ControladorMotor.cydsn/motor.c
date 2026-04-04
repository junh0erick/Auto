#include "motor.h"
#include "pid.h"

/*******************************************************************************
* INSTANCIA DEL CONTROLADOR PI
*******************************************************************************/

static PID_t pi_motor = {
    .kp         = PI_MOTOR_KP,
    .ki         = PI_MOTOR_KI,
    .kd         = 0.0f,
    .integrator = 0.0f,
    .prev_error = 0.0f,
    .out_max    = PI_MOTOR_OUT_MAX,
    .out_min    = PI_MOTOR_OUT_MIN
};

/*******************************************************************************
* VARIABLES GLOBALES
*******************************************************************************/

volatile float  omega_setpoint    = 0.0f;
volatile float  motor_speed_rads  = 0.0f;
volatile int16  pi_motor_output   = 0;
volatile float  motor_u           = 0.0f;
volatile uint8  flag_control      = 0u;
volatile uint16 print_cnt         = 0u;

static volatile int32 encoder_prev_count = 0;
static volatile float motor_speed_rpm    = 0.0f;

/*******************************************************************************
* CURVAS POLINOMIALES
*******************************************************************************/

float RPM_Estimado(int16 pwm)
{
    float x = (float)pwm;
    return (((RPM_P1 * x + RPM_P2) * x + RPM_P3) * x + RPM_P4) * x + RPM_P5;
}

int16 PWM_Desde_Voltaje(float v)
{
    int16 pwm = (int16)((((PWM_P1 * v + PWM_P2) * v + PWM_P3) * v + PWM_P4) * v + PWM_P5);
    if (pwm > MOTOR_MAX) pwm = MOTOR_MAX;
    if (pwm < 0)         pwm = 0;
    return pwm;
}

/*******************************************************************************
* ISR DEL TIMER_1 — Disparo periódico cada SAMPLE_TIME_S
*******************************************************************************/

CY_ISR(Motor_Speed_ISR)
{
    Timer_1_ReadStatusRegister();
    flag_control = 1u;
    print_cnt++;
}

/*******************************************************************************
* LAZO DE CONTROL — ejecutado desde main cuando flag_control == 1
*******************************************************************************/

void Motor_Stop(void)
{
    PID_Reset(&pi_motor);
    motor_u          = 0.0f;
    pi_motor_output  = 0;
    VDAC8_u_SetValue((uint8)((0.0f + 12.0f) * (255.0f / 24.0f)));
    Motor_Free();
}

void Motor_Controller_Run(void)
{
    int32 current_count = QuadDec_1_GetCounter();
    int32 delta_counts  = current_count - encoder_prev_count;
    encoder_prev_count  = current_count;

    motor_speed_rpm  = ((float)delta_counts * 60.0f) /
                       ((float)ENCODER_CPR * SAMPLE_TIME_S);
    motor_speed_rads = motor_speed_rpm * (3.14159265f / 30.0f);

    /* Setpoint = 0: liberar motor y resetear PI para evitar windup */
    if (omega_setpoint > -0.01f && omega_setpoint < 0.01f)
    {
        Motor_Stop();
        return;
    }

    float u = PID_Compute(&pi_motor, omega_setpoint, motor_speed_rads);
    motor_u = u;

    if (u >= 0.0f)
        pi_motor_output =  (int16)PWM_Desde_Voltaje( u);
    else
        pi_motor_output = -(int16)PWM_Desde_Voltaje(-u);

    Motor_Control(pi_motor_output);
}

/*******************************************************************************
* CONTROL DE HARDWARE DEL MOTOR (L298N)
*
* Pines: Pin_ENA (PWM velocidad), Pin_IN1 / Pin_IN2 (dirección)
*******************************************************************************/

void Motor_Control(int16 velocidad)
{
    if (velocidad > MOTOR_MAX) velocidad = MOTOR_MAX;
    if (velocidad < MOTOR_MIN) velocidad = MOTOR_MIN;

    if (velocidad > 0) {
        Pin_IN1_Write(1);
        Pin_IN2_Write(0);
        PWM_Motor_WriteCompare((uint16)velocidad);
    } else if (velocidad < 0) {
        Pin_IN1_Write(0);
        Pin_IN2_Write(1);
        PWM_Motor_WriteCompare((uint16)(-velocidad));
    } else {
        Pin_IN1_Write(0);
        Pin_IN2_Write(0);
        PWM_Motor_WriteCompare(0);
    }
}

/* Freno activo (cortocircuito) */
void Motor_Brake(void)
{
    Pin_IN1_Write(0);
    Pin_IN2_Write(0);
    PWM_Motor_WriteCompare(PWM_PERIOD);
}

/* Motor libre (sin freno) */
void Motor_Free(void)
{
    Pin_IN1_Write(0);
    Pin_IN2_Write(0);
    PWM_Motor_WriteCompare(0);
}
