#include "pid.h"
#include "motor.h"   /* SAMPLE_TIME_S */

float PID_Compute(PID_t *pid, float setpoint, float measured)
{
    float error  = setpoint - measured;
    float p_term = pid->kp * error;

    /* Integrador con anti-windup por saturación (clamping) */
    pid->integrator += pid->ki * error * SAMPLE_TIME_S;
    if      (pid->integrator > pid->out_max) pid->integrator = pid->out_max;
    else if (pid->integrator < pid->out_min) pid->integrator = pid->out_min;

    float d_term = pid->kd * (error - pid->prev_error) / SAMPLE_TIME_S;
    pid->prev_error = error;

    float output = p_term + pid->integrator + d_term;
    if      (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;

    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integrator = 0.0f;
    pid->prev_error  = 0.0f;
}
