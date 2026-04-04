#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float prev_error;
    float out_max;
    float out_min;
} PID_t;

float PID_Compute(PID_t *pid, float setpoint, float measured);
void  PID_Reset(PID_t *pid);

#endif /* PID_H */
