#ifndef CONTROL_APP_H
#define CONTROL_APP_H

#include <stdint.h>
#include "uartp_sw.h"

/* matrices/coef accesibles si querés */
extern float tf_num[6];
extern float tf_den[6];

extern float ss_A[2][2];
extern float ss_B[2];
extern float ss_C[2];
extern float ss_D;
extern float ss_L[2];
extern float ss_K[2];
extern float ss_Ki;
extern uint8_t ss_has_integrator;

/* callbacks para UARTP */
bool control_stop_suave_step(void);
void control_start(float u0);
void control_apply_tf(const float* c, uint16_t n);
void control_apply_ss(const float* c, uint16_t n);

/* step del control (placeholder) */
void control_step(void);

#endif
