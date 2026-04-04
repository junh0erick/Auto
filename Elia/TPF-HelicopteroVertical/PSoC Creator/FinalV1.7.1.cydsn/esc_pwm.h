#ifndef ESC_PWM_H
#define ESC_PWM_H

#include <stdint.h>

/* Inicializa el PWM del ESC y deja en mínimo */
void esc_pwm_init(void);

/* Escribe pulso en microsegundos (satura 1000..2000) */
void esc_pwm_write_us(uint16_t us);

#endif
