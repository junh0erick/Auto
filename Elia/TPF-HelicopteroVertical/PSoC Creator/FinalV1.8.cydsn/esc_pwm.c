#include "project.h"
#include "esc_pwm.h"

/* Ajustá si tu ESC usa otros límites */


static uint16_t clamp_u16(uint16_t x)
{
    if (x < ESC_MIN_US) return ESC_MIN_US;
    if (x > ESC_MAX_US) return ESC_MAX_US;
    return x;
}

void esc_pwm_init(void)
{
    PWM_ESC_Start();
    esc_pwm_write_us(ESC_MIN_US);
}

void esc_pwm_write_us(uint16_t us)
{
    us = clamp_u16(us);
    PWM_ESC_WriteCompare(us);
}
