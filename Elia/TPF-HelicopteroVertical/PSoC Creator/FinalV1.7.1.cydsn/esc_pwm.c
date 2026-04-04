#include "project.h"
#include "esc_pwm.h"

/* Ajustá si tu ESC usa otros límites */
#define ESC_MIN_US   (1000u)
#define ESC_MAX_US   (2000u)

static uint16_t clamp_u16(uint16_t x, uint16_t lo, uint16_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void esc_pwm_init(void)
{
    PWM_ESC_Start();
    esc_pwm_write_us(ESC_MIN_US);
}

void esc_pwm_write_us(uint16_t us)
{
    us = clamp_u16(us, ESC_MIN_US, ESC_MAX_US);
    PWM_ESC_WriteCompare(us);
}
