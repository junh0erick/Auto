#ifndef TFMINI_POLY_CALIB_H
#define TFMINI_POLY_CALIB_H

#include <stdint.h>
#include "arm_math.h"   /* float32_t */

/* y = p1*x^6 + p2*x^5 + ... + p6*x + p7 */
#define TFM_POLY_ORDER (6u)
#define TFM_P1 ((float32_t)-2.73570923e-12f)
#define TFM_P2 ((float32_t)-1.34153322e-09f)
#define TFM_P3 ((float32_t)5.72459271e-07f)
#define TFM_P4 ((float32_t)-7.39463285e-05f)
#define TFM_P5 ((float32_t)0.00469714031f)
#define TFM_P6 ((float32_t)0.837754309f)
#define TFM_P7 ((float32_t)4.27228498f)

static inline float32_t tfmini_correct_distance_cm_poly(float32_t x)
{
    float32_t y = TFM_P1;
    y = y * x + TFM_P2;
    y = y * x + TFM_P3;
    y = y * x + TFM_P4;
    y = y * x + TFM_P5;
    y = y * x + TFM_P6;
    y = y * x + TFM_P7;
    return y;
}

static inline uint16_t tfmini_calibrate_cm_poly(uint16_t dist_cm)
{
    float32_t y = tfmini_correct_distance_cm_poly((float32_t)dist_cm);
    if (y < 0.0f) y = 0.0f;
    if (y > 65535.0f) y = 65535.0f;
    return (uint16_t)(y + 0.5f);
}

#endif /* TFMINI_POLY_CALIB_H */
