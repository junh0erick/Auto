#ifndef TFM_LN1P_FROM_Q_H_
#define TFM_LN1P_FROM_Q_H_

#include "arm_math.h"
#include <stdint.h>

/* tu LUT autogenerada */
#include "tfm_ln1p_lut.h"

/* Interpolación lineal:
   q en [0..TFM_LN1P_Q_MAX] -> ln(1+q)
*/
static inline float32_t tfm_ln1p_from_q(float32_t q)
{
    if (q <= 0.0f) {
        return tfm_ln1p_lut[0];
    }

    float32_t qmax = (float32_t)TFM_LN1P_Q_MAX;
    if (q >= qmax) {
        return tfm_ln1p_lut[TFM_LN1P_LUT_N - 1u];
    }

    /* map q -> idx real en [0..N-1] */
    float32_t x = q * ((float32_t)(TFM_LN1P_LUT_N - 1u)) / qmax;

    uint32_t i = (uint32_t)x;
    if (i >= (TFM_LN1P_LUT_N - 1u)) {
        return tfm_ln1p_lut[TFM_LN1P_LUT_N - 1u];
    }

    float32_t frac = x - (float32_t)i;
    float32_t y0 = tfm_ln1p_lut[i];
    float32_t y1 = tfm_ln1p_lut[i + 1u];

    return y0 + frac * (y1 - y0);
}

#endif /* TFM_LN1P_FROM_Q_H_ */
