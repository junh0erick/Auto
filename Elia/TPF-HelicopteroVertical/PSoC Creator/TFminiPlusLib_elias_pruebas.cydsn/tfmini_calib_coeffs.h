/* Auto-generated from MATLAB fitlm */
/* Model: Dist_real ~ 1 + Freq_prom + Dist_cm + Temp_C + Lux_log1p + Dist_cm2 + Freq_prom2 + Temp_C2 + Lux_log1p2 */

#ifndef TFMINI_CALIB_COEFFS_H
#define TFMINI_CALIB_COEFFS_H

#include <stdint.h>
#include "arm_math.h"   /* float32_t */
#include "tfmini_ln_lut.h"  /* tfm_ln_lux_f / tfm_ln1p_lux_f */

/* Coefficients (float32_t) */
#define TFMC_INTERCEPT ((float32_t)-30.3803082f)
#define TFMC_F ((float32_t)-0.000886993716f)
#define TFMC_D ((float32_t)0.96595186f)
#define TFMC_T_C ((float32_t)0.983333647f)
#define TFMC_L_1 ((float32_t)1.93899715f)
#define TFMC_D_2 ((float32_t)0.000113420785f)
#define TFMC_F_2 ((float32_t)7.41639894e-07f)
#define TFMC_T_C2 ((float32_t)-0.00937278476f)
#define TFMC_L_1_2 ((float32_t)-0.109950855f)

/* Inputs:
 *   Dist_cm, Freq_prom, Temp_C as float32_t
 *   Lux as uint16_t (ADC / sensor raw) used to compute ln(Lux) via LUT
 */
static inline float32_t tfmini_correct_distance_cm(float32_t Dist_cm, float32_t Freq_prom, float32_t Temp_C, uint16_t Lux)
{
    float32_t y = 0.0f;
    y += TFMC_INTERCEPT;
    y += TFMC_F * (Freq_prom);
    y += TFMC_D * (Dist_cm);
    y += TFMC_T_C * (Temp_C);
    y += TFMC_L_1 * (tfm_ln1p_lux_f(Lux));
    y += TFMC_D_2 * ((Dist_cm*Dist_cm));
    y += TFMC_F_2 * ((Freq_prom*Freq_prom));
    y += TFMC_T_C2 * ((Temp_C*Temp_C));
    y += TFMC_L_1_2 * ((tfm_ln1p_lux_f(Lux)*tfm_ln1p_lux_f(Lux)));
    return y;
}

#endif /* TFMINI_CALIB_COEFFS_H */
