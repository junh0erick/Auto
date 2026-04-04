/* Auto-generated calibration header (MATLAB) */
/* Model: y = A + B*d + C*d2 + K_LNQ*lnq + K_T*T + K_F*F + K_DLNQ*(d*lnq) + K_DT*(d*T) + K_DF*(d*F) */

#ifndef TFM_CALIB_SIMPLE_H_
#define TFM_CALIB_SIMPLE_H_

#include <stdint.h>
#include "arm_math.h" /* float32_t */
#include <math.h>     /* logf (si usás la función con light_raw) */

#define TFM_CAL_A      ((float32_t)0.0f)
#define TFM_CAL_B      ((float32_t)0.0f)
#define TFM_CAL_C      ((float32_t)0.0f)
#define TFM_CAL_K_LNQ  ((float32_t)0.0f)
#define TFM_CAL_K_T    ((float32_t)0.0f)
#define TFM_CAL_K_F    ((float32_t)0.0f)
#define TFM_CAL_K_DLNQ ((float32_t)0.0f)
#define TFM_CAL_K_DT   ((float32_t)0.0f)
#define TFM_CAL_K_DF   ((float32_t)0.0f)

/*
 * Variante SIN logf: pasás lnq = log(1+light_raw) ya calculado.
 */
static inline float32_t tfmini_correct_distance_cm_simple_lnq(float32_t dis_sensor_cm,
                                      float32_t lnq,
                                      float32_t temp_c,
                                      float32_t freq_prom)
{
    float32_t d  = dis_sensor_cm;
    float32_t d2 = d*d;
    float32_t y = TFM_CAL_A
                + TFM_CAL_B*d
                + TFM_CAL_C*d2
                + TFM_CAL_K_LNQ*lnq
                + TFM_CAL_K_T*temp_c
                + TFM_CAL_K_F*freq_prom
                + TFM_CAL_K_DLNQ*(d*lnq)
                + TFM_CAL_K_DT*(d*temp_c)
                + TFM_CAL_K_DF*(d*freq_prom);
    return y;
}

/*
 * Variante con logf: calcula lnq = log(1+light_raw).
 * OJO: requiere linkear libm (si tu toolchain no lo hace, usá la _lnq).
 */
static inline float32_t tfmini_correct_distance_cm_simple(float32_t dis_sensor_cm,
                                 float32_t light_raw,
                                 float32_t temp_c,
                                 float32_t freq_prom)
{
    if (light_raw < 0.0f) light_raw = 0.0f;
    float32_t lnq = logf(1.0f + light_raw);
    return tfmini_correct_distance_cm_simple_lnq(dis_sensor_cm, lnq, temp_c, freq_prom);
}

#endif /* TFM_CALIB_SIMPLE_H_ */
