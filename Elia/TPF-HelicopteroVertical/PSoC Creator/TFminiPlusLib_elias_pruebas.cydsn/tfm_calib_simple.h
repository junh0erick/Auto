/* Auto-generated calibration header (MATLAB) */
/* Model: fitlm: y = a + b*d + c*d2 + k_lnq*lnq + k_T*T + k_F*F + k_dlnq*d*lnq + k_dT*d*T + k_dF*d*F */

#ifndef TFM_CALIB_SIMPLE_H_
#define TFM_CALIB_SIMPLE_H_

#include <stdint.h>
#include "arm_math.h" /* float32_t */
#include <math.h>     /* logf */
 
#define TFM_CAL_A      ((float32_t)    3.795825465480922f)
#define TFM_CAL_B      ((float32_t) 0.999665425307861f)
#define TFM_CAL_C      ((float32_t)   -0.001091976037850f)
#define TFM_CAL_D      ((float32_t) 0.000030684108644f)
#define TFM_CAL_E      ((float32_t) -0.000000160862163f)
    #define TFM_CAL_K_LNQ  ((float32_t)-0.0f)
#define TFM_CAL_K_T    ((float32_t)-0.0f)
#define TFM_CAL_K_F    ((float32_t)-0.0f)
#define TFM_CAL_K_DLNQ ((float32_t)0.0f)
#define TFM_CAL_K_DT   ((float32_t)0.0f)
#define TFM_CAL_K_DF   ((float32_t)0.0f)


/*
 * Inputs:
 *   dis_sensor_cm : lectura cruda del TFMini (cm)
 *   light_raw     : intensidaddeluz (misma escala que tu Excel)
 *   temp_c        : temperatura (C)
 *   freq_prom     : freq_prom (misma escala que tu Excel)
 * Output:
 *   y (cm)  (SIN clamp)
 */
  
    
#include "tfm_ln1p_from_q.h"   /* <-- usa LUT */

static inline float32_t tfmini_correct_distance_cm_simple(float32_t dis_sensor_cm,
                                                          float32_t light_raw,
                                                          float32_t temp_c,
                                                          float32_t freq_prom)
{
    /* lnq = ln(1+q) por LUT (q >= 0) */
    if (light_raw < 0.0f) light_raw = 0.0f;
    float32_t lnq = tfm_ln1p_from_q(light_raw);

    float32_t d  = dis_sensor_cm;
    float32_t d2 = d*d;
    float32_t d3 = d2*d;
    float32_t d4 = d3*d;
    float32_t y = TFM_CAL_A
                + TFM_CAL_B*d
                + TFM_CAL_C*d2
                //+ TFM_CAL_D*d3
                //+ TFM_CAL_D*d4
                //+ TFM_CAL_K_LNQ*lnq
                //+ TFM_CAL_K_T*temp_c
                //+ TFM_CAL_K_F*freq_prom
                //+ TFM_CAL_K_DLNQ*(d*lnq)
                //+ TFM_CAL_K_DT*(d*temp_c)
                //+ TFM_CAL_K_DF*(d*freq_prom)
                ;

    return y; /* SIN clamp */
}

#endif /* TFM_CALIB_SIMPLE_H_ */
