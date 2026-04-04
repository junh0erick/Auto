#ifndef TFM_CALIB_SIMPLE_H_
#define TFM_CALIB_SIMPLE_H_

/* y = A + B*d + C*d^2  (ajustá desde MATLAB) */
#define TFM_CAL_A   (3.795825465480922f)
#define TFM_CAL_B   (0.999665425307861f)
#define TFM_CAL_C   (-0.001091976037850f)

static inline float tfmini_correct_distance_cm_simple(float d_cm)
{
    return (TFM_CAL_A + (TFM_CAL_B * d_cm) + (TFM_CAL_C * d_cm * d_cm));
}

#endif /* TFM_CALIB_SIMPLE_H_ */
