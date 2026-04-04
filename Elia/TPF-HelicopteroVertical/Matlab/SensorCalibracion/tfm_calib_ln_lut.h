/* Auto-generated calibration header */
#ifndef TFM_CALIB_LN_LUT_H_
#define TFM_CALIB_LN_LUT_H_

#include <stdint.h>
#include "arm_math.h"   /* float32_t */

#define TFM_CALIB_NBETA (7)

static const float32_t tfm_calib_beta[7] = {
    1.80462151f,
    0.000126537879f,
    0.969727473f,
    -0.000857472444f,
    0.0794217574f,
    0.000194203696f,
    0.00125999736f
};

#define TFM_LN_LUX_MAX      (20828u)
#define TFM_LN_LUT_SHIFT    (7u)
#define TFM_LN_LUT_STEP     (1u << TFM_LN_LUT_SHIFT)
#define TFM_LN_LUT_SIZE     (163u)

static const float32_t tfm_ln1p_lux_lut[163] = {
    0.0f,
    4.85981226f,
    5.54907608f,
    5.95324326f,
    6.24027586f,
    6.46302938f,
    6.64509106f,
    6.79905605f,
    6.93244791f,
    7.05012274f,
    7.15539646f,
    7.25063562f,
    7.33758783f,
    7.4175806f,
    7.49164534f,
    7.56060123f,
    7.62510729f,
    7.68570328f,
    7.742836f,
    7.79688025f,
    7.84815311f,
    7.8969245f,
    7.94342756f,
    7.98786402f,
    8.03040981f,
    8.07121849f,
    8.1104269f,
    8.14815617f,
    8.18451405f,
    8.21959591f,
    8.25348759f,
    8.28626919f,
    8.31801033f,
    8.34877491f,
    8.37862015f,
    8.40760136f,
    8.43576622f,
    8.46315956f,
    8.48982239f,
    8.51579189f,
    8.54110527f,
    8.56579304f,
    8.58988571f,
    8.6134119f,
    8.63639736f,
    8.65886593f,
    8.68084145f,
    8.70234394f,
    8.72339439f,
    8.74400997f,
    8.76420975f,
    8.78400898f,
    8.80342388f,
    8.82246971f,
    8.84115887f,
    8.85950565f,
    8.87752151f,
    8.89521885f,
    8.91260815f,
    8.9296999f,
    8.94650459f,
    8.96303177f,
    8.97929096f,
    8.99528885f,
    9.01103497f,
    9.0265379f,
    9.04180336f,
    9.05683994f,
    9.07165241f,
    9.08625031f,
    9.10063744f,
    9.11482048f,
    9.12880516f,
    9.14259624f,
    9.15620136f,
    9.16962242f,
    9.1828661f,
    9.19593716f,
    9.20883942f,
    9.22157669f,
    9.2341547f,
    9.24657631f,
    9.25884438f,
    9.27096462f,
    9.28293991f,
    9.2947731f,
    9.30646801f,
    9.31802845f,
    9.32945633f,
    9.34075451f,
    9.3519268f,
    9.36297607f,
    9.37390423f,
    9.38471413f,
    9.39540863f,
    9.40598965f,
    9.41646004f,
    9.42682171f,
    9.43707752f,
    9.44722939f,
    9.45727825f,
    9.46722794f,
    9.47707939f,
    9.48683548f,
    9.4964962f,
    9.50606537f,
    9.51554298f,
    9.52493191f,
    9.53423405f,
    9.5434494f,
    9.55258179f,
    9.5616312f,
    9.5705986f,
    9.57948685f,
    9.58829689f,
    9.59703064f,
    9.6056881f,
    9.61427116f,
    9.6227808f,
    9.63121986f,
    9.6395874f,
    9.64788532f,
    9.65611553f,
    9.66427803f,
    9.67237473f,
    9.68040657f,
    9.68837452f,
    9.69627857f,
    9.70412159f,
    9.71190357f,
    9.71962452f,
    9.72728729f,
    9.73489094f,
    9.74243832f,
    9.74992847f,
    9.75736332f,
    9.76474285f,
    9.77206802f,
    9.77934074f,
    9.78656006f,
    9.79372883f,
    9.80084515f,
    9.80791187f,
    9.81492996f,
    9.82189751f,
    9.82881832f,
    9.8356905f,
    9.84251595f,
    9.84929562f,
    9.85602856f,
    9.86271763f,
    9.86936188f,
    9.87596226f,
    9.88251972f,
    9.88903332f,
    9.89550591f,
    9.90193653f,
    9.90832615f,
    9.91467476f,
    9.92098331f,
    9.92725277f,
    9.93348312f,
    9.93967438f
};

static inline float32_t tfm_ln1p_lux_f(uint16_t Lux)
{
    uint32_t x = (uint32_t)Lux;
    if (x > TFM_LN_LUX_MAX) x = TFM_LN_LUX_MAX;
    uint32_t idx = (x >> TFM_LN_LUT_SHIFT);
    if (idx >= (TFM_LN_LUT_SIZE - 1u)) {
        return tfm_ln1p_lux_lut[TFM_LN_LUT_SIZE - 1u];
    }
    uint32_t x0 = (idx << TFM_LN_LUT_SHIFT);
    float32_t y0 = tfm_ln1p_lux_lut[idx];
    float32_t y1 = tfm_ln1p_lux_lut[idx + 1u];
    float32_t t  = (float32_t)(x - x0) * (1.0f / (float32_t)TFM_LN_LUT_STEP);
    return y0 + (y1 - y0) * t;
}

static inline float32_t tfm_clampf(float32_t v, float32_t lo, float32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float32_t tfmini_correct_distance_cm(float32_t Dist_cm, float32_t Freq_prom, float32_t Temp_C, uint16_t Lux)
{
    const float32_t lnq = tfm_ln1p_lux_f(Lux);
    const float32_t b0 = tfm_calib_beta[0];
    const float32_t b1 = tfm_calib_beta[1];
    const float32_t b2 = tfm_calib_beta[2];
    const float32_t b3 = tfm_calib_beta[3];
    const float32_t b4 = tfm_calib_beta[4];
    const float32_t b5 = tfm_calib_beta[5];
    const float32_t b6 = tfm_calib_beta[6];
    float32_t y = b0;
    y += b1 * Freq_prom;
    y += b2 * Dist_cm;
    y += b3 * Temp_C;
    y += b4 * lnq;
    y += b5 * (Dist_cm * Temp_C);
    y += b6 * (Dist_cm * lnq);
    return tfm_clampf(y, 12.0f, 134.0f);
}

#endif /* TFM_CALIB_LN_LUT_H_ */
