/* ============================================================
   ctrl_pend.c — Two-plant on-PSoC controller  v6.3
   CMSIS-DSP: f32 + Q31 / Q15 / Q7 paths.

   Tipo numérico seleccionable (g_num_type) para el observer SS:
     F32  → arm_dot_prod/scale/add_f32      (siempre preciso)
     Q31  → arm_dot_prod/scale/add_q31      (entero, eficiente CM3)
     Q15  → arm_dot_prod/scale/add_q15      (SIMD, muy eficiente CM3)
     Q7   → arm_dot_prod/scale/add_q7       (baja precisión)
     F64/F16 → fallback a F32 en PSoC

   Normalización para paths Q:
     q_scale (c[16]): todas las señales físicas (y, xhat, u, ref, vint)
     se dividen por q_scale antes de convertir a Q.
     Los coeficientes (A,B,C,D,K,L,Kx) se almacenan PRE-CONVERTIDOS a Q
     SIN normalizar por q_scale (sus valores ya deben ser |coef| < 1).
     NOTA: si |K[i]|, |L[i]| > 1, la conversión a Q satura — normal para
     sistemas físicos con ganancias grandes. El usuario ajusta q_scale y
     diseña el controlador normalizado para usar paths Q con precisión.

   Integrador siempre en float (acumula suma de errores sin Ts —
   convención Elia). La ley de control final se hace en float tras
   recuperar xhat del path Q.

   TF mode: usa siempre F32 (DF2T scalar, Q no aporta para 6 coef).

   Basado en Elia control_app.c (helicóptero FinalV1.8):
     - TF DF2T orden 5 idéntico
     - SS predictor/actual idéntico (innov, z=Ax+Bu, xhat=z+Linv)
     - u_out tracking (saturado) idéntico a g_u_out de Elia
     - vint += e (sin Ts) idéntico

   Build: ARM_MATH_CM3 en Preprocessor Defines.
   ============================================================ */

#define ARM_MATH_CM3    /* PSoC 5LP = Cortex-M3, sin FPU */
#include "arm_math.h"

#include "ctrl_pend.h"
#include "pendulo.h"
#include "motor.h"
#include <string.h>

/* ============================================================
   Constantes físicas
   ============================================================ */
#define PI_F            3.14159265f
#define MOTOR_CPR_F     1040.0f
#define PENDULO_CPR_F   10000.0f
#define PWM_MAX_F       1264.0f
#define PWM_MIN_F      (-1264.0f)

/* ============================================================
   Struct de estado por planta
   ============================================================ */
typedef struct {
    uint8  mode;            /* CTRL_MODE_* */

    /* TF – Direct Form II Transposed, hasta orden 5 */
    float  tf_b[6];
    float  tf_a[6];         /* a[0] siempre 1 tras normalización */
    float  tf_w[5];         /* estados de retardo */

    /* SS – 2 estados, float */
    float  A[4];            /* 2×2 row-major: [A11 A12 A21 A22] */
    float  B[2];            /* 2×1 */
    float  C[2];            /* 1×2 */
    float  D;
    float  L[2];            /* ganancia de observador */
    float  K[2];            /* ganancia de realimentación */
    float  Kx;              /* Nbar (sin integrador) o Ki (con integrador) */

    /* SS – estados del observador (float, siempre) */
    float  xhat[2];         /* estimado posterior */
    float  zhat[2];         /* estimado prior (observer actual) */
    float  vint;            /* acumulador integral (float, sin Ts) */

    float  ref;             /* setpoint actual */
    float  u_out;           /* salida anterior saturada (para observer) */

    /* Decimación (solo outer, planta 1) */
    uint16 N;
    uint16 cnt;

    /* Normalización para Q-format (c[16], default 1.0f).
       Señales: x_q = float_to_q(x / q_scale).
       Coeficientes: almacenados sin dividir por q_scale. */
    float  q_scale;

    /* Pre-convertidos Q31 (SS mode, arm_float_to_q31) */
    q31_t  qA31[4];
    q31_t  qB31[2];
    q31_t  qC31[2];
    q31_t  qK31[2];
    q31_t  qL31[2];
    q31_t  qD31;
    q31_t  qKx31;

    /* Pre-convertidos Q15 */
    q15_t  qA15[4];
    q15_t  qB15[2];
    q15_t  qC15[2];
    q15_t  qK15[2];
    q15_t  qL15[2];
    q15_t  qD15;
    q15_t  qKx15;

    /* Pre-convertidos Q7 */
    q7_t   qA7[4];
    q7_t   qB7[2];
    q7_t   qC7[2];
    q7_t   qK7[2];
    q7_t   qL7[2];
    q7_t   qD7;
    q7_t   qKx7;

} CtrlLoop;

/* ============================================================
   Estado del módulo
   ============================================================ */
static CtrlLoop g_lp[PLANT_COUNT];
static float    g_Ts      = 0.005f;
static uint8    g_running = 0u;

/* Tipo numérico activo */
static uint8    g_num_type = CTRL_NUM_F32;

/* Telemetría */
volatile float ctrl_telem_u1    = 0.0f;
volatile float ctrl_telem_y1    = 0.0f;
volatile float ctrl_telem_u2    = 0.0f;
volatile float ctrl_telem_y2    = 0.0f;
volatile float ctrl_telem_x1i   = 0.0f;
volatile float ctrl_telem_x2i   = 0.0f;
volatile float ctrl_telem_x1o   = 0.0f;
volatile float ctrl_telem_x2o   = 0.0f;
volatile uint8 ctrl_telem_ready = 0u;

/* Timer */
volatile uint8  ctrl_period_pending = 0u;
volatile uint32 ctrl_period_ticks   = (TIMER_CLOCK_HZ / 200u) - 1u;  /* 200 Hz por defecto */

/* Saturación configurable */
static float    g_sat_min = PWM_MIN_F;
static float    g_sat_max = PWM_MAX_F;

/* ============================================================
   Helpers escalares genéricos
   ============================================================ */
static float satf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

static uint8 mode_ss(uint8 m)
{
    return (m == CTRL_MODE_SS_PRED_NOI || m == CTRL_MODE_SS_ACT_NOI ||
            m == CTRL_MODE_SS_PRED_I   || m == CTRL_MODE_SS_ACT_I);
}

static uint8 mode_has_i(uint8 m)
{
    return (m == CTRL_MODE_SS_PRED_I) || (m == CTRL_MODE_SS_ACT_I);
}


/* ============================================================
   Q-format inline conversions
   Convención: todos los valores representados como fracción de
   q_scale, es decir: q_val = physical_val / q_scale ∈ [-1, 1).
   ============================================================ */

/* Float normalizado [-1,1) → Q31 */
static inline q31_t flt_to_q31n(float v) {
    if (v >= 1.0f)  return  (q31_t)0x7FFFFFFF;
    if (v < -1.0f)  return  (q31_t)0x80000000;
    return (q31_t)(v * 2147483648.0f);
}
/* Q31 → float normalizado [-1,1) */
static inline float q31_to_flt(q31_t v) {
    return (float)v * (1.0f / 2147483648.0f);
}

/* Float normalizado [-1,1) → Q15 */
static inline q15_t flt_to_q15n(float v) {
    if (v >= 1.0f)  return  (q15_t)0x7FFF;
    if (v < -1.0f)  return  (q15_t)0x8000;
    return (q15_t)(v * 32768.0f);
}
/* Q15 → float normalizado */
static inline float q15_to_flt(q15_t v) {
    return (float)v * (1.0f / 32768.0f);
}

/* Float normalizado [-1,1) → Q7 */
static inline q7_t flt_to_q7n(float v) {
    if (v >= 1.0f)  return  (q7_t)0x7F;
    if (v < -1.0f)  return  (q7_t)0x80;
    return (q7_t)(v * 128.0f);
}
/* Q7 → float normalizado */
static inline float q7_to_flt(q7_t v) {
    return (float)v * (1.0f / 128.0f);
}

/* Saturating add helpers para escalares Q */
static inline q31_t qsat31(q63_t v) {
    if (v > (q63_t)0x7FFFFFFF)  return (q31_t)0x7FFFFFFF;
    if (v < (q63_t)0xFFFFFFFF80000000LL) return (q31_t)0x80000000;
    return (q31_t)v;
}
static inline q15_t qsat15(q31_t v) {
    if (v >  0x7FFF) return (q15_t)0x7FFF;
    if (v < (q31_t)0xFFFF8000) return (q15_t)0x8000;
    return (q15_t)v;
}
static inline q7_t qsat7(q15_t v) {
    if (v >  0x7F) return (q7_t)0x7F;
    if (v < (q15_t)0xFF80) return (q7_t)0x80;
    return (q7_t)v;
}

/* ============================================================
   TF – Direct Form II Transposed step
   Igual al patrón de Elia. CMSIS no aporta para 6 coeficientes
   escalares; DF2T es ya la implementación óptima.
   ============================================================ */
static float tf_step(CtrlLoop *lp, float x)
{
    float y = lp->tf_b[0]*x + lp->tf_w[0];
    lp->tf_w[0] = lp->tf_w[1] + lp->tf_b[1]*x - lp->tf_a[1]*y;
    lp->tf_w[1] = lp->tf_w[2] + lp->tf_b[2]*x - lp->tf_a[2]*y;
    lp->tf_w[2] = lp->tf_w[3] + lp->tf_b[3]*x - lp->tf_a[3]*y;
    lp->tf_w[3] = lp->tf_w[4] + lp->tf_b[4]*x - lp->tf_a[4]*y;
    lp->tf_w[4] =                lp->tf_b[5]*x - lp->tf_a[5]*y;
    return y;
}

/* ============================================================
   SS helpers — F32 (patrón idéntico a Elia control_app.c)
   arm_dot_prod_f32: dot product
   arm_scale_f32:    v * escalar → destino
   arm_add_f32:      suma elemento a elemento
   ============================================================ */

/* ŷ = C·x + D·u */
static float ss_yhat_f32(CtrlLoop *lp, const float x[2], float u)
{
    float out = 0.0f;
    arm_dot_prod_f32((float32_t*)lp->C, (float32_t*)x, 2u, (float32_t*)&out);
    return out + lp->D * u;
}

/* z = A·x + B·u  (por fila, igual que Elia dot2_cmsis) */
static void ss_predict_f32(CtrlLoop *lp, const float x[2], float u, float z[2])
{
    arm_dot_prod_f32((float32_t*)&lp->A[0], (float32_t*)x, 2u, (float32_t*)&z[0]);
    z[0] += lp->B[0] * u;
    arm_dot_prod_f32((float32_t*)&lp->A[2], (float32_t*)x, 2u, (float32_t*)&z[1]);
    z[1] += lp->B[1] * u;
}

/* u_cmd = −K·xhat + Kx*(vint o ref) */
static float ss_u_cmd_f32(CtrlLoop *lp)
{
    float kx = 0.0f;
    arm_dot_prod_f32((float32_t*)lp->K, (float32_t*)lp->xhat, 2u, (float32_t*)&kx);
    float extra = mode_has_i(lp->mode) ? (lp->Kx * lp->vint)
                                       : (lp->Kx * lp->ref);
    return extra - kx;
}

/* Observer predictor:
   innov = y − ŷ(xhat, u_prev)
   z = A·xhat + B·u_k
   xhat ← z + L·innov                                         */
static void ss_obs_pred_f32(CtrlLoop *lp, float y, float u_prev, float u_k)
{
    float innov = y - ss_yhat_f32(lp, lp->xhat, u_prev);
    float z[2], Linv[2];
    ss_predict_f32(lp, lp->xhat, u_k, z);
    arm_scale_f32((float32_t*)lp->L, innov, (float32_t*)Linv, 2u);
    arm_add_f32((float32_t*)z, (float32_t*)Linv, (float32_t*)lp->xhat, 2u);
}

/* Observer actual — corrección: xhat ← zhat + L·innov       */
static void ss_obs_act_correct_f32(CtrlLoop *lp, float y, float u_prev)
{
    float innov = y - ss_yhat_f32(lp, lp->zhat, u_prev);
    float Linv[2];
    arm_scale_f32((float32_t*)lp->L, innov, (float32_t*)Linv, 2u);
    arm_add_f32((float32_t*)lp->zhat, (float32_t*)Linv, (float32_t*)lp->xhat, 2u);
}

/* Observer actual — predicción: zhat ← A·xhat + B·u         */
static void ss_obs_act_predict_f32(CtrlLoop *lp, float u_k)
{
    ss_predict_f32(lp, lp->xhat, u_k, lp->zhat);
}

/* ============================================================
   SS helpers — Q31
   Señales normalizadas: x_q = flt_to_q31n(x / q_scale)
   Coeficientes pre-convertidos: qA31, qB31, etc. (= arm_float_to_q31 del valor físico)

   arm_dot_prod_q31: resultado 16.48 → >>17 para Q31
   arm_scale_q31(..., shift=0): (src × scaleFract) >> 31 → Q31
   arm_add_q31: suma saturada Q31
   ============================================================ */

static float ss_yhat_q31(CtrlLoop *lp, const float x[2], float u)
{
    float qs = lp->q_scale;
    q31_t xq[2];
    q63_t dot;

    xq[0] = flt_to_q31n(x[0] / qs);
    xq[1] = flt_to_q31n(x[1] / qs);
    q31_t uq = flt_to_q31n(u / qs);

    /* C·x: dot product, 16.48 → >> 17 → Q31 */
    arm_dot_prod_q31(lp->qC31, xq, 2u, &dot);
    q31_t Cx = (q31_t)(dot >> 17);

    /* D·u: (D_q31 × u_q) >> 31 → Q31 */
    q31_t Du = (q31_t)(((q63_t)lp->qD31 * uq) >> 31);

    return q31_to_flt(qsat31((q63_t)Cx + Du)) * qs;
}

static void ss_predict_q31(CtrlLoop *lp, const float x[2], float u, float z[2])
{
    float qs = lp->q_scale;
    q31_t xq[2], Buq[2];
    q63_t dot;

    xq[0] = flt_to_q31n(x[0] / qs);
    xq[1] = flt_to_q31n(x[1] / qs);
    q31_t uq = flt_to_q31n(u / qs);

    /* B·u: arm_scale_q31 escala el vector B por el escalar u.
       Resultado: Buq[i] = (B_q31[i] × uq) >> 31 → Q31 */
    arm_scale_q31(lp->qB31, uq, 0, Buq, 2u);

    /* Fila 0: A[0..1]·x */
    arm_dot_prod_q31(&lp->qA31[0], xq, 2u, &dot);
    q31_t Ax0 = (q31_t)(dot >> 17);
    z[0] = q31_to_flt(qsat31((q63_t)Ax0 + Buq[0])) * qs;

    /* Fila 1: A[2..3]·x */
    arm_dot_prod_q31(&lp->qA31[2], xq, 2u, &dot);
    q31_t Ax1 = (q31_t)(dot >> 17);
    z[1] = q31_to_flt(qsat31((q63_t)Ax1 + Buq[1])) * qs;
}

static float ss_u_cmd_q31(CtrlLoop *lp)
{
    float qs = lp->q_scale;
    q31_t xhat_q[2];
    q63_t dot;

    xhat_q[0] = flt_to_q31n(lp->xhat[0] / qs);
    xhat_q[1] = flt_to_q31n(lp->xhat[1] / qs);

    arm_dot_prod_q31(lp->qK31, xhat_q, 2u, &dot);
    q31_t kx = (q31_t)(dot >> 17);

    /* extra = Kx * vint_or_ref — calculado en float, luego normalizado */
    float extra_f = mode_has_i(lp->mode) ? (lp->Kx * lp->vint)
                                         : (lp->Kx * lp->ref);
    q31_t extra_q = flt_to_q31n(extra_f / qs);

    return q31_to_flt(qsat31((q63_t)extra_q - kx)) * qs;
}

static void ss_obs_pred_q31(CtrlLoop *lp, float y, float u_prev, float u_k)
{
    float qs = lp->q_scale;

    /* innov = y - ŷ(xhat, u_prev) */
    float innov = y - ss_yhat_q31(lp, lp->xhat, u_prev);
    q31_t innov_q = flt_to_q31n(innov / qs);

    /* z = A·xhat + B·u_k (resultado en float) */
    float z[2];
    ss_predict_q31(lp, lp->xhat, u_k, z);

    /* Linv = L × innov: arm_scale_q31(L_q, innov_q, shift=0, Linv, 2) */
    q31_t Linv[2];
    arm_scale_q31(lp->qL31, innov_q, 0, Linv, 2u);

    /* xhat = z + Linv (suma Q31 saturada) */
    q31_t zq[2];
    zq[0] = flt_to_q31n(z[0] / qs);
    zq[1] = flt_to_q31n(z[1] / qs);
    q31_t xhat_q[2];
    arm_add_q31(zq, Linv, xhat_q, 2u);

    lp->xhat[0] = q31_to_flt(xhat_q[0]) * qs;
    lp->xhat[1] = q31_to_flt(xhat_q[1]) * qs;
}

static void ss_obs_act_correct_q31(CtrlLoop *lp, float y, float u_prev)
{
    float qs = lp->q_scale;

    float innov = y - ss_yhat_q31(lp, lp->zhat, u_prev);
    q31_t innov_q = flt_to_q31n(innov / qs);

    q31_t Linv[2];
    arm_scale_q31(lp->qL31, innov_q, 0, Linv, 2u);

    q31_t zhat_q[2];
    zhat_q[0] = flt_to_q31n(lp->zhat[0] / qs);
    zhat_q[1] = flt_to_q31n(lp->zhat[1] / qs);
    q31_t xhat_q[2];
    arm_add_q31(zhat_q, Linv, xhat_q, 2u);

    lp->xhat[0] = q31_to_flt(xhat_q[0]) * qs;
    lp->xhat[1] = q31_to_flt(xhat_q[1]) * qs;
}

static void ss_obs_act_predict_q31(CtrlLoop *lp, float u_k)
{
    ss_predict_q31(lp, lp->xhat, u_k, lp->zhat);
}

/* ============================================================
   SS helpers — Q15
   arm_dot_prod_q15: resultado 34.30 → >>15 para Q15
   arm_scale_q15(..., shift=0): (src × scaleFract) >> 15 → Q15
   arm_add_q15: suma saturada Q15
   ============================================================ */

static float ss_yhat_q15(CtrlLoop *lp, const float x[2], float u)
{
    float qs = lp->q_scale;
    q15_t xq[2];
    q63_t dot;

    xq[0] = flt_to_q15n(x[0] / qs);
    xq[1] = flt_to_q15n(x[1] / qs);
    q15_t uq = flt_to_q15n(u / qs);

    arm_dot_prod_q15(lp->qC15, xq, 2u, &dot);
    q15_t Cx = qsat15((q31_t)(dot >> 15));

    /* D·u: (D_q15 × u_q) >> 15 → Q15 */
    q15_t Du = qsat15((q31_t)((q31_t)lp->qD15 * uq >> 15));

    return q15_to_flt(qsat15((q31_t)Cx + Du)) * qs;
}

static void ss_predict_q15(CtrlLoop *lp, const float x[2], float u, float z[2])
{
    float qs = lp->q_scale;
    q15_t xq[2], Buq[2];
    q63_t dot;

    xq[0] = flt_to_q15n(x[0] / qs);
    xq[1] = flt_to_q15n(x[1] / qs);
    q15_t uq = flt_to_q15n(u / qs);

    /* B·u: arm_scale_q15 escala el vector B por escalar u */
    arm_scale_q15(lp->qB15, uq, 0, Buq, 2u);

    arm_dot_prod_q15(&lp->qA15[0], xq, 2u, &dot);
    q15_t Ax0 = qsat15((q31_t)(dot >> 15));
    z[0] = q15_to_flt(qsat15((q31_t)Ax0 + Buq[0])) * qs;

    arm_dot_prod_q15(&lp->qA15[2], xq, 2u, &dot);
    q15_t Ax1 = qsat15((q31_t)(dot >> 15));
    z[1] = q15_to_flt(qsat15((q31_t)Ax1 + Buq[1])) * qs;
}

static float ss_u_cmd_q15(CtrlLoop *lp)
{
    float qs = lp->q_scale;
    q15_t xhat_q[2];
    q63_t dot;

    xhat_q[0] = flt_to_q15n(lp->xhat[0] / qs);
    xhat_q[1] = flt_to_q15n(lp->xhat[1] / qs);

    arm_dot_prod_q15(lp->qK15, xhat_q, 2u, &dot);
    q15_t kx = qsat15((q31_t)(dot >> 15));

    float extra_f = mode_has_i(lp->mode) ? (lp->Kx * lp->vint)
                                         : (lp->Kx * lp->ref);
    q15_t extra_q = flt_to_q15n(extra_f / qs);

    return q15_to_flt(qsat15((q31_t)extra_q - kx)) * qs;
}

static void ss_obs_pred_q15(CtrlLoop *lp, float y, float u_prev, float u_k)
{
    float qs = lp->q_scale;

    float innov = y - ss_yhat_q15(lp, lp->xhat, u_prev);
    q15_t innov_q = flt_to_q15n(innov / qs);

    float z[2];
    ss_predict_q15(lp, lp->xhat, u_k, z);

    q15_t Linv[2];
    arm_scale_q15(lp->qL15, innov_q, 0, Linv, 2u);

    q15_t zq[2];
    zq[0] = flt_to_q15n(z[0] / qs);
    zq[1] = flt_to_q15n(z[1] / qs);
    q15_t xhat_q[2];
    arm_add_q15(zq, Linv, xhat_q, 2u);

    lp->xhat[0] = q15_to_flt(xhat_q[0]) * qs;
    lp->xhat[1] = q15_to_flt(xhat_q[1]) * qs;
}

static void ss_obs_act_correct_q15(CtrlLoop *lp, float y, float u_prev)
{
    float qs = lp->q_scale;

    float innov = y - ss_yhat_q15(lp, lp->zhat, u_prev);
    q15_t innov_q = flt_to_q15n(innov / qs);

    q15_t Linv[2];
    arm_scale_q15(lp->qL15, innov_q, 0, Linv, 2u);

    q15_t zhat_q[2];
    zhat_q[0] = flt_to_q15n(lp->zhat[0] / qs);
    zhat_q[1] = flt_to_q15n(lp->zhat[1] / qs);
    q15_t xhat_q[2];
    arm_add_q15(zhat_q, Linv, xhat_q, 2u);

    lp->xhat[0] = q15_to_flt(xhat_q[0]) * qs;
    lp->xhat[1] = q15_to_flt(xhat_q[1]) * qs;
}

static void ss_obs_act_predict_q15(CtrlLoop *lp, float u_k)
{
    ss_predict_q15(lp, lp->xhat, u_k, lp->zhat);
}

/* ============================================================
   SS helpers — Q7
   arm_dot_prod_q7: resultado q31_t en 18.14 → >>7 para Q7
   arm_scale_q7(..., shift=0): (src × scaleFract) >> 7 → Q7
   arm_add_q7: suma saturada Q7
   ============================================================ */

static float ss_yhat_q7(CtrlLoop *lp, const float x[2], float u)
{
    float qs = lp->q_scale;
    q7_t xq[2];
    q31_t dot7;

    xq[0] = flt_to_q7n(x[0] / qs);
    xq[1] = flt_to_q7n(x[1] / qs);
    q7_t uq = flt_to_q7n(u / qs);

    /* C·x: dot product 18.14 → >>7 → Q7 */
    arm_dot_prod_q7(lp->qC7, xq, 2u, &dot7);
    q7_t Cx = qsat7((q15_t)(dot7 >> 7));

    /* D·u: (D_q7 × u_q) >> 7 → Q7 */
    q7_t Du = qsat7((q15_t)((q15_t)lp->qD7 * uq >> 7));

    return q7_to_flt(qsat7((q15_t)Cx + Du)) * qs;
}

static void ss_predict_q7(CtrlLoop *lp, const float x[2], float u, float z[2])
{
    float qs = lp->q_scale;
    q7_t xq[2], Buq[2];
    q31_t dot7;

    xq[0] = flt_to_q7n(x[0] / qs);
    xq[1] = flt_to_q7n(x[1] / qs);
    q7_t uq = flt_to_q7n(u / qs);

    /* B·u: arm_scale_q7 escala el vector B por escalar u */
    arm_scale_q7(lp->qB7, uq, 0, Buq, 2u);

    arm_dot_prod_q7(&lp->qA7[0], xq, 2u, &dot7);
    q7_t Ax0 = qsat7((q15_t)(dot7 >> 7));
    z[0] = q7_to_flt(qsat7((q15_t)Ax0 + Buq[0])) * qs;

    arm_dot_prod_q7(&lp->qA7[2], xq, 2u, &dot7);
    q7_t Ax1 = qsat7((q15_t)(dot7 >> 7));
    z[1] = q7_to_flt(qsat7((q15_t)Ax1 + Buq[1])) * qs;
}

static float ss_u_cmd_q7(CtrlLoop *lp)
{
    float qs = lp->q_scale;
    q7_t xhat_q[2];
    q31_t dot7;

    xhat_q[0] = flt_to_q7n(lp->xhat[0] / qs);
    xhat_q[1] = flt_to_q7n(lp->xhat[1] / qs);

    arm_dot_prod_q7(lp->qK7, xhat_q, 2u, &dot7);
    q7_t kx = qsat7((q15_t)(dot7 >> 7));

    float extra_f = mode_has_i(lp->mode) ? (lp->Kx * lp->vint)
                                         : (lp->Kx * lp->ref);
    q7_t extra_q = flt_to_q7n(extra_f / qs);

    return q7_to_flt(qsat7((q15_t)extra_q - kx)) * qs;
}

static void ss_obs_pred_q7(CtrlLoop *lp, float y, float u_prev, float u_k)
{
    float qs = lp->q_scale;

    float innov = y - ss_yhat_q7(lp, lp->xhat, u_prev);
    q7_t innov_q = flt_to_q7n(innov / qs);

    float z[2];
    ss_predict_q7(lp, lp->xhat, u_k, z);

    q7_t Linv[2];
    arm_scale_q7(lp->qL7, innov_q, 0, Linv, 2u);

    q7_t zq[2];
    zq[0] = flt_to_q7n(z[0] / qs);
    zq[1] = flt_to_q7n(z[1] / qs);
    q7_t xhat_q[2];
    arm_add_q7(zq, Linv, xhat_q, 2u);

    lp->xhat[0] = q7_to_flt(xhat_q[0]) * qs;
    lp->xhat[1] = q7_to_flt(xhat_q[1]) * qs;
}

static void ss_obs_act_correct_q7(CtrlLoop *lp, float y, float u_prev)
{
    float qs = lp->q_scale;

    float innov = y - ss_yhat_q7(lp, lp->zhat, u_prev);
    q7_t innov_q = flt_to_q7n(innov / qs);

    q7_t Linv[2];
    arm_scale_q7(lp->qL7, innov_q, 0, Linv, 2u);

    q7_t zhat_q[2];
    zhat_q[0] = flt_to_q7n(lp->zhat[0] / qs);
    zhat_q[1] = flt_to_q7n(lp->zhat[1] / qs);
    q7_t xhat_q[2];
    arm_add_q7(zhat_q, Linv, xhat_q, 2u);

    lp->xhat[0] = q7_to_flt(xhat_q[0]) * qs;
    lp->xhat[1] = q7_to_flt(xhat_q[1]) * qs;
}

static void ss_obs_act_predict_q7(CtrlLoop *lp, float u_k)
{
    ss_predict_q7(lp, lp->xhat, u_k, lp->zhat);
}

/* ============================================================
   Despacho de observer según g_num_type
   ============================================================ */
static void obs_pred(CtrlLoop *lp, float y, float u_prev, float u_k)
{
    switch (g_num_type) {
        case CTRL_NUM_Q31: ss_obs_pred_q31(lp, y, u_prev, u_k); break;
        case CTRL_NUM_Q15: ss_obs_pred_q15(lp, y, u_prev, u_k); break;
        case CTRL_NUM_Q7:  ss_obs_pred_q7 (lp, y, u_prev, u_k); break;
        default:           ss_obs_pred_f32(lp, y, u_prev, u_k); break;
    }
}
static void obs_act_correct(CtrlLoop *lp, float y, float u_prev)
{
    switch (g_num_type) {
        case CTRL_NUM_Q31: ss_obs_act_correct_q31(lp, y, u_prev); break;
        case CTRL_NUM_Q15: ss_obs_act_correct_q15(lp, y, u_prev); break;
        case CTRL_NUM_Q7:  ss_obs_act_correct_q7 (lp, y, u_prev); break;
        default:           ss_obs_act_correct_f32(lp, y, u_prev); break;
    }
}
static void obs_act_predict(CtrlLoop *lp, float u_k)
{
    switch (g_num_type) {
        case CTRL_NUM_Q31: ss_obs_act_predict_q31(lp, u_k); break;
        case CTRL_NUM_Q15: ss_obs_act_predict_q15(lp, u_k); break;
        case CTRL_NUM_Q7:  ss_obs_act_predict_q7 (lp, u_k); break;
        default:           ss_obs_act_predict_f32(lp, u_k); break;
    }
}
static float u_cmd(CtrlLoop *lp)
{
    switch (g_num_type) {
        case CTRL_NUM_Q31: return ss_u_cmd_q31(lp);
        case CTRL_NUM_Q15: return ss_u_cmd_q15(lp);
        case CTRL_NUM_Q7:  return ss_u_cmd_q7 (lp);
        default:           return ss_u_cmd_f32(lp);
    }
}

/* ============================================================
   Paso de control completo (una planta)
   Devuelve salida cruda (sin saturar).

   Patrón idéntico a Elia control_app.c:
   - u_out guarda el saturado para que el observer use lo real
   - vint += e (sin Ts) — igual que Elia g_vint
   - Anti-windup: vint += (u_sat - u) / Kx (Elia no lo tiene)
   ============================================================ */
static float run_step(CtrlLoop *lp, float y)
{
    float u      = 0.0f;
    float u_prev = lp->u_out;   /* saturado del tick anterior */
    float u_sat;

    switch (lp->mode)
    {
        case CTRL_MODE_TF:
            /* TF usa siempre F32 (DF2T escalar) */
            u = tf_step(lp, lp->ref - y);
            break;

        case CTRL_MODE_SS_PRED_NOI:
        case CTRL_MODE_SS_PRED_I:
            /* Integrador: vint += e  (sin Ts — igual que Elia control_app.c).
               Ki en MATLAB se diseña con esta convención: u_i = Ki × sum(e[k]). */
            if (mode_has_i(lp->mode)) lp->vint += (lp->ref - y);

            u     = u_cmd(lp);
            u_sat = satf(u, g_sat_min, g_sat_max);

            /* Anti-windup back-calculation: corrige vint si hay saturación.
               Sin efecto cuando u_sat == u (no satura). */
            if (mode_has_i(lp->mode) && lp->Kx != 0.0f) {
                lp->vint += (u_sat - u) / lp->Kx;
            }

            /* Observer usa u_sat — coincide con lo que ve la planta */
            obs_pred(lp, y, u_prev, u_sat);
            lp->u_out = u_sat;
            return u;

        case CTRL_MODE_SS_ACT_NOI:
        case CTRL_MODE_SS_ACT_I:
            /* Observer actual: primero corrección, luego ley de control */
            obs_act_correct(lp, y, u_prev);

            if (mode_has_i(lp->mode)) lp->vint += (lp->ref - y);

            u     = u_cmd(lp);
            u_sat = satf(u, g_sat_min, g_sat_max);

            if (mode_has_i(lp->mode) && lp->Kx != 0.0f) {
                lp->vint += (u_sat - u) / lp->Kx;
            }

            /* Predicción de zhat usa u_sat real */
            obs_act_predict(lp, u_sat);
            lp->u_out = u_sat;
            return u;

        case CTRL_MODE_OPEN_LOOP:
            u = lp->ref;
            break;

        default:    /* CTRL_MODE_OFF */
            u = 0.0f;
            break;
    }

    lp->u_out = u;
    return u;
}

/* ============================================================
   ctrl_init()
   ============================================================ */
void ctrl_init(void)
{
    uint8 p;
    memset(g_lp, 0, sizeof(g_lp));
    for (p = 0u; p < PLANT_COUNT; p++) {
        g_lp[p].mode    = CTRL_MODE_OFF;
        g_lp[p].N       = 1u;
        g_lp[p].q_scale = 1.0f;
    }
    g_running           = 0u;
    g_num_type          = CTRL_NUM_F32;
    ctrl_telem_ready    = 0u;
    ctrl_period_pending = 0u;
}

/* ============================================================
   ctrl_set_mode()
   ============================================================ */
void ctrl_set_mode(uint8 plant_id, uint8 mode)
{
    if (plant_id >= PLANT_COUNT) return;
    if (mode > CTRL_MODE_OFF) mode = CTRL_MODE_OFF;
    g_lp[plant_id].mode = mode;
}

/* ============================================================
   ctrl_apply_coeffs()
   Parsea 25 floats al struct de control.
   c[16] = q_scale para paths Q (0 → default 1.0).
   El modo DEBE estar seteado antes de llamar.
   ============================================================ */
void ctrl_apply_coeffs(uint8 plant_id, const float* c, uint16 n)
{
    if (plant_id >= PLANT_COUNT) return;
    if (!c || n < 16u) return;

    CtrlLoop *lp = &g_lp[plant_id];

    /* --- N y Fs (siempre) --- */
    {
        uint32 Nv = (uint32)(c[14] + 0.5f);
        if (Nv < 1u) Nv = 1u;
        lp->N = (uint16)(Nv & 0xFFFFu);

        float fs = c[15];
        if (fs > 0.5f && fs <= 10000.0f) {
            g_Ts = 1.0f / fs;
            uint32 period = (uint32)((float)TIMER_CLOCK_HZ / fs);
            if (period < 2u) period = 2u;
            ctrl_period_ticks   = period - 1u;
            ctrl_period_pending = 1u;
        }
    }

    /* --- q_scale (c[16], nuevo en v6.3) --- */
    if (n > 16u && c[16] > 0.0f)
        lp->q_scale = c[16];
    else
        lp->q_scale = 1.0f;

    /* --- Coeficientes TF --- */
    if (lp->mode == CTRL_MODE_TF)
    {
        uint8 i;
        for (i = 0u; i < 6u; i++) {
            lp->tf_b[i] = c[i];
            lp->tf_a[i] = c[i + 6u];
        }
        float a0 = lp->tf_a[0];
        if (a0 != 0.0f && a0 != 1.0f) {
            uint8 j;
            for (j = 0u; j < 6u; j++) lp->tf_b[j] /= a0;
            for (j = 1u; j < 6u; j++) lp->tf_a[j] /= a0;
            lp->tf_a[0] = 1.0f;
        }
        memset(lp->tf_w, 0, sizeof(lp->tf_w));
    }
    /* --- Coeficientes SS --- */
    else if (mode_ss(lp->mode))
    {
        lp->A[0] = c[0];  lp->A[1] = c[1];
        lp->A[2] = c[2];  lp->A[3] = c[3];
        lp->B[0] = c[4];  lp->B[1] = c[5];
        lp->C[0] = c[6];  lp->C[1] = c[7];
        lp->D    = c[8];
        lp->L[0] = c[9];  lp->L[1] = c[10];
        lp->K[0] = c[11]; lp->K[1] = c[12];
        lp->Kx   = c[13];
        memset(lp->xhat, 0, sizeof(lp->xhat));
        memset(lp->zhat, 0, sizeof(lp->zhat));
        lp->vint = 0.0f;

        /* --- Pre-convertir a Q31/Q15/Q7 para los paths Q ---
           arm_float_to_q31 espera valores en [-1,1); satura si no.
           Los coeficientes se almacenan SIN dividir por q_scale:
           la normalización aplica solo a las señales en cada paso.
           Condición para resultados exactos: |coef| < 1.        */
        arm_float_to_q31((float32_t*)lp->A, lp->qA31, 4u);
        arm_float_to_q31((float32_t*)lp->B, lp->qB31, 2u);
        arm_float_to_q31((float32_t*)lp->C, lp->qC31, 2u);
        arm_float_to_q31((float32_t*)lp->K, lp->qK31, 2u);
        arm_float_to_q31((float32_t*)lp->L, lp->qL31, 2u);
        lp->qD31  = flt_to_q31n(lp->D);
        lp->qKx31 = flt_to_q31n(lp->Kx);

        arm_float_to_q15((float32_t*)lp->A, lp->qA15, 4u);
        arm_float_to_q15((float32_t*)lp->B, lp->qB15, 2u);
        arm_float_to_q15((float32_t*)lp->C, lp->qC15, 2u);
        arm_float_to_q15((float32_t*)lp->K, lp->qK15, 2u);
        arm_float_to_q15((float32_t*)lp->L, lp->qL15, 2u);
        lp->qD15  = flt_to_q15n(lp->D);
        lp->qKx15 = flt_to_q15n(lp->Kx);

        arm_float_to_q7((float32_t*)lp->A, lp->qA7, 4u);
        arm_float_to_q7((float32_t*)lp->B, lp->qB7, 2u);
        arm_float_to_q7((float32_t*)lp->C, lp->qC7, 2u);
        arm_float_to_q7((float32_t*)lp->K, lp->qK7, 2u);
        arm_float_to_q7((float32_t*)lp->L, lp->qL7, 2u);
        lp->qD7  = flt_to_q7n(lp->D);
        lp->qKx7 = flt_to_q7n(lp->Kx);
    }
    /* OL / OFF: solo N y Fs importan, ya parseados */
}

/* ============================================================
   ctrl_set_num_type()
   ============================================================ */
void ctrl_set_num_type(uint8 num_type)
{
    if (num_type > CTRL_NUM_Q7) num_type = CTRL_NUM_F32;
    g_num_type = num_type;
}

uint8 ctrl_get_num_type(void)
{
    return g_num_type;
}

/* ============================================================
   ctrl_start()
   ============================================================ */
void ctrl_start(float ref_inner)
{
    uint8 p;
    for (p = 0u; p < PLANT_COUNT; p++) {
        CtrlLoop *lp = &g_lp[p];
        memset(lp->tf_w, 0, sizeof(lp->tf_w));
        memset(lp->xhat, 0, sizeof(lp->xhat));
        memset(lp->zhat, 0, sizeof(lp->zhat));
        lp->vint  = 0.0f;
        lp->u_out = 0.0f;
        lp->cnt   = 0u;
    }

    g_lp[PLANT_INNER].ref = ref_inner;
    g_lp[PLANT_OUTER].ref = 0.0f;     /* péndulo vertical = 0 rad */

    ctrl_telem_ready = 0u;
    g_running        = 1u;
}

/* ============================================================
   ctrl_stop()
   ============================================================ */
void ctrl_stop(void)
{
    g_running = 0u;
    Motor_Free();
}

/* ============================================================
   ctrl_update_ref()
   ============================================================ */
void ctrl_update_ref(float ref_inner)
{
    g_lp[PLANT_INNER].ref = ref_inner;
}

/* ============================================================
   ctrl_set_sat()
   ============================================================ */
void ctrl_set_sat(float sat_min, float sat_max)
{
    g_sat_min = (sat_min < PWM_MIN_F) ? PWM_MIN_F : sat_min;
    g_sat_max = (sat_max > PWM_MAX_F) ? PWM_MAX_F : sat_max;
    if (g_sat_min > g_sat_max) {
        g_sat_min = PWM_MIN_F;
        g_sat_max = PWM_MAX_F;
    }
}

/* ============================================================
   ctrl_step()   — llamar una vez por tick de g_flag_control
   ============================================================ */
void ctrl_step(void)
{
    int32  theta_cnt;
    int16  delta_om_cnt;

    /* Guard: ctrl_stop() (llamado desde isr_2) puede limpiar g_running
       en cualquier momento. Si ya no corremos, soltar motor y salir. */
    if (!g_running) { Motor_Free(); return; }

    pendulo_read(&theta_cnt, &delta_om_cnt);

    float y1 = (float)delta_om_cnt * (2.0f * PI_F / MOTOR_CPR_F)    / g_Ts;
    float y2 = (float)theta_cnt    * (2.0f * PI_F / PENDULO_CPR_F);

    CtrlLoop *inner = &g_lp[PLANT_INNER];
    CtrlLoop *outer = &g_lp[PLANT_OUTER];

    float u1 = 0.0f;
    float u2 = outer->u_out;

    /* --- Loop outer (cada N ticks inner) --- */
    if (outer->mode != CTRL_MODE_OFF)
    {
        outer->cnt++;
        if (outer->cnt >= outer->N)
        {
            outer->cnt = 0u;
            u2 = run_step(outer, y2);
            if (inner->mode != CTRL_MODE_OFF)
                inner->ref = u2;
        }
    }

    /* --- Loop inner (cada tick) --- */
    if (inner->mode != CTRL_MODE_OFF)
    {
        u1 = run_step(inner, y1);
        Motor_Control((int16)satf(u1, g_sat_min, g_sat_max));
    }
    else if (outer->mode != CTRL_MODE_OFF)
    {
        u1 = satf(u2, g_sat_min, g_sat_max);
        Motor_Control((int16)u1);
    }
    else
    {
        Motor_Free();
    }

    /* --- Snapshot telemetría (sección crítica mínima) --- */
    {
        uint8 intr = CyEnterCriticalSection();
        ctrl_telem_u1    = u1;
        ctrl_telem_y1    = y1;
        ctrl_telem_u2    = inner->ref;
        ctrl_telem_y2    = y2;
        ctrl_telem_x1i   = inner->xhat[0];
        ctrl_telem_x2i   = inner->xhat[1];
        ctrl_telem_x1o   = outer->xhat[0];
        ctrl_telem_x2o   = outer->xhat[1];
        ctrl_telem_ready = 1u;
        CyExitCriticalSection(intr);
    }
}
