#include "control_app.h"

#include "project.h"
#include "uartp_sw.h"     /* UARTP_Impl, impl enums */
#include "arm_math.h"
#include <string.h>

/* =======================
   Variables ISR -> main
   ======================= */
volatile float   control_last_y = 0.0f;
volatile uint8_t control_sample_pending = 0u;

/* “espejo” opcional (debug) */
volatile float control_ref_v = 0.0f;

/* =======================
   IO callbacks
   ======================= */
static control_sample_isr_fn_t s_sample_isr_cb = 0;
static control_write_u_fn_t    s_write_u_cb    = 0;

/* =======================
   Estado general
   ======================= */
static float g_ref = 0.0f;    /* referencia REAL */
static float g_Ts  = 1.0f;    /* sample time integrador */

static float g_u_cmd = 0.0f;  /* comando antes de offset */
static float g_u_out = 0.0f;  /* salida física (post offset + sat) */

static float g_vint  = 0.0f;  /* integrador */

/* =======================
   TF (IIR orden 5, DF2T)
   ======================= */
static float tf_b[6] = {0};
static float tf_a[6] = {0};
static float tf_w[5] = {0};

/* =======================
   SS (2 estados)
   ======================= */
static float ss_A[4] = {0};   /* [A11 A12; A21 A22] en orden fila */
static float ss_B[2] = {0};   /* [B1; B2] */
static float ss_C[2] = {0};   /* [C1 C2] */
static float ss_D    = 0.0f;
static float ss_L[2] = {0};   /* [L1; L2] */
static float ss_K[2] = {0};   /* [K1 K2] */
static float ss_Kx   = 0.0f;  /* Kr (NOI) o Ki (I) */

/* Observador:
   xhat = estado corregido (posterior)
   zhat = predicción (prior) para ACT */
static float xhat[2] = {0,0};
static float zhat[2] = {0,0};

/* =======================
   Logging simple
   ======================= */
static float    log_buf[CONTROL_LOG_LEN * CONTROL_LOG_FIELDS];
static uint16_t log_idx = 0;
static uint8_t  log_ready_flag = 0;

void control_log_reset(void)
{
    log_idx = 0u;
    log_ready_flag = 0u;
}

uint8_t control_log_ready(void)
{
    return log_ready_flag;
}

const uint8_t* control_log_bytes(uint16_t* nbytes)
{
    if (nbytes) {
        *nbytes = (uint16_t)(log_idx * CONTROL_LOG_FIELDS * sizeof(float));
    }
    return (const uint8_t*)log_buf;
}

static inline void log_push(float r, float y, float u_phy)
{
    if (log_ready_flag) return;
    if (log_idx >= CONTROL_LOG_LEN) { log_ready_flag = 1u; return; }

    uint16_t base = (uint16_t)(log_idx * CONTROL_LOG_FIELDS);
    if (CONTROL_LOG_FIELDS >= 1u) log_buf[base + 0u] = r;
    if (CONTROL_LOG_FIELDS >= 2u) log_buf[base + 1u] = y;
    if (CONTROL_LOG_FIELDS >= 3u) log_buf[base + 2u] = u_phy;

    log_idx++;
    if (log_idx >= CONTROL_LOG_LEN) log_ready_flag = 1u;
}

/* =======================
   Helpers (CMSIS)
   ======================= */
static inline float dot2_cmsis(const float a[2], const float b[2])
{
    float out = 0.0f;
    arm_dot_prod_f32((float32_t*)a, (float32_t*)b, 2u, (float32_t*)&out);
    return out;
}

static inline float satf(float u, float umin, float umax)
{
    if (u > umax) return umax;
    if (u < umin) return umin;
    return u;
}

/* =======================
   Modo SS: Ki/Kr efectivos
   ======================= */
static inline uint8 ss_mode_has_integrator(uint8 impl)
{
    return (impl == UARTP_IMPL_SS_PRED_I) || (impl == UARTP_IMPL_SS_ACT_I);
}

static inline float ss_Kr_eff(void)
{
    return ss_mode_has_integrator(UARTP_Impl) ? 0.0f : ss_Kx;
}

static inline float ss_Ki_eff(void)
{
    return ss_mode_has_integrator(UARTP_Impl) ? ss_Kx : 0.0f;
}

/* back-calc (para bumpless) */
static inline float ucmd_from_uphy(float u_phy)
{
    return (u_phy - CONTROL_U_OFFSET);
}

/* =======================
   write_u (único actuador)
   ======================= */
static inline void write_u(float u_cmd)
{
    g_u_cmd = u_cmd;

    float u_phy = u_cmd + CONTROL_U_OFFSET;
    u_phy = satf(u_phy, CONTROL_SAT_MIN, CONTROL_SAT_MAX);

    g_u_out = u_phy;

    if (s_write_u_cb) {
        s_write_u_cb(u_phy);
    }
}

/* =======================
   SS helpers correctos
   ======================= */
static inline float ss_yhat(const float x[2], float u)
{
    return dot2_cmsis(ss_C, x) + ss_D * u;
}

static inline void ss_predict(const float x[2], float u, float z[2])
{
    /* evitar update in-place */
    float x0 = x[0];
    float x1 = x[1];

    z[0] = ss_A[0]*x0 + ss_A[1]*x1 + ss_B[0]*u;
    z[1] = ss_A[2]*x0 + ss_A[3]*x1 + ss_B[1]*u;
}

/* Predictor: xhat(k+1) = A xhat(k) + B u(k) + L*( y(k) - yhat(xhat(k), u_prev) ) */
static inline void ss_observer_pred_step(float y, float u_prev, float u_k)
{
    float innov = y - ss_yhat(xhat, u_prev);

    float z[2];
    ss_predict(xhat, u_k, z);

    xhat[0] = z[0] + ss_L[0]*innov;
    xhat[1] = z[1] + ss_L[1]*innov;
}

/* Actual (Current):
   xhat(k) = zhat(k) + L*( y(k) - yhat(zhat(k), u_prev) )
   zhat(k+1) = A xhat(k) + B u(k)
*/
static inline void ss_observer_act_correct(float y, float u_prev)
{
    float innov = y - ss_yhat(zhat, u_prev);

    xhat[0] = zhat[0] + ss_L[0]*innov;
    xhat[1] = zhat[1] + ss_L[1]*innov;
}

static inline void ss_observer_act_predict(float u_k)
{
    ss_predict(xhat, u_k, zhat);
}

/* =======================
   Registro IO / ISR glue
   ======================= */
void control_register_io(control_sample_isr_fn_t sample_isr,
                         control_write_u_fn_t    write_u_fn)
{
    s_sample_isr_cb = sample_isr;
    s_write_u_cb    = write_u_fn;
}

void control_on_sample_isr(void)
{
    if (s_sample_isr_cb) {
        s_sample_isr_cb(); /* debe llamar control_sample_isr_push(y) */
    } else {
        control_sample_pending = 1u;
    }
}

void control_sample_isr_push(float y)
{
    control_last_y = y;
    control_sample_pending = 1u;
}

/* =======================
   Referencia y Ts
   ======================= */
void control_set_reference(float r)
{
    uint8 intr = CyEnterCriticalSection();
    g_ref = r;
    control_ref_v = r;
    CyExitCriticalSection(intr);
}

float control_get_reference(void)
{
    return g_ref;
}

void control_set_sample_time(float Ts)
{
    if (Ts <= 0.0f) Ts = 1.0f;
    g_Ts = Ts;
}

float control_get_sample_time(void)
{
    return g_Ts;
}

/* =======================
   Start / Stop suave
   ======================= */
void control_start(float ref0)
{
    control_set_reference(ref0);

    /* arrancar desde el valor actual del DAC (bumpless) */
    float u0_cmd = ucmd_from_uphy(g_u_out);

    /* reset estados */
    xhat[0] = 0.0f; xhat[1] = 0.0f;
    zhat[0] = 0.0f; zhat[1] = 0.0f;
    memset(tf_w, 0, sizeof(tf_w));
    control_log_reset();

    /* bumpless para integrador */
    g_vint = 0.0f;
    if (UARTP_Impl != UARTP_IMPL_TF) {
        float Ki = ss_Ki_eff();
        if (Ki != 0.0f) {
            g_vint = u0_cmd / Ki;
        }
    }

    /* aplicar salida inicial */
    write_u(u0_cmd);

    /* IMPORTANTÍSIMO para ACT:
       el primer sample y(k) llega con u_prev = g_u_out ya aplicado,
       entonces zhat debe representar la predicción asociada a ese u_prev. */
    if (UARTP_Impl != UARTP_IMPL_TF) {
        ss_predict(xhat, g_u_out, zhat); /* xhat=0 => zhat = B*u0 */
    }
}

bool control_stop_suave_step(void)
{
    float u = g_u_cmd;

    if (u < CONTROL_STOP_TARGET) {
        u += CONTROL_STOP_STEP;
        if (u > CONTROL_STOP_TARGET) u = CONTROL_STOP_TARGET;
        write_u(u);
        return false;
    }
    if (u > CONTROL_STOP_TARGET) {
        u -= CONTROL_STOP_STEP;
        if (u < CONTROL_STOP_TARGET) u = CONTROL_STOP_TARGET;
        write_u(u);
        return false;
    }
    return true;
}

/* =======================
   Carga de coeficientes
   ======================= */
void control_apply_tf(const float* c, uint16_t n)
{
    if (!c || n < 16u) return;

    for (uint8_t i = 0; i < 6u; i++) {
        tf_b[i] = c[i];
        tf_a[i] = c[i + 6u];
    }

    /* normalizar por a0 */
    if (tf_a[0] != 0.0f && tf_a[0] != 1.0f) {
        float a0 = tf_a[0];
        for (uint8_t i = 0; i < 6u; i++) tf_b[i] /= a0;
        for (uint8_t i = 1; i < 6u; i++) tf_a[i] /= a0;
        tf_a[0] = 1.0f;
    }

    memset(tf_w, 0, sizeof(tf_w));
}

void control_apply_ss(const float* c, uint16_t n)
{
    if (!c || n < 16u) return;

    ss_A[0] = c[0]; ss_A[1] = c[1];
    ss_A[2] = c[2]; ss_A[3] = c[3];

    ss_B[0] = c[4]; ss_B[1] = c[5];

    ss_C[0] = c[6]; ss_C[1] = c[7];

    ss_D    = c[8];

    ss_L[0] = c[9];  ss_L[1] = c[10];
    ss_K[0] = c[11]; ss_K[1] = c[12];

    ss_Kx   = c[13];

    xhat[0] = 0.0f; xhat[1] = 0.0f;
    zhat[0] = 0.0f; zhat[1] = 0.0f;
    g_vint  = 0.0f;
}

/* =======================
   TF step (DF2T)
   ======================= */
static inline float tf_step(float x)
{
    float y = tf_b[0] * x + tf_w[0];

    tf_w[0] = tf_w[1] + tf_b[1]*x - tf_a[1]*y;
    tf_w[1] = tf_w[2] + tf_b[2]*x - tf_a[2]*y;
    tf_w[2] = tf_w[3] + tf_b[3]*x - tf_a[3]*y;
    tf_w[3] = tf_w[4] + tf_b[4]*x - tf_a[4]*y;
    tf_w[4] =           tf_b[5]*x - tf_a[5]*y;

    return y;
}

/* =======================
   CONTROL STEP (main loop)
   ======================= */
void control_step(void)
{
    /* snapshot (y, r, u_prev) */
    uint8 intr = CyEnterCriticalSection();
    float y = control_last_y;
    control_sample_pending = 0u;
    float r = g_ref;
    float u_prev = g_u_out;   /* u aplicado durante este sample */
    CyExitCriticalSection(intr);

    switch (UARTP_Impl)
    {
        case UARTP_IMPL_TF:
        {
            float e = r - y;
            float u = tf_step(e);
            write_u(u);
            log_push(r, y, g_u_out);
        } break;

        /* ===== SS Predictor (NOI e I) ===== */
        case UARTP_IMPL_SS_PRED_NOI:
        case UARTP_IMPL_SS_PRED_I:
        {
            uint8 has_i = ss_mode_has_integrator(UARTP_Impl);

            if (has_i) {
                float e = r - y;
                g_vint += e * g_Ts;
            }

            /* u(k) con xhat(k) */
            float u_cmd = ss_Kr_eff()*r + ss_Ki_eff()*g_vint - dot2_cmsis(ss_K, xhat);
            write_u(u_cmd);
            float u_k = g_u_out;

            /* xhat(k+1) */
            ss_observer_pred_step(y, u_prev, u_k);

            log_push(r, y, g_u_out);
        } break;

        /* ===== SS Actual/Current (NOI e I) ===== */
        case UARTP_IMPL_SS_ACT_NOI:
        case UARTP_IMPL_SS_ACT_I:
        default:
        {
            uint8 has_i = ss_mode_has_integrator(UARTP_Impl);

            /* 1) corregir xhat(k) desde zhat(k) */
            ss_observer_act_correct(y, u_prev);

            if (has_i) {
                float e = r - y;
                g_vint += e * g_Ts;
            }

            /* 2) u(k) con xhat(k) corregido */
            float u_cmd = ss_Kr_eff()*r + ss_Ki_eff()*g_vint - dot2_cmsis(ss_K, xhat);
            write_u(u_cmd);
            float u_k = g_u_out;

            /* 3) zhat(k+1) */
            ss_observer_act_predict(u_k);

            log_push(r, y, g_u_out);
        } break;
    }
}
