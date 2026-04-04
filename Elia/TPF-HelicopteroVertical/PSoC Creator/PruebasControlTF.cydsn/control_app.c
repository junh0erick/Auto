#include "control_app.h"

#include "project.h"
#include "uartp_sw.h"     /* UARTP_Impl */
#include "arm_math.h"     /* CMSIS-DSP */
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
static float g_ref = 0.0f;    /* referencia REAL (desde UART) */
static float g_Ts  = 1.0f;    /* sample time para integrador */

/* u comando (salida del control, antes de offset) */
static float g_u_cmd = 0.0f;

/* u físico (después de offset + saturación): lo que va al actuador */
static float g_u_out = 0.0f;

/* integrador (servo) */
static float g_vint = 0.0f;

/* =======================
   TF (IIR orden 5, DF2T)
   Layout UART:
     c[0..5]  = b0..b5
     c[6..11] = a0..a5
     c[12..15]= reservados
   ======================= */
static float tf_b[6] = {0};
static float tf_a[6] = {0};
static float tf_w[5] = {0}; /* estados DF2T */

/* =======================
   SS (2 estados)
   Layout UART (16 floats):
     0..3   A11 A12 A21 A22
     4..5   B1  B2
     6..7   C1  C2
     8      D
     9..10  L1  L2
     11..12 K1  K2
     13     Ki
     14     Kr   (si 0 => se asume 1)
     15     reservado
   ======================= */
static float ss_A[4] = {0};
static float ss_B[2] = {0};
static float ss_C[2] = {0};
static float ss_D    = 0.0f;
static float ss_L[2] = {0};
static float ss_K[2] = {0};
static float ss_Ki   = 0.0f;
static float ss_Kr   = 1.0f;

static float xhat[2] = {0,0};

/* =======================
   Logging simple
   ======================= */
static float    log_buf[CONTROL_LOG_LEN * CONTROL_LOG_FIELDS];
static uint16_t log_idx = 0;
static uint8_t  log_ready_flag = 0;

void control_log_reset(void)
{
    log_idx = 0;
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

static inline void write_u(float u_cmd)
{
    /* guardo comando interno */
    g_u_cmd = u_cmd;

    /* mapping físico opcional */
    float u_phy = u_cmd + CONTROL_U_OFFSET;
    u_phy = satf(u_phy, CONTROL_SAT_MIN, CONTROL_SAT_MAX);

    g_u_out = u_phy;

    if (s_write_u_cb) {
        s_write_u_cb(u_phy);
    }
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

/* Llamar en tu ISR real (isr_sampling) */
void control_on_sample_isr(void)
{
    if (s_sample_isr_cb) {
        /* tu callback debe terminar llamando control_sample_isr_push(y) */
        s_sample_isr_cb();
    } else {
        /* fallback: solo levanta bandera */
        control_sample_pending = 1u;
    }
}

/* Helper: ISR setea las 2 variables pedidas */
void control_sample_isr_push(float y)
{
    control_last_y = y;
    control_sample_pending = 1u;
}

/* =======================
   Referencia y sample time
   ======================= */
void control_set_reference(float r)
{
    uint8_t intr = CyEnterCriticalSection();
    g_ref = r;
    control_ref_v = r; /* espejo opcional */
    CyExitCriticalSection(intr);
}

float control_get_reference(void)
{
    /* lectura simple (float 32-bit alineado es atómica en CM3) */
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
    /* ref0 VIENE DESDE UART ('i') */
    control_set_reference(ref0);

    g_vint = 0.0f;
    xhat[0] = 0.0f; xhat[1] = 0.0f;

    memset(tf_w, 0, sizeof(tf_w));
    control_log_reset();

    /* salida inicial razonable */
    float u0 = 0.0f;
    if (UARTP_Impl != UARTP_IMPL_TF) {
        float r = control_get_reference();
        u0 = ss_Kr * r;   /* xhat=0 */
    }
    write_u(u0);
}

bool control_stop_suave_step(void)
{
    float u = g_u_cmd; /* ramp sobre comando, no sobre físico */

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

    ss_D = c[8];

    ss_L[0] = c[9];  ss_L[1]  = c[10];
    ss_K[0] = c[11]; ss_K[1]  = c[12];

    ss_Ki = c[13];
    ss_Kr = (c[14] == 0.0f) ? 1.0f : c[14];

    xhat[0] = 0.0f; xhat[1] = 0.0f;
    g_vint = 0.0f;
}

/* =======================
   TF step (DF2T)
   y = H(z)*x, orden 5
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
   SS helpers (2 estados)
   ======================= */
static inline void ss_predict(float u_phy, float z[2])
{
    z[0] = ss_A[0]*xhat[0] + ss_A[1]*xhat[1] + ss_B[0]*u_phy;
    z[1] = ss_A[2]*xhat[0] + ss_A[3]*xhat[1] + ss_B[1]*u_phy;
}

static inline float ss_yhat_from_xu(const float x[2], float u_phy)
{
    float cx = dot2_cmsis(ss_C, x);
    return cx + ss_D*u_phy;
}

/* =======================
   CONTROL STEP (main loop)
   ======================= */
void control_step(void)
{
    /* snapshot coherente de (y, r) y limpiamos bandera */
    uint8_t intr = CyEnterCriticalSection();
    float y = control_last_y;
    control_sample_pending = 0u;
    float r = g_ref;          /* referencia REAL: viene de UART via control_start() */
    CyExitCriticalSection(intr);

    switch (UARTP_Impl)
    {
        /* ================= TF ================= */
        case UARTP_IMPL_TF:
        {
            float e = r - y;
            float u = tf_step(e);
            write_u(u);
            log_push(r, y, g_u_out);
        } break;

        /* ===== SS predictor, sin integrador ===== */
        case UARTP_IMPL_SS_PRED_NOI:
        {
            float u = ss_Kr*r - dot2_cmsis(ss_K, xhat);
            write_u(u);

            float z[2];
            ss_predict(g_u_out, z);

            float yhat = ss_yhat_from_xu(z, g_u_out);
            float innov = y - yhat;

            xhat[0] = z[0] + ss_L[0]*innov;
            xhat[1] = z[1] + ss_L[1]*innov;

            log_push(r, y, g_u_out);
        } break;

        /* ===== SS actual/current, sin integrador ===== */
        case UARTP_IMPL_SS_ACT_NOI:
        {
            float yhat = ss_yhat_from_xu(xhat, g_u_out);
            float innov = y - yhat;

            xhat[0] = xhat[0] + ss_L[0]*innov;
            xhat[1] = xhat[1] + ss_L[1]*innov;

            float u = ss_Kr*r - dot2_cmsis(ss_K, xhat);
            write_u(u);

            float z[2];
            ss_predict(g_u_out, z);
            xhat[0] = z[0];
            xhat[1] = z[1];

            log_push(r, y, g_u_out);
        } break;

        /* ===== SS predictor, con integrador ===== */
        case UARTP_IMPL_SS_PRED_I:
        {
            float e = r - y;
            g_vint += e * g_Ts;

            float u = ss_Kr*r + ss_Ki*g_vint - dot2_cmsis(ss_K, xhat);
            write_u(u);

            float z[2];
            ss_predict(g_u_out, z);

            float yhat = ss_yhat_from_xu(z, g_u_out);
            float innov = y - yhat;

            xhat[0] = z[0] + ss_L[0]*innov;
            xhat[1] = z[1] + ss_L[1]*innov;

            log_push(r, y, g_u_out);
        } break;

        /* ===== SS actual/current, con integrador ===== */
        case UARTP_IMPL_SS_ACT_I:
        default:
        {
            float yhat = ss_yhat_from_xu(xhat, g_u_out);
            float innov = y - yhat;

            xhat[0] = xhat[0] + ss_L[0]*innov;
            xhat[1] = xhat[1] + ss_L[1]*innov;

            float e = r - y;
            g_vint += e * g_Ts;

            float u = ss_Kr*r + ss_Ki*g_vint - dot2_cmsis(ss_K, xhat);
            write_u(u);

            float z[2];
            ss_predict(g_u_out, z);
            xhat[0] = z[0];
            xhat[1] = z[1];

            log_push(r, y, g_u_out);
        } break;
    }
}
