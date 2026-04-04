/* =========================
   control_app.c
   ========================= */
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

/* espejo opcional (debug) */
volatile float control_ref_v = 0.0f;

/* =======================
   IO callbacks
   ======================= */
static control_sample_isr_fn_t s_sample_isr_cb = 0;
static control_write_u_fn_t    s_write_u_cb    = 0;

/* =======================
   Estado general
   ======================= */
static float g_ref = 0.0f;
static float g_Ts  = 1.0f;

/* u_cmd: interno (antes de offset). En este proyecto es igual a PWM us. */
static float g_u_cmd = 0.0f;
static float g_u_out = 0.0f;  /* físico (post offset + sat) */

/* integrador */
static float g_vint  = 0.0f;

/* =======================
   TF (IIR orden 5, DF2T)
   ======================= */
static float tf_b[6] = {0};
static float tf_a[6] = {0};
static float tf_w[5] = {0};

/* =======================
   SS (2 estados)
   ======================= */
static float ss_A[4] = {0};   /* [A11 A12; A21 A22] fila-major */
static float ss_B[2] = {0};   /* [B1; B2] */
static float ss_C[2] = {0};   /* [C1 C2] */
static float ss_D    = 0.0f;
static float ss_L[2] = {0};   /* [L1; L2] */
static float ss_K[2] = {0};   /* [K1 K2] */
static float ss_Kx   = 0.0f;  /* Kr (NOI) o Ki (I) */

static float xhat[2] = {0.0f, 0.0f};  /* posterior */
static float zhat[2] = {0.0f, 0.0f};  /* prior (para ACT) */

/* =======================
   Helpers
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

static inline float ucmd_from_uphy(float u_phy)
{
    return (u_phy - CONTROL_U_OFFSET);
}

/* --- clamp de referencia según modo (lo que querías) --- */
static inline float clamp_ref(float r)
{
    if (UARTP_Impl == UARTP_IMPL_OPENLOOP)
    {
        /* Open-loop: ref = PWM directo (us) */
        if (r < CONTROL_OL_REF_MIN) return CONTROL_OL_REF_MIN;
        if (r > CONTROL_OL_REF_MAX) return CONTROL_OL_REF_MAX;
        return r;
    }
    else
    {
        /* Closed-loop: ref = altura (cm) */
        if (r < CONTROL_CL_REF_MIN) return CONTROL_CL_REF_MIN;
        if (r > CONTROL_CL_REF_MAX) return CONTROL_CL_REF_MAX;
        return r;
    }
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

    if (s_write_u_cb) s_write_u_cb(u_phy);
}

void control_force_min(void)
{
    write_u( ucmd_from_uphy(CONTROL_SAT_MIN) );
}

/* =======================
   SS: yhat / predict
   ======================= */
static inline float ss_yhat_from_xu(const float x[2], float u)
{
    return dot2_cmsis(ss_C, x) + ss_D * u;
}

static inline void ss_predict_from_xu(const float x[2], float u, float z[2])
{
    arm_dot_prod_f32((float32_t*)&ss_A[0], (float32_t*)x, 2u, (float32_t*)&z[0]);
    z[0] += ss_B[0] * u;

    arm_dot_prod_f32((float32_t*)&ss_A[2], (float32_t*)x, 2u, (float32_t*)&z[1]);
    z[1] += ss_B[1] * u;
}

/* =======================
   Observadores
   ======================= */
static inline void ss_observer_pred_step(float y, float u_prev, float u_k)
{
    const float innov = y - ss_yhat_from_xu(xhat, u_prev);

    float z[2];
    ss_predict_from_xu(xhat, u_k, z);

    float Linv[2];
    arm_scale_f32((float32_t*)ss_L, innov, (float32_t*)Linv, 2u);

    arm_add_f32((float32_t*)z, (float32_t*)Linv, (float32_t*)xhat, 2u);
}

static inline void ss_observer_act_correct(float y, float u_prev)
{
    const float innov = y - ss_yhat_from_xu(zhat, u_prev);

    float Linv[2];
    arm_scale_f32((float32_t*)ss_L, innov, (float32_t*)Linv, 2u);

    arm_add_f32((float32_t*)zhat, (float32_t*)Linv, (float32_t*)xhat, 2u);
}

static inline void ss_observer_act_predict(float u_k)
{
    ss_predict_from_xu(xhat, u_k, zhat);
}

/* =======================
   TF DF2T step
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
   IO / ISR glue
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
        s_sample_isr_cb();
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

    /* bumpless: arrancar desde el u físico actual */
    float u0_cmd = ucmd_from_uphy(g_u_out);

    xhat[0] = 0.0f; xhat[1] = 0.0f;
    zhat[0] = 0.0f; zhat[1] = 0.0f;
    memset(tf_w, 0, sizeof(tf_w));

    g_vint = 0.0f;

    /* Si estás en SS con integrador, inicializá vint para que u0 salga estable */
    if (UARTP_Impl != UARTP_IMPL_TF && UARTP_Impl != UARTP_IMPL_OPENLOOP) {
        float Ki = ss_Ki_eff();
        if (Ki != 0.0f) g_vint = u0_cmd / Ki;
    }

    write_u(u0_cmd);

    if (UARTP_Impl != UARTP_IMPL_TF && UARTP_Impl != UARTP_IMPL_OPENLOOP) {
        ss_predict_from_xu(xhat, g_u_out, zhat);
    }
}
//
//bool control_stop_suave_step(void)
//{
//    const float target = CONTROL_STOP_TARGET;
//
//    float tol = CONTROL_STOP_TOL_ABS;
//    float rel = CONTROL_STOP_TOL_REL * ((target >= 0.0f) ? target : -target);
//    if (rel > tol) tol = rel;
//
//    for (uint32_t i = 0; i < CONTROL_STOP_MAX_ITERS; i++)
//    {
//        float u = g_u_cmd;
//
//        float err = target - u;
//        if (err < 0.0f) err = -err;
//
//        if (err <= tol)
//        {
//            write_u(target);
//            return true;
//        }
//
//        if (u < target) {
//            u += CONTROL_STOP_STEP;
//            if (u > target) u = target;
//        } else {
//            u -= CONTROL_STOP_STEP;
//            if (u < target) u = target;
//        }
//
//        write_u(u);
//        CyDelayUs(CONTROL_STOP_DELAY_US);
//    }
//
//    write_u(target);
//    return true;
//}
bool control_stop_suave_step(void)
{
    const float u_target = CONTROL_DESC_TARGET_US;

    /* 1) Rampa hasta u_target */
    for (uint32_t i = 0u; i < (uint32_t)CONTROL_DESC_MAX_ITERS; i++)
    {
        float u = g_u_cmd; /* u_cmd interno */

        if (u < u_target) {
            u += CONTROL_DESC_STEP_US;
            if (u > u_target) u = u_target;
            write_u(u);
        } else if (u > u_target) {
            u -= CONTROL_DESC_STEP_US;
            if (u < u_target) u = u_target;
            write_u(u);
        } else {
            /* ya llegó */
            break;
        }

        CyDelayUs(CONTROL_DESC_DELAY_US);
    }

    /* 2) Mantener u_target por 5s */
    write_u(u_target);

    uint32_t hold_loops = (CONTROL_DESC_HOLD_MS * 1000u) / CONTROL_DESC_HOLD_POLL_US;
    if (hold_loops < 1u) hold_loops = 1u;

    for (uint32_t k = 0u; k < hold_loops; k++) {
        /* si querés, re-escribimos por seguridad (por si otro código lo pisa) */
        write_u(u_target);
        CyDelayUs(CONTROL_DESC_HOLD_POLL_US);
    }

    /* 3) Cortar motor y terminar */
    write_u(ucmd_from_uphy((float)CONTROL_DESC_CUTOFF_US));
    return true;
}


/* =======================
   Carga coeficientes
   ======================= */
void control_apply_tf(const float* c, uint16_t n)
{
    if (!c || n < 16u) return;

    for (uint8_t i = 0; i < 6u; i++) {
        tf_b[i] = c[i];
        tf_a[i] = c[i + 6u];
    }

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
   CONTROL STEP (main)
   ======================= */
void control_step(void)
{
    uint8 intr = CyEnterCriticalSection();
    float y = control_last_y;
    control_sample_pending = 0u;

    /* r se interpreta distinto según el modo */
    float r = clamp_ref(g_ref);

    float u_prev = g_u_out;
    CyExitCriticalSection(intr);

    switch (UARTP_Impl)
    {
        /* ===== OPEN LOOP: u_phy = ref (PWM directo) ===== */
        case UARTP_IMPL_OPENLOOP:
        {
            /* r ya está clamp a [1000..2000] */
            float u_cmd = ucmd_from_uphy(r);
            write_u(u_cmd);
        } break;

        /* ===== TF: e = (cm) ===== */
        case UARTP_IMPL_TF:
        {
            float e = r - y;             /* cm */
            float u_cmd = tf_step(e);    /* debe dar PWM(us) */
            write_u(u_cmd);
        } break;

        /* ===== SS Predictor (NOI e I) ===== */
        case UARTP_IMPL_SS_PRED_NOI:
        case UARTP_IMPL_SS_PRED_I:
        {
            uint8 has_i = ss_mode_has_integrator(UARTP_Impl);

            if (has_i) {
                float e = r - y;   /* cm */
                g_vint += e;       /* como querés: sin Ts (Ts=1 “normalizado”) */
            }

            float u_cmd = ss_Kr_eff()*r + ss_Ki_eff()*g_vint - dot2_cmsis(ss_K, xhat);
            write_u(u_cmd);
            float u_k = g_u_out;

            ss_observer_pred_step(y, u_prev, u_k);
        } break;

        /* ===== SS Actual/Current (NOI e I) ===== */
        case UARTP_IMPL_SS_ACT_NOI:
        case UARTP_IMPL_SS_ACT_I:
        default:
        {
            uint8 has_i = ss_mode_has_integrator(UARTP_Impl);

            ss_observer_act_correct(y, u_prev);

            if (has_i) {
                float e = r - y;   /* cm */
                g_vint += e;       /* sin Ts */
            }

            float u_cmd = ss_Kr_eff()*r + ss_Ki_eff()*g_vint - dot2_cmsis(ss_K, xhat);
            write_u(u_cmd);
            float u_k = g_u_out;

            ss_observer_act_predict(u_k);
        } break;
    }

    UARTP_Telemetry_Push(g_u_out, y);
}
