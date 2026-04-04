/* =========================
   control_app.c
   (PAQUETE FIJO 25 floats)
   =========================
   Meta SIEMPRE al final:
     c[23] = N
     c[24] = FsHz
   TF además usa:
     c[22] = order (0..10)
*/

#include "control_app.h"

#include "project.h"
#include "uartp_sw.h"
#include "arm_math.h"
#include <string.h>

/* =======================
   Defaults de estabilización post-calib (si no están en .h)
   ======================= */
#ifndef CONTROL_U0_SETTLE_SAMPLES
    #define CONTROL_U0_SETTLE_SAMPLES (50u)
#endif
#ifndef CONTROL_U0_SETTLE_DY_CM
    #define CONTROL_U0_SETTLE_DY_CM   (0.5f)
#endif

/* =======================
   Variables ISR -> main
   ======================= */
volatile float   control_last_y = 0.0f;
volatile uint8_t control_sample_pending = 0u;
volatile float   control_ref_v = 0.0f;

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

/* Interno: Δu (us) */
static float g_u_cmd = 0.0f;  /* comando interno (Δu) */
static float g_u_out = 0.0f;  /* físico (PWM us) post offset+sat */

/* integrador */
static float g_vint  = 0.0f;

/* Offset runtime (u0). Se calcula en calibración o cae al default */
static float g_u0_offset_us = CONTROL_U_OFFSET;

float control_get_u0_offset_us(void)
{
    return g_u0_offset_us;
}

/* =======================
   Sesión de control
   ======================= */
static uint8_t g_session_active = 0u; /* 0=parado, 1=en sesión */
static uint8_t g_u0_valid       = 0u; /* 0=pendiente calibrar, 1=u0 fijo en esta sesión */
static uint8_t g_first_start = 1u;

/* =======================
   TF (IIR DF2T) hasta orden 10
   ======================= */
static float tf_b[CONTROL_TF_MAX_ORDER + 1u] = {0}; /* b0..b10 */
static float tf_a[CONTROL_TF_MAX_ORDER + 1u] = {0}; /* a0..a10 */
static float tf_w[CONTROL_TF_MAX_ORDER]      = {0}; /* w0..w9 (usa solo [0..order-1]) */
static uint8  tf_order = 0u;                         /* orden efectivo 0..10 */

/* =======================
   SS (3 estados)
   ======================= */
static float ss_A[9] = {0};   /* fila-major 3x3 */
static float ss_B[3] = {0};   /* 3x1 */
static float ss_C[3] = {0};   /* 1x3 */
static float ss_D    = 0.0f;
static float ss_L[3] = {0};   /* 3x1 */
static float ss_K[3] = {0};   /* 1x3 */
static float ss_Kx   = 0.0f;  /* Ki (modo I) o Kr (modo NOI) */

static float xhat[3] = {0.0f, 0.0f, 0.0f};  /* posterior */
static float zhat[3] = {0.0f, 0.0f, 0.0f};  /* prior (para ACT) */

/* =======================
   Auto-calibración u0 (estado)
   ======================= */
#if CONTROL_ENABLE_AUTOCAL
typedef enum {
    CALIB_IDLE = 0,
    CALIB_RAMP,
    CALIB_POSTMOVE,
    CALIB_SETTLE,
    CALIB_DONE,
    CALIB_FAIL
} calib_state_t;

static calib_state_t g_calib_state   = CALIB_IDLE;

/* Calibración por niveles */
static float    g_calib_u_phy         = CONTROL_CALIB_START_US;
static float    g_calib_y_level0      = 0.0f;
static uint16_t g_calib_hits          = 0u;
static uint16_t g_calib_level_samples = 0u;
static uint32_t g_calib_total_samples = 0u;
static uint16_t g_postmove_cnt   = 0u;
static uint16_t g_postmove_need  = 0u;
static float    g_postmove_u_phy = 0.0f;


/* Estabilización post-calib */
static float    g_settle_y_prev       = 0.0f;
static uint16_t g_settle_stable_cnt   = 0u;
static uint8_t  g_settle_first        = 1u;
#endif


static volatile uint8_t g_stop_busy      = 0u;
static volatile uint8_t g_start_queued   = 0u;
static volatile float   g_start_ref_q    = 0.0f;

/* =======================
   Helpers
   ======================= */

static inline uint16_t samples_from_ms(uint16_t ms)
{
    float Ts = g_Ts;
    if (!(Ts > 0.0f)) Ts = 1.0f;

    float fs = 1.0f / Ts;
    float n  = fs * ((float)ms / 1000.0f);

    if (n < 1.0f) n = 1.0f;
    if (n > 65535.0f) n = 65535.0f;

    return (uint16_t)(n + 0.5f);
}


static inline uint16_t u16_from_float(float x)
{
    if (x <= 0.0f) return 0u;
    if (x >= 65535.0f) return 65535u;
    return (uint16_t)(x + 0.5f);
}

static inline uint8 tf_order_from_float(float x)
{
    int32_t o = (int32_t)(x + 0.5f);
    if (o < 0) o = 0;
    if (o > (int32_t)CONTROL_TF_MAX_ORDER) o = (int32_t)CONTROL_TF_MAX_ORDER;
    return (uint8)o;
}

static inline float dot3_cmsis(const float a[3], const float b[3])
{
    float out = 0.0f;
    arm_dot_prod_f32((float32_t*)a, (float32_t*)b, 3u, (float32_t*)&out);
    return out;
}

static inline float ss_yhat_from_xu(const float x[3], float u_cmd)
{
    return dot3_cmsis(ss_C, x) + ss_D * u_cmd;
}

static inline void ss_predict_from_xu(const float x[3], float u_cmd, float z[3])
{
    arm_dot_prod_f32((float32_t*)&ss_A[0], (float32_t*)x, 3u, (float32_t*)&z[0]);
    z[0] += ss_B[0] * u_cmd;

    arm_dot_prod_f32((float32_t*)&ss_A[3], (float32_t*)x, 3u, (float32_t*)&z[1]);
    z[1] += ss_B[1] * u_cmd;

    arm_dot_prod_f32((float32_t*)&ss_A[6], (float32_t*)x, 3u, (float32_t*)&z[2]);
    z[2] += ss_B[2] * u_cmd;
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
    return ss_mode_has_integrator((uint8)UARTP_Impl) ? 0.0f : ss_Kx;
}

static inline float ss_Ki_eff(void)
{
    return ss_mode_has_integrator((uint8)UARTP_Impl) ? ss_Kx : 0.0f;
}

/* Conversión: PWM físico (us) -> Δu (us) */
static inline float ucmd_from_uphy(float u_phy)
{
    return (u_phy - g_u0_offset_us);
}

/* Conversión: Δu (us) -> PWM físico (us) */
static inline float uphy_from_ucmd(float u_cmd)
{
    return (u_cmd + g_u0_offset_us);
}

/* Rango Δu permitido (dinámico) */
static inline float cmd_min_dyn(void) { return (CONTROL_SAT_MIN - g_u0_offset_us); }
static inline float cmd_max_dyn(void) { return (CONTROL_SAT_MAX - g_u0_offset_us); }

static inline float y_rel_from_y(float y_phys) { return (y_phys - (float)CONTROL_DESC_MIN_Y_CM); }
static inline float r_rel_from_r(float r_phys) { return (r_phys - (float)CONTROL_DESC_MIN_Y_CM); }

/* --- clamp de referencia según modo --- */
static inline float clamp_ref(float r)
{
    if (UARTP_Impl == UARTP_IMPL_OPENLOOP)
    {
#if CONTROL_OL_REF_IS_DELTA
        return satf(r, cmd_min_dyn(), cmd_max_dyn());
#else
        if (r < CONTROL_OL_REF_MIN) return CONTROL_OL_REF_MIN;
        if (r > CONTROL_OL_REF_MAX) return CONTROL_OL_REF_MAX;
        return r;
#endif
    }
    else
    {
        if (r < CONTROL_CL_REF_MIN) return CONTROL_CL_REF_MIN;
        if (r > CONTROL_CL_REF_MAX) return CONTROL_CL_REF_MAX;
        return r;
    }
}

/* =======================
   write_u (único actuador)
   Entrada: Δu (us)
   ======================= */
static inline void write_u(float u_cmd)
{
    u_cmd = satf(u_cmd, cmd_min_dyn(), cmd_max_dyn());
    g_u_cmd = u_cmd;

    float u_phy = uphy_from_ucmd(u_cmd);
    u_phy = satf(u_phy, CONTROL_SAT_MIN, CONTROL_SAT_MAX);
    g_u_out = u_phy;

    if (s_write_u_cb) s_write_u_cb(u_phy);
}

void control_force_min(void)
{
    write_u(ucmd_from_uphy(CONTROL_SAT_MIN));
}

/* =======================
   Observadores
   ======================= */
static inline void ss_observer_pred_step(float y, float u_prev_cmd, float u_k_cmd)
{
    const float innov = y - ss_yhat_from_xu(xhat, u_prev_cmd);

    float z[3];
    ss_predict_from_xu(xhat, u_k_cmd, z);

    float Linv[3];
    arm_scale_f32((float32_t*)ss_L, innov, (float32_t*)Linv, 3u);

    arm_add_f32((float32_t*)z, (float32_t*)Linv, (float32_t*)xhat, 3u);
}

static inline void ss_observer_act_correct(float y, float u_prev_cmd)
{
    const float innov = y - ss_yhat_from_xu(zhat, u_prev_cmd);

    float Linv[3];
    arm_scale_f32((float32_t*)ss_L, innov, (float32_t*)Linv, 3u);

    arm_add_f32((float32_t*)zhat, (float32_t*)Linv, (float32_t*)xhat, 3u);
}

static inline void ss_observer_act_predict(float u_k_cmd)
{
    ss_predict_from_xu(xhat, u_k_cmd, zhat);
}

/* =======================
   TF DF2T step (usa SOLO el orden efectivo)
   ======================= */
static inline float tf_step(float x)
{
    const uint8 ord = tf_order;

    if (ord == 0u) {
        return tf_b[0] * x;
    }

    float y = tf_b[0] * x + tf_w[0];

    for (uint8 i = 0u; i < (uint8)(ord - 1u); i++) {
        tf_w[i] = tf_w[i + 1u] + tf_b[i + 1u] * x - tf_a[i + 1u] * y;
    }

    tf_w[ord - 1u] = tf_b[ord] * x - tf_a[ord] * y;

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
   START: 1x por sesión
   ======================= */
void control_start(float ref0)
{
    
     control_set_reference(ref0);

    /* ===== FIX RÁPIDO: limpiar estado en el primer START ===== */
    if (g_first_start)
    {
        g_first_start   = 0u;

        /* reset duro de estados que pueden quedar "a medio camino" */
        g_session_active = 0u;
        g_stop_busy      = 0u;
        g_start_queued   = 0u;

#if CONTROL_ENABLE_AUTOCAL
        g_calib_state = CALIB_IDLE;
#endif
        g_u0_valid = 0u;                 /* fuerza calibración */
        g_u0_offset_us = CONTROL_U_OFFSET;

        g_u_cmd = 0.0f;
        g_u_out = CONTROL_SAT_MIN;       /* o CONTROL_U_OFFSET si preferís */
        g_vint  = 0.0f;

        xhat[0]=0; xhat[1]=0; xhat[2]=0;
        zhat[0]=0; zhat[1]=0; zhat[2]=0;
        memset(tf_w, 0, sizeof(tf_w));
    }
    /* ======================================================== */

#if CONTROL_ENABLE_AUTOCAL
    if (g_stop_busy)
    {
        g_start_queued = 1u;
        g_start_ref_q  = ref0;
        return;
    }
#endif

    if (g_session_active) return;
    g_session_active = 1u;
   

    xhat[0] = 0.0f; xhat[1] = 0.0f; xhat[2] = 0.0f;
    zhat[0] = 0.0f; zhat[1] = 0.0f; zhat[2] = 0.0f;

    memset(tf_w, 0, sizeof(tf_w));
    g_vint = 0.0f;

#if CONTROL_ENABLE_AUTOCAL
    if (!g_u0_valid)
    {
        g_calib_state          = CALIB_RAMP;
        g_calib_u_phy          = CONTROL_CALIB_START_US;
        g_calib_hits           = 0u;
        g_calib_level_samples  = 0u;
        g_calib_total_samples  = 0u;
        g_calib_y_level0       = 0.0f;

        g_u0_offset_us = CONTROL_U_OFFSET;

        write_u(ucmd_from_uphy(g_calib_u_phy));
        return;
    }
#endif

    /* bumpless desde u actual */
    {
        float u0_cmd = ucmd_from_uphy(g_u_out);

        if (UARTP_Impl != UARTP_IMPL_TF && UARTP_Impl != UARTP_IMPL_OPENLOOP) {
            float Ki = ss_Ki_eff();
            if (Ki != 0.0f) g_vint = -u0_cmd / Ki;
        }

        write_u(u0_cmd);

        if (UARTP_Impl != UARTP_IMPL_TF && UARTP_Impl != UARTP_IMPL_OPENLOOP) {
            ss_predict_from_xu(xhat, u0_cmd, zhat);
        }
    }
}

/* =======================
   STOP suave: termina sesión
   (FIX: target es Δu directo, no PWM absoluto)
   ======================= */
bool control_stop_suave_step(void)
{
   g_stop_busy = 1u;

    /* TARGET es Δu relativo (cmd) */
    float u_target_cmd = (float)CONTROL_DESC_TARGET_CMD_US;
    u_target_cmd = satf(u_target_cmd, cmd_min_dyn(), cmd_max_dyn());

    for (uint32_t i = 0u; i < (uint32_t)CONTROL_DESC_MAX_ITERS; i++)
    {
        float u = g_u_cmd;

        if (u < u_target_cmd) {
            break;
        } else if (u > u_target_cmd) {
            u -= CONTROL_DESC_STEP_US;
            if (u < u_target_cmd) u = u_target_cmd;
            write_u(u);
        } else {
            break;
        }

        CyDelayUs(CONTROL_DESC_DELAY_US);
    }

    write_u(u_target_cmd);

    uint32_t hold_loops = (CONTROL_DESC_HOLD_MS * 1000u) / CONTROL_DESC_HOLD_POLL_US;
    if (hold_loops < 1u) hold_loops = 1u;

    for (uint32_t k = 0u; k < hold_loops; k++) {
        write_u(u_target_cmd);
        CyDelayUs(CONTROL_DESC_HOLD_POLL_US);
    }

    /* CUTOFF es PWM absoluto (us) -> convertir a Δu */
    write_u(ucmd_from_uphy((float)CONTROL_DESC_CUTOFF_US));

    g_session_active = 0u;
    g_u0_valid       = 0u;
    #if CONTROL_ENABLE_AUTOCAL
        g_calib_state    = CALIB_IDLE;
    #endif
        
    g_stop_busy = 0u;

    #if CONTROL_ENABLE_AUTOCAL
        /* Si llegó un START durante el STOP, ejecutalo ahora */
        if (g_start_queued)
        {
            float r0 = g_start_ref_q;
            g_start_queued = 0u;
            control_start(r0);
        }
    #endif

    return true;

}

/* =======================
   Carga coeficientes (PAQUETE FIJO 25 floats)
   ======================= */

void control_apply_tf(const float* c, uint16_t n)
{
    if (!c || n < 25u) return;

    for (uint8 i = 0u; i < (uint8)(CONTROL_TF_MAX_ORDER + 1u); i++) {
        tf_b[i] = c[i];
        tf_a[i] = c[i + 11u];
    }

    tf_order = tf_order_from_float(c[22]);

    for (uint8 i = (uint8)(tf_order + 1u); i < (uint8)(CONTROL_TF_MAX_ORDER + 1u); i++) {
        tf_b[i] = 0.0f;
        tf_a[i] = 0.0f;
    }

    if (tf_a[0] == 0.0f) {
        tf_a[0] = 1.0f;
    }
    if (tf_a[0] != 1.0f) {
        const float a0 = tf_a[0];
        for (uint8 i = 0u; i <= tf_order; i++) {
            tf_b[i] /= a0;
        }
        for (uint8 i = 1u; i <= tf_order; i++) {
            tf_a[i] /= a0;
        }
        tf_a[0] = 1.0f;
    }

    UARTP_StreamN    = u16_from_float(c[23]);
    UARTP_StreamFsHz = c[24];

    memset(tf_w, 0, sizeof(tf_w));
}

void control_apply_ss(const float* c, uint16_t n)
{
    if (!c || n < 25u) return;

    ss_A[0] = c[0]; ss_A[1] = c[1]; ss_A[2] = c[2];
    ss_A[3] = c[3]; ss_A[4] = c[4]; ss_A[5] = c[5];
    ss_A[6] = c[6]; ss_A[7] = c[7]; ss_A[8] = c[8];

    ss_B[0] = c[9];  ss_B[1] = c[10]; ss_B[2] = c[11];

    ss_C[0] = c[12]; ss_C[1] = c[13]; ss_C[2] = c[14];

    ss_D    = c[15];

    ss_L[0] = c[16]; ss_L[1] = c[17]; ss_L[2] = c[18];

    ss_K[0] = c[19]; ss_K[1] = c[20]; ss_K[2] = c[21];

    ss_Kx   = c[22];

    UARTP_StreamN    = u16_from_float(c[23]);
    UARTP_StreamFsHz = c[24];

    xhat[0] = 0.0f; xhat[1] = 0.0f; xhat[2] = 0.0f;
    zhat[0] = 0.0f; zhat[1] = 0.0f; zhat[2] = 0.0f;
    g_vint  = 0.0f;
}



bool control_is_calibrating(void)
{
#if CONTROL_ENABLE_AUTOCAL
    return (g_calib_state == CALIB_RAMP) ||
       (g_calib_state == CALIB_POSTMOVE) ||
       (g_calib_state == CALIB_SETTLE);

#else
    return false;
#endif
}

bool control_u0_is_valid(void)
{
    return (g_u0_valid != 0u);
}













/* =======================
   CONTROL STEP (main)
   + Anti-windup (freeze) en:
     - TF (congelando estados DF2T si satura y el error empuja hacia afuera)
     - SS_PRED_I / SS_ACT_I (congelando g_vint si satura y el error empuja hacia afuera)
   ======================= */
void control_step(void)
{
    uint8 intr = CyEnterCriticalSection();
    controlPin_Write(1);

    float y_phys = control_last_y;
    control_sample_pending = 0u;

    float r_phys = g_ref;

    /* u_prev_cmd: Δu usado para yhat/innov (consistente con tu obs) */
    float u_prev_cmd = ucmd_from_uphy(g_u_out);

    CyExitCriticalSection(intr);

    r_phys = clamp_ref(r_phys);

#if CONTROL_ENABLE_AUTOCAL
    /* ... TODO lo tuyo de autocal igual (sin cambios) ... */
    if (g_calib_state == CALIB_RAMP)
    {
        g_calib_total_samples++;

        if (g_calib_level_samples == 0u) {
            g_calib_y_level0 = y_phys;
            g_calib_hits = 0u;
        }

        g_calib_level_samples++;

        float dy = y_phys - g_calib_y_level0;
        if (dy < 0) dy = -dy;

        if (dy >= CONTROL_CALIB_DY_CM) {
            if (g_calib_hits < 65535u) g_calib_hits++;
        } else {
            g_calib_hits = 0u;
        }

        if (g_calib_hits >= (uint16_t)CONTROL_CALIB_MIN_HITS)
        {
            g_u0_offset_us = g_calib_u_phy;
            g_u0_valid     = 1u;

            g_postmove_u_phy = g_calib_u_phy;
            g_postmove_cnt   = 0u;
            g_postmove_need  = samples_from_ms((uint16_t)CONTROL_CALIB_POSTMOVE_MS);
            g_calib_state    = CALIB_POSTMOVE;

            write_u(ucmd_from_uphy(g_postmove_u_phy));

            UARTP_Telemetry_Push(g_u_cmd, y_phys);
            controlPin_Write(0);
            return;
        }

        if (g_calib_total_samples >= CONTROL_CALIB_MAX_SAMPLES || g_calib_u_phy >= CONTROL_CALIB_MAX_US)
        {
            g_calib_state   = CALIB_FAIL;
            g_u0_offset_us  = CONTROL_U_OFFSET;
            g_u0_valid      = 0u;

            write_u(0.0f);
            UARTP_Telemetry_Push(g_u_cmd, y_phys);
            controlPin_Write(0);
            return;
        }

        if (g_calib_level_samples < (uint16_t)CONTROL_CALIB_HOLD_SAMPLES)
        {
            write_u(ucmd_from_uphy(g_calib_u_phy));
            UARTP_Telemetry_Push(g_u_cmd, y_phys);
            controlPin_Write(0);
            return;
        }

        g_calib_u_phy += CONTROL_CALIB_STEP_US;
        if (g_calib_u_phy > CONTROL_CALIB_MAX_US) g_calib_u_phy = CONTROL_CALIB_MAX_US;

        g_calib_level_samples = 0u;

        write_u(ucmd_from_uphy(g_calib_u_phy));
        UARTP_Telemetry_Push(g_u_cmd, y_phys);
        controlPin_Write(0);
        return;
    }

    if (g_calib_state == CALIB_POSTMOVE)
    {
        write_u(ucmd_from_uphy(g_postmove_u_phy));

        if (g_postmove_cnt < 65535u) g_postmove_cnt++;

        if (g_postmove_cnt >= g_postmove_need)
        {
            write_u(0.0f);
            g_calib_state       = CALIB_SETTLE;
            g_settle_first      = 1u;
            g_settle_stable_cnt = 0u;

            UARTP_Telemetry_Push(g_u_cmd, y_phys);
            controlPin_Write(0);
            return;
        }

        UARTP_Telemetry_Push(g_u_cmd, y_phys);
        controlPin_Write(0);
        return;
    }

    if (g_calib_state == CALIB_SETTLE)
    {
        write_u(0.0f);

        if (g_settle_first) {
            g_settle_y_prev = y_phys;
            g_settle_first = 0u;
            g_settle_stable_cnt = 0u;
            UARTP_Telemetry_Push(g_u_cmd, y_phys);
            controlPin_Write(0);
            return;
        }

        float dy = y_phys - g_settle_y_prev;
        if (dy < 0) dy = -dy;
        g_settle_y_prev = y_phys;

        if (dy <= (float)CONTROL_U0_SETTLE_DY_CM) {
            if (g_settle_stable_cnt < 65535u) g_settle_stable_cnt++;
        } else {
            g_settle_stable_cnt = 0u;
        }

        if (g_settle_stable_cnt >= (uint16_t)CONTROL_U0_SETTLE_SAMPLES)
        {
            g_calib_state = CALIB_DONE;

            if (UARTP_Impl != UARTP_IMPL_TF && UARTP_Impl != UARTP_IMPL_OPENLOOP) {
                ss_predict_from_xu(xhat, 0.0f, zhat);
            }

            UARTP_Telemetry_Push(g_u_cmd, y_phys);
            controlPin_Write(0);
            return;
        }

        UARTP_Telemetry_Push(g_u_cmd, y_phys);
        controlPin_Write(0);
        return;
    }
#endif

    /* referencias en modo closed-loop: en cm (físico) -> relativo */
    float y_rel = y_rel_from_y(y_phys);
    float r_rel = r_rel_from_r(r_phys);

    /* límites de Δu (cmd) (dinámicos por u0) */
    const float umin_cmd = cmd_min_dyn();
    const float umax_cmd = cmd_max_dyn();

switch (UARTP_Impl)
{
    case UARTP_IMPL_OPENLOOP:
    {
#if CONTROL_OL_REF_IS_DELTA
        float u_cmd = satf(r_phys, umin_cmd, umax_cmd);
        write_u(u_cmd);
#else
        float u_cmd = ucmd_from_uphy(r_phys);
        write_u(u_cmd);
#endif
    } break;

    case UARTP_IMPL_TF:
{
    /* Error en relativo (cm) */
    const float e = (r_rel - y_rel);

    /* TF devuelve comando relativo Δu (us) */
    float u_unsat = tf_step(e);

    /* Saturación en Δu (dinámica por u0) */
    float u_cmd = satf(u_unsat, umin_cmd, umax_cmd);

    /* write_u espera Δu */
    write_u(u_cmd);
} break;


case UARTP_IMPL_SS_PRED_NOI:
case UARTP_IMPL_SS_PRED_I:
{
    uint8 has_i = ss_mode_has_integrator((uint8)UARTP_Impl);

    if (has_i)
    {
        /* Ogata: v(k+1)=v(k)+e ; u = K1*v(k+1) - K2*xhat */
        const float K1 = ss_Kx;                 /* K1 (integrador) */
        const float e  = (r_rel - y_rel);

        const float v0 = g_vint;
        const float v1 = v0 + e;                /* si querés Ts: v0 + g_Ts*e */

        const float u_unsat = (K1 * v1) - dot3_cmsis(ss_K, xhat);  /* ss_K = K2 */
        const float u_cmd   = satf(u_unsat, umin_cmd, umax_cmd);

        /* Anti-windup: CONDITIONAL INTEGRATION
           - Integro si NO satura
           - Si satura: integro solo si el error empuja hacia adentro
             (saturado arriba => e<0 baja u; saturado abajo => e>0 sube u) */
        if (u_unsat == u_cmd)
        {
            g_vint = v1;
        }
        else
        {
            if ( (u_unsat > umax_cmd && e < 0.0f) ||
                 (u_unsat < umin_cmd && e > 0.0f) )
            {
                g_vint = v1;   /* deja integrar: desatura */
            }
            else
            {
                g_vint = v0;   /* freeze */
            }
        }

        write_u(u_cmd);
    }
    else
    {
        /* NOI: NO TOCAR */
        float u_cmd = ss_Kr_eff()*r_rel - dot3_cmsis(ss_K, xhat);
        write_u(u_cmd);
    }

    /* predictor step (NO TOCAR) */
    {
        float u_k_cmd = ucmd_from_uphy(g_u_out);
        ss_observer_pred_step(y_rel, u_prev_cmd, u_k_cmd);
    }
} break;

case UARTP_IMPL_SS_ACT_NOI:
case UARTP_IMPL_SS_ACT_I:
default:
{
    uint8 has_i = ss_mode_has_integrator((uint8)UARTP_Impl);

    /* corrección ACT (NO TOCAR) */
    ss_observer_act_correct(y_rel, u_prev_cmd);

    if (has_i)
    {
        /* Ogata: v(k+1)=v(k)+e ; u = K1*v(k+1) - K2*xhat */
        const float K1 = ss_Kx;
        const float e  = (r_rel - y_rel);

        const float v0 = g_vint;
        const float v1 = v0 + e;

        const float u_unsat = (K1 * v1) - dot3_cmsis(ss_K, xhat);
        const float u_cmd   = satf(u_unsat, umin_cmd, umax_cmd);

        /* Anti-windup: CONDITIONAL INTEGRATION (igual que arriba) */
        if (u_unsat == u_cmd)
        {
            g_vint = v1;
        }
        else
        {
            if ( (u_unsat > umax_cmd && e < 0.0f) ||
                 (u_unsat < umin_cmd && e > 0.0f) )
            {
                g_vint = v1;
            }
            else
            {
                g_vint = v0;
            }
        }

        write_u(u_cmd);
    }
    else
    {
        /* NOI: NO TOCAR */
        float u_cmd = ss_Kr_eff()*r_rel - dot3_cmsis(ss_K, xhat);
        write_u(u_cmd);
    }

    /* predicción ACT (NO TOCAR) */
    {
        float u_k_cmd = ucmd_from_uphy(g_u_out);
        ss_observer_act_predict(u_k_cmd);
    }
} break;


}

    UARTP_Telemetry_Push(g_u_cmd, y_phys);
    controlPin_Write(0);
}
