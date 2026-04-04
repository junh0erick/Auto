/* ============================================================
   ctrl_pend.c — Two-plant on-PSoC controller (no arm_math)
   ============================================================ */
#include "ctrl_pend.h"
#include "pendulo.h"
#include "motor.h"
#include <string.h>

/* ============================================================
   Physical constants
   ============================================================ */
#define PI_F            3.14159265f
#define MOTOR_CPR_F     1040.0f
#define PENDULO_CPR_F   10000.0f
#define PWM_MAX_F       1264.0f
#define PWM_MIN_F      (-1264.0f)

/* ============================================================
   Per-loop state struct
   ============================================================ */
typedef struct {
    uint8  mode;            /* CTRL_MODE_* */

    /* TF – Direct Form II Transposed, order 5 (6 coeffs each) */
    float  tf_b[6];
    float  tf_a[6];         /* a[0] always 1 after normalization */
    float  tf_w[5];         /* delay state */

    /* SS – 2 states */
    float  A[4];            /* 2×2 row-major: [A11 A12 A21 A22] */
    float  B[2];            /* 2×1 */
    float  C[2];            /* 1×2 */
    float  D;
    float  L[2];            /* observer gain */
    float  K[2];            /* state feedback gain */
    float  Kx;              /* Kr (no-integrator) or Ki (integrator) */

    float  xhat[2];         /* posterior state estimate */
    float  zhat[2];         /* prior (actual observer) */
    float  vint;            /* integral accumulator */

    float  ref;             /* current setpoint */
    float  u_out;           /* last output (before saturation for plant 1) */

    /* Outer-loop decimation (plant 1 only, ignored for plant 0) */
    uint16 N;               /* run outer every N inner ticks */
    uint16 cnt;             /* inner tick counter */

} CtrlLoop;

/* ============================================================
   Module-level state
   ============================================================ */
static CtrlLoop g_lp[PLANT_COUNT];
static float    g_Ts      = 0.005f;   /* inner sample time [s] */
static uint8    g_running = 0u;

/* Telemetry */
volatile float ctrl_telem_u1    = 0.0f;
volatile float ctrl_telem_y1    = 0.0f;
volatile float ctrl_telem_u2    = 0.0f;
volatile float ctrl_telem_y2    = 0.0f;
volatile uint8 ctrl_telem_ready = 0u;

/* Timer period */
volatile uint8  ctrl_period_pending = 0u;
volatile uint32 ctrl_period_ticks   = 0u;

/* ============================================================
   Helpers
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

static uint8 mode_actual(uint8 m)
{
    return (m == CTRL_MODE_SS_ACT_NOI) || (m == CTRL_MODE_SS_ACT_I);
}

/* ============================================================
   TF – DF2T step
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
   SS helpers (plain C, 2-state)
   ============================================================ */
/* ŷ = C*x + D*u */
static float ss_yhat(CtrlLoop *lp, const float x[2], float u)
{
    return lp->C[0]*x[0] + lp->C[1]*x[1] + lp->D*u;
}

/* z = A*x + B*u */
static void ss_predict(CtrlLoop *lp, const float x[2], float u, float z[2])
{
    z[0] = lp->A[0]*x[0] + lp->A[1]*x[1] + lp->B[0]*u;
    z[1] = lp->A[2]*x[0] + lp->A[3]*x[1] + lp->B[1]*u;
}

/* u_cmd = −K*xhat + Kx*(vint or r) */
static float ss_u_cmd(CtrlLoop *lp)
{
    float extra = mode_has_i(lp->mode) ? (lp->Kx * lp->vint)
                                       : (lp->Kx * lp->ref);
    return extra - (lp->K[0]*lp->xhat[0] + lp->K[1]*lp->xhat[1]);
}

/* Predictor observer update:
   innov = y − ŷ(xhat, u_prev)
   xhat ← A*xhat + B*u + L*innov                                */
static void ss_obs_pred(CtrlLoop *lp, float y, float u_prev, float u)
{
    float innov = y - ss_yhat(lp, lp->xhat, u_prev);
    float z[2];
    ss_predict(lp, lp->xhat, u, z);
    lp->xhat[0] = z[0] + lp->L[0]*innov;
    lp->xhat[1] = z[1] + lp->L[1]*innov;
}

/* Actual observer:
   Correct: xhat ← zhat + L*(y − ŷ(zhat, u_prev))
   Predict: zhat ← A*xhat + B*u                                 */
static void ss_obs_act_correct(CtrlLoop *lp, float y, float u_prev)
{
    float innov = y - ss_yhat(lp, lp->zhat, u_prev);
    lp->xhat[0] = lp->zhat[0] + lp->L[0]*innov;
    lp->xhat[1] = lp->zhat[1] + lp->L[1]*innov;
}

static void ss_obs_act_predict(CtrlLoop *lp, float u)
{
    float z[2];
    ss_predict(lp, lp->xhat, u, z);
    lp->zhat[0] = z[0];
    lp->zhat[1] = z[1];
}

/* ============================================================
   Single loop step → returns raw output (unsaturated)
   ============================================================ */
static float run_step(CtrlLoop *lp, float y)
{
    float u      = 0.0f;
    float u_prev = lp->u_out;

    switch (lp->mode)
    {
        case CTRL_MODE_TF:
            u = tf_step(lp, lp->ref - y);
            break;

        case CTRL_MODE_SS_PRED_NOI:
        case CTRL_MODE_SS_PRED_I:
            if (mode_has_i(lp->mode)) lp->vint += (lp->ref - y) * g_Ts;
            u = ss_u_cmd(lp);
            ss_obs_pred(lp, y, u_prev, u);
            break;

        case CTRL_MODE_SS_ACT_NOI:
        case CTRL_MODE_SS_ACT_I:
            ss_obs_act_correct(lp, y, u_prev);
            if (mode_has_i(lp->mode)) lp->vint += (lp->ref - y) * g_Ts;
            u = ss_u_cmd(lp);
            ss_obs_act_predict(lp, u);
            break;

        case CTRL_MODE_OPEN_LOOP:
            u = lp->ref;    /* output = reference directly */
            break;

        default:            /* CTRL_MODE_OFF */
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
        g_lp[p].mode = CTRL_MODE_OFF;
        g_lp[p].N    = 1u;
    }
    g_running           = 0u;
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
   Parses 25 floats into the relevant controller state.
   Mode MUST be set before calling this.
   ============================================================ */
void ctrl_apply_coeffs(uint8 plant_id, const float* c, uint16 n)
{
    if (plant_id >= PLANT_COUNT) return;
    if (!c || n < 16u) return;

    CtrlLoop *lp = &g_lp[plant_id];

    /* --- Always parse N and Fs --- */
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

    /* --- TF coefficients --- */
    if (lp->mode == CTRL_MODE_TF)
    {
        uint8 i;
        for (i = 0u; i < 6u; i++) {
            lp->tf_b[i] = c[i];
            lp->tf_a[i] = c[i + 6u];
        }
        /* normalise by a[0] */
        float a0 = lp->tf_a[0];
        if (a0 != 0.0f && a0 != 1.0f) {
            uint8 j;
            for (j = 0u; j < 6u; j++) lp->tf_b[j] /= a0;
            for (j = 1u; j < 6u; j++) lp->tf_a[j] /= a0;
            lp->tf_a[0] = 1.0f;
        }
        memset(lp->tf_w, 0, sizeof(lp->tf_w));
    }
    /* --- SS coefficients --- */
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
    }
    /* OPEN_LOOP / OFF: only N and Fs matter, already parsed above */
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
    g_lp[PLANT_OUTER].ref = 0.0f;     /* pendulum upright = 0 rad */

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
   ctrl_step()   — call once per g_flag_control tick
   ============================================================ */
void ctrl_step(void)
{
    int32  theta_cnt;
    int16  delta_om_cnt;

    pendulo_read(&theta_cnt, &delta_om_cnt);

    /* --- Convert to physical units --- */
    float y1 = (float)delta_om_cnt * (2.0f * PI_F / MOTOR_CPR_F)    / g_Ts; /* ω [rad/s] */
    float y2 = (float)theta_cnt    * (2.0f * PI_F / PENDULO_CPR_F);          /* θ [rad]   */

    CtrlLoop *inner = &g_lp[PLANT_INNER];
    CtrlLoop *outer = &g_lp[PLANT_OUTER];

    float u1 = 0.0f;
    float u2 = outer->u_out;    /* outer output: last computed omega_ref */

    /* --- Outer loop (runs every N inner ticks) --- */
    if (outer->mode != CTRL_MODE_OFF)
    {
        outer->cnt++;
        if (outer->cnt >= outer->N)
        {
            outer->cnt = 0u;
            u2 = run_step(outer, y2);
            /* outer output becomes inner reference */
            if (inner->mode != CTRL_MODE_OFF) {
                inner->ref = u2;
            }
        }
    }

    /* --- Inner loop (runs every tick) --- */
    if (inner->mode != CTRL_MODE_OFF)
    {
        u1 = run_step(inner, y1);
        Motor_Control((int16)satf(u1, PWM_MIN_F, PWM_MAX_F));
    }
    else if (outer->mode != CTRL_MODE_OFF)
    {
        /* Inner off, outer on → outer output drives motor directly */
        u1 = satf(u2, PWM_MIN_F, PWM_MAX_F);
        Motor_Control((int16)u1);
    }
    else
    {
        Motor_Free();
    }

    /* --- Snapshot telemetry (atomic write) --- */
    {
        uint8 intr = CyEnterCriticalSection();
        ctrl_telem_u1    = u1;
        ctrl_telem_y1    = y1;
        ctrl_telem_u2    = u2;
        ctrl_telem_y2    = y2;
        ctrl_telem_ready = 1u;
        CyExitCriticalSection(intr);
    }
}
