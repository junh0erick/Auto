#include "project.h"
#include "control_app.h"
#include "uartp_sw.h"
#include <stdint.h>

/* ==============================
   CONFIG
   ============================== */
#define VDAC_FS_V        (4.08f)
#define VDAC_MAX_CODE    (255.0f)

#define TS_SEC           (10e-6f)     // 100 kHz
#define REF_DEFAULT      (1.0f)       // referencia para probar (step 1.0)

/* ===== Blink timing (con clock 1 kHz => 1 tick = 1 ms) ===== */
#define BLINK_ON_MS      (120u)
#define BLINK_OFF_MS     (180u)

/* ==============================
   Rotación / estados (DEFINIR ANTES DE BLINK)
   ============================== */
typedef enum {
    ST_OFF = 0,
    ST_RUN,
    ST_STOPPING
} run_state_t;

/* ==============================
   COEFICIENTES (LOS TUYOS)
   ============================== */


static const float COEFF_TF[16] = {
    0.462607056f,
    -0.0626070574f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    1.0f,
    -1.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f
};
static const float COEFF_SS_PRED_NOI[16] = {
    0.135335281f,
    0.000000000f,
    0.000000000f,
    0.000000000f,
    1.000000000f,
    0.000000000f,
    0.864664733f,
    0.000000000f,
    0.000000000f,
    -0.190437645f,
    0.000000000f,
    -0.464664727f,
    0.000000000f,
    0.462607056f,
    0.000000000f,
    0.000000000f
};

static const float COEFF_SS_ACT_NOI[16] = {
    0.135335281f,
    0.000000000f,
    0.000000000f,
    0.000000000f,
    1.000000000f,
    0.000000000f,
    0.864664733f,
    0.000000000f,
    0.000000000f,
    -1.407154441f,
    0.000000000f,
    -0.464664727f,
    0.000000000f,
    0.462607056f,
    0.000000000f,
    0.000000000f
};

static const float COEFF_SS_PRED_I[16] = {
    0.135335281f,
    0.000000000f,
    0.000000000f,
    0.000000000f,
    1.000000000f,
    0.000000000f,
    0.864664733f,
    0.000000000f,
    0.000000000f,
    -0.190437645f,
    0.000000000f,
    -0.164664716f,
    0.000000000f,
    23130.353515625f,
    0.000000000f,
    0.000000000f
};

static const float COEFF_SS_ACT_I[16] = {
    0.135335281f,
    0.000000000f,
    0.000000000f,
    0.000000000f,
    1.000000000f,
    0.000000000f,
    0.864664733f,
    0.000000000f,
    0.000000000f,
    -1.407154441f,
    0.000000000f,
    -0.164664716f,
    0.000000000f,
    23130.353515625f,
    0.000000000f,
    0.000000000f
};
/* ==============================
   Helpers analógicos
   ============================== */
static inline uint8 clamp_u8_from_volts(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > VDAC_FS_V) v = VDAC_FS_V;

    float code = (v * VDAC_MAX_CODE) / VDAC_FS_V;
    if (code < 0.0f) code = 0.0f;
    if (code > 255.0f) code = 255.0f;
    return (uint8)(code + 0.5f);
}

/* ==============================
   Callbacks de CONTROL_APP
   ============================== */
static void my_write_u(float u_volts)
{
    VDAC8_esfuerzo_SetValue(clamp_u8_from_volts(u_volts));
}

static void my_sample_isr_cb(void)
{
    int16 counts = ADC_muestra_GetResult16();
    float y = ADC_muestra_CountsTo_Volts(counts);
    control_sample_isr_push(y);
}

/* ==============================
   ISR Sampling (EOC)
   ============================== */
CY_ISR(isr_sampling_handler)
{
    control_on_sample_isr();
    isr_sampling_ClearPending();
}

/* ==============================
   BLINK FSM con Timer one-shot
   ============================== */
typedef enum {
    BLK_IDLE = 0,
    BLK_ON_WAIT,
    BLK_OFF_WAIT
} blink_state_t;

static volatile uint8_t g_timer_evt = 0u;

static blink_state_t blk_state = BLK_IDLE;
static uint8_t blk_pulses_left = 0u;

static inline void blink_timer_start_ms(uint16_t ms)
{
    if (ms == 0u) ms = 1u;

    timer_tintineo_Stop();
    timer_tintineo_WriteCounter(0u);
    timer_tintineo_WritePeriod((uint16_t)(ms - 1u));
    timer_tintineo_Start();
}

static inline uint8_t blink_active(void)
{
    return (blk_state != BLK_IDLE);
}

static void blink_start_mode(uint8_t mode_idx)
{
    blk_pulses_left = (uint8_t)(mode_idx + 1u);

    LED_BUILTIN_Write(1);
    blk_state = BLK_ON_WAIT;

    blink_timer_start_ms(BLINK_ON_MS);
}

/* ✅ ahora run_state_t ya existe */
static void blink_step(run_state_t run_state_now)
{
    if (blk_state == BLK_IDLE) return;

    if (blk_state == BLK_ON_WAIT) {
        LED_BUILTIN_Write(0);
        blk_state = BLK_OFF_WAIT;
        blink_timer_start_ms(BLINK_OFF_MS);
        return;
    }

    if (blk_pulses_left > 0u) blk_pulses_left--;

    if (blk_pulses_left == 0u) {
        blk_state = BLK_IDLE;
        LED_BUILTIN_Write((run_state_now == ST_RUN) ? 1u : 0u);
        return;
    }

    LED_BUILTIN_Write(1);
    blk_state = BLK_ON_WAIT;
    blink_timer_start_ms(BLINK_ON_MS);
}

CY_ISR(isr_timer_handler)
{
    g_timer_evt = 1u;
    isr_timer_ClearPending();
}

/* ==============================
   Rotación de controladores
   ============================== */
static volatile uint8_t g_btn_evt = 0u;

static run_state_t g_state = ST_OFF;
static uint8_t g_idx = 0u;

static const uint8_t k_impls[5] = {
    UARTP_IMPL_TF,
    UARTP_IMPL_SS_PRED_NOI,
    UARTP_IMPL_SS_ACT_NOI,
    UARTP_IMPL_SS_PRED_I,
    UARTP_IMPL_SS_ACT_I
};

static const float* k_coeffs_ss[5] = {
    0,
    COEFF_SS_PRED_NOI,
    COEFF_SS_ACT_NOI,
    COEFF_SS_PRED_I,
    COEFF_SS_ACT_I
};

static void apply_controller(uint8_t idx)
{
    UARTP_Impl = k_impls[idx];

    if (UARTP_Impl == UARTP_IMPL_TF) {
        control_apply_tf(COEFF_TF, 16u);
    } else {
        control_apply_ss(k_coeffs_ss[idx], 16u);
    }

    control_start(REF_DEFAULT);
}

CY_ISR(isr_toggle_handler)
{
    g_btn_evt = 1u;
    isr_toggle_ref_ClearPending();
}

/* ==============================
   MAIN
   ============================== */
int main(void)
{
    CyGlobalIntEnable;

    VDAC8_esfuerzo_Start();
    follower_Start();
    ADC_muestra_Start();

    LED_BUILTIN_Write(0);
    isr_toggle_ref_StartEx(isr_toggle_handler);

    isr_timer_StartEx(isr_timer_handler);
    timer_tintineo_Stop();

    control_register_io(my_sample_isr_cb, my_write_u);
    control_set_sample_time(TS_SEC);

    isr_sampling_StartEx(isr_sampling_handler);

    for (;;)
    {
        if (g_timer_evt) {
            g_timer_evt = 0u;
            blink_step(g_state);
        }

        if (g_btn_evt && !blink_active())
        {
            g_btn_evt = 0u;

            if (g_state == ST_OFF) {
                apply_controller(g_idx);
                g_state = ST_RUN;
                blink_start_mode(g_idx);

            } else if (g_state == ST_RUN) {
                g_state = ST_STOPPING;
            }
        }

        if (control_sample_pending)
        {
            if (g_state == ST_RUN) {
                control_step();

            } else if (g_state == ST_STOPPING) {
                control_sample_pending = 0u;

                if (control_stop_suave_step()) {
                    g_state = ST_OFF;
                    if (!blink_active()) LED_BUILTIN_Write(0);

                    g_idx++;
                    if (g_idx >= 5u) g_idx = 0u;
                }

            } else {
                control_sample_pending = 0u;
            }
        }
    }
}
