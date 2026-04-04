#include "project.h"
#include "uartp_sw.h"
#include "control_app.h"
#include "tfmini_psoc.h"
#include "esc_pwm.h"
#include <stdint.h>

#define TFMINI_FPS_DEFAULT   (100u)

/* =========================================================
   Actuador: u_phy = microsegundos (1000..2000)
   El control_app ya satura, pero acá también por seguridad.
   ========================================================= */
static void my_write_u(float u_us)
{
    if (u_us < 1000.0f) u_us = 1000.0f;
    if (u_us > 2000.0f) u_us = 2000.0f;

    esc_pwm_write_us((uint16_t)(u_us + 0.5f));
}

/* =========================================================
   Callbacks para UARTP -> TFMini
   ========================================================= */
static void my_sampling_enable(void)      { (void)tfmini_enable(); }
static void my_sampling_disable(void)     { (void)tfmini_disable(); }
static void my_sampling_clear_flags(void) { tfmini_clear_flags(); }

static void my_sampling_change_fs(float fs_hz)
{
    if (!(fs_hz > 0.0f)) return;

    uint16_t fps = (uint16_t)(fs_hz + 0.5f);
    if (fps < 1u) fps = 1u;
    if (fps > 1000u) fps = 1000u;

    if (tfmini_set_fps(fps)) {
        /* Ts lo mantenemos coherente con el FPS del sensor */
        control_set_sample_time(1.0f / (float)fps);
    }
}

/* =========================================================
   MAIN
   ========================================================= */
int main(void)
{
    CyGlobalIntEnable;

    /* UARTP usa timer_uart (si tu implementación lo requiere) */
    timer_uart_Start();

    /* ===== ESC PWM ===== */
    esc_pwm_init();
    esc_pwm_write_us(1000u);  /* seguro al boot */

    /* ===== Control app =====
       - sample_isr_cb = NULL porque el muestreo lo manejamos por polling de tfmini_pop_cm()
       - write_u_cb = my_write_u
    */
    control_register_io(NULL, my_write_u);
    control_set_sample_time(1.0f / (float)TFMINI_FPS_DEFAULT);

    /* ===== TFMini ===== */
    tfmini_init();
    (void)tfmini_set_fps(TFMINI_FPS_DEFAULT);
    (void)tfmini_enable();

    /* ===== UARTP ===== */
    uartp_cfg_t cfg = {
        .stop_step = control_stop_suave_step,
        .start     = control_start,
        .force_min = control_force_min,
        .apply_tf  = control_apply_tf,
        .apply_ss  = control_apply_ss,

        .sampling_enable      = my_sampling_enable,
        .sampling_disable     = my_sampling_disable,
        .sampling_change_fs   = my_sampling_change_fs,
        .sampling_clear_flags = my_sampling_clear_flags
    };
    UARTP_Init(&cfg);

    for (;;)
    {
        switch (UARTP_SysMode)
        {
            case UARTP_SYS_COMMAND:
                (void)UARTP_ProcessOnce();
                break;

            case UARTP_SYS_CONTROL:
            {
                uint16_t y_cm;

                /* Consumimos una muestra nueva del sensor */
                if (tfmini_pop_cm(&y_cm)) {
                    control_sample_isr_push((float)y_cm);
                }

                /* Si hay muestra lista -> ejecutar control */
                if (control_sample_pending) {
                    control_sample_pending = 0u;
                    control_step();
                }

                UARTP_ControlTick();
                break;
            }

            default:
                UARTP_EnterCommandMode();
                break;
        }
    }
}
