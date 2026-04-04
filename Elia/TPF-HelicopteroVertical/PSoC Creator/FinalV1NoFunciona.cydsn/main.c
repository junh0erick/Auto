#include "project.h"
#include "control_app.h"
#include "uartp_sw.h"
#include <stdint.h>

/* ==============================
   CONFIG ANALÓGICA
   ============================== */
#define VDAC_FS_V        (4.08f)
#define VDAC_MAX_CODE    (255.0f)

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
   CALLBACK: escribir esfuerzo
   ============================== */
static void write_u_cb(float u_volts)
{
    VDAC8_esfuerzo_SetValue(clamp_u8_from_volts(u_volts));
}

/* ==============================
   CALLBACK: muestreo (ISR-safe)
   ============================== */
static void sample_isr_cb(void)
{
    int16 counts = ADC_muestra_GetResult16();
    float y = ADC_muestra_CountsTo_Volts(counts);
    control_sample_isr_push(y);
}

/* ==============================
   ISR: timer_sampling
   ============================== */
CY_ISR(isr_start_sampling_handler)
{
    /* Avisar a UARTP (decimación / flags de envío) */
    UARTP_OnSampleIsr();

    /* Avisar al control */
    control_on_sample_isr();

    /* Limpiar interrupción */
    timer_sampling_ReadStatusRegister();
}

/* ==============================
   MAIN
   ============================== */
int main(void)
{
    CyGlobalIntEnable;

    /* --- HW analógico --- */
    VDAC8_esfuerzo_Start();
    follower_buffer_Start();
    ADC_muestra_Start();

    /* --- Registrar callbacks de control --- */
    control_register_io(sample_isr_cb, write_u_cb);

    /* --- UART protocol (control total desde MATLAB) --- */
    uartp_cfg_t uart_cfg = {
        .stop_step = control_stop_suave_step,
        .start     = control_start,
        .apply_tf  = control_apply_tf,
        .apply_ss  = control_apply_ss
    };
    UARTP_Init(&uart_cfg);

    /* --- Timer de muestreo --- */
    isr_start_sampling_StartEx(isr_start_sampling_handler);
    timer_sampling_Start();

    for (;;)
    {
        /* COMMAND MODE: UART */
        if (UARTP_SysMode == UARTP_SYS_COMMAND) {
            UARTP_ProcessOnce();
        }

        /* CONTROL MODE: cálculo */
        if (control_sample_pending) {
            control_step();
        }

        /* Stop suave (si fue pedido) */
        UARTP_ControlTick();
    }
}
