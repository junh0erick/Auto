#include "project.h"
#include "uartp_sw.h"
#include "control_app.h"
#include <stdint.h>

/* ==============================
   CONFIG ANALÓGICA
   ============================== */
#define VDAC_FS_V        (4.08f)
#define VDAC_MAX_CODE    (255.0f)

/* Default Ts (si no se cambia por UART) */
#define TS_SEC_DEFAULT   (1e-3f)

/* Clock real del timer_sampling (AJUSTÁ a tu diseño) */
#define TIMER_SAMPLING_CLK_HZ   (1.491e6f)
/* Si tu Fs efectiva es clk/(2*period), dejá esto en 2.0f; si es clk/period poné 1.0f */
#define TIMER_SAMPLING_FACTOR   (2.0f)

/* Si tu control_app expone estas globals, usalas para telemetría */
extern volatile uint8 control_sample_pending;

/* ==============================
   Helpers analógicos
   ============================== */
static inline uint8 clamp_u8_from_volts(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > VDAC_FS_V) v = VDAC_FS_V;

    float code = (v * VDAC_MAX_CODE) / VDAC_FS_V;
    if (code < 0.0f)   code = 0.0f;
    if (code > 255.0f) code = 255.0f;
    return (uint8)(code + 0.5f);
}

/* ==============================
   Callbacks IO (sensor/actuador)
   ============================== */
static void my_write_u(float u_volts)
{
    VDAC8_esfuerzo_SetValue(clamp_u8_from_volts(u_volts));
}

/* Este callback lo llama control_on_sample_isr() */
static void my_sample_isr_cb(void)
{
    int16 counts = ADC_muestra_GetResult16();
    float y = ADC_muestra_CountsTo_Volts(counts);
    control_sample_isr_push(y);
}

/* ==============================
   ISR Sampling
   ============================== */

/* Timer tick -> dispara conversión */
CY_ISR(isr_start_sampling_handler)
{
    ADC_muestra_StartConvert();
    isr_start_sampling_ClearPending();
}

/* ADC EOC -> consume muestra y marca pending */
CY_ISR(isr_sampling_handler)
{
    control_on_sample_isr();      /* llama a my_sample_isr_cb adentro */
    isr_sampling_ClearPending();
}

/* ==============================
   Sampling control (callbacks para UARTP)
   ============================== */
static float period_to_ts_sec(uint16 period_ticks)
{
    if (period_ticks == 0u) period_ticks = 1u;
    return (TIMER_SAMPLING_FACTOR * (float)period_ticks) / TIMER_SAMPLING_CLK_HZ;
}

static void my_sampling_enable(void)
{
    /* IMPORTANTE: Sleep/Wakeup acá te deja colgado fácil; Start/Stop es lo robusto */
    timer_sampling_Start();
}

static void my_sampling_disable(void)
{
    timer_sampling_Stop();
}


static void my_sampling_change_fs(float fs_hz)
{
    if (!(fs_hz > 0.0f)) return;

    /* tu fórmula real (ajustá el 750000.0f si cambia tu clock efectivo) */
    uint16 ticks = (uint16)(750000.0f / fs_hz + 0.5f);
    if (ticks < 1u) ticks = 1u;

    timer_sampling_WritePeriod(ticks);

    /* si tu control necesita Ts coherente */
    control_set_sample_time(1.0f / fs_hz);
}

static void my_sampling_clear_flags(void)
{
    (void)timer_sampling_ReadStatusRegister();
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

    /* Timers (UARTP maneja timer_uart internamente, pero Start no molesta) */
    timer_led_Start();
    timer_uart_Start();

    /* --- Control app: IO + Ts default --- */
    control_register_io(my_sample_isr_cb, my_write_u);
    control_set_sample_time(TS_SEC_DEFAULT);

    /* --- ISR de muestreo --- */
    isr_start_sampling_StartEx(isr_start_sampling_handler);
    isr_sampling_StartEx(isr_sampling_handler);

    /* --- UARTP config --- */
    uartp_cfg_t cfg = {
        .stop_step = control_stop_suave_step,
        .start     = control_start,
        .force_min = control_force_min,
        .apply_tf  = control_apply_tf,
        .apply_ss  = control_apply_ss,

        .sampling_enable        = my_sampling_enable,
        .sampling_disable       = my_sampling_disable,
        .sampling_change_fs     = my_sampling_change_fs,
        .sampling_clear_flags   = my_sampling_clear_flags
    };
    UARTP_Init(&cfg);

    for (;;)
    {
        switch (UARTP_SysMode)
        {
            case UARTP_SYS_COMMAND:
                (void)UARTP_ProcessOnce();
                registro_led_Write(1);
                break;

            case UARTP_SYS_CONTROL:
                if (control_sample_pending)
                {
                    control_sample_pending = 0u;
                    control_step();

                   
                }

                UARTP_ControlTick();
                registro_led_Write(2);
                break;

            default:
                UARTP_EnterCommandMode();
                registro_led_Write(3);
                break;
        }
    }
}
