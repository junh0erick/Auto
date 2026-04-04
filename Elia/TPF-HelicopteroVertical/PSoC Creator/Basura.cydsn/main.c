#include "project.h"
#include "uartp_sw.h"
#include "control_app.h"
#include <stdint.h>

/* ==============================
   CONFIG ANALÓGICA
   ============================== */
#define VDAC_FS_V        (4.08f)
#define VDAC_MAX_CODE    (255.0f)

/* sample time default */
#define TS_SEC_DEFAULT   (1e-3f)

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
   ISR Sampling (EOC del ADC)
   ============================== */
CY_ISR(isr_sampling_handler)
{
    control_on_sample_isr();
    isr_sampling_ClearPending();
}

CY_ISR(isr_start_sampling_handler)
{
    ADC_muestra_StartConvert();
}

/* ==============================
   UARTP set_ref callback
   ============================== */
static void my_set_ref(float r)
{
    control_set_reference(r);
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

    timer_led_Start();
    timer_uart_Start();
    timer_sampling_Start();

    timer_sampling_WritePeriod(1500);

    /* --- Control app: IO + Ts default --- */
    control_register_io(my_sample_isr_cb, my_write_u);
    control_set_sample_time(TS_SEC_DEFAULT);

    /* --- ISR del ADC EOC --- */
    isr_start_sampling_StartEx(isr_start_sampling_handler);
    isr_sampling_StartEx(isr_sampling_handler);

    /* --- UARTP config --- */
    uartp_cfg_t cfg = {
        .stop_step = control_stop_suave_step,
        .start     = control_start,
        .apply_tf  = control_apply_tf,
        .apply_ss  = control_apply_ss,
        .set_ref   = my_set_ref
    };
    UARTP_Init(&cfg);

    for (;;)
    {
        /* aplicar cambio de periodo pedido por control_apply_* */
        if (control_period_update_pending()) {
            uint16_t pt = control_period_get_pending_ticks();
            timer_sampling_WritePeriod(pt);
            control_period_clear_pending();
        }

        switch (UARTP_SysMode)
        {
            case UARTP_SYS_COMMAND:
            {
                /* LED distinto según estado */
                if (UARTP_CtrlState == UARTP_CTRL_STOPPED)      registro_led_Write(11);
                else if (UARTP_CtrlState == UARTP_CTRL_STOPPING) registro_led_Write(12);
                else                                            registro_led_Write(13);

                (void)UARTP_ProcessOnce();
            } break;

            case UARTP_SYS_CONTROL:
            {
                /* En CONTROL: LED = mode+1 (1..5) */
                registro_led_Write((uint8)(UARTP_Impl + 1u));

                if (control_sample_pending) {
                    control_step();
                }

                UARTP_ControlTick();

                /* Telemetría ligera: si hay snapshot, manda u luego y */
                float u_send, y_send;
                if (control_telem_pop(&u_send, &y_send)) {
                    UARTP_UART_PutArray((uint8*)&u_send, 4);
                    UARTP_UART_PutArray((uint8*)&y_send, 4);
                }
            } break;

            default:
                registro_led_Write(99);
                UARTP_EnterCommandMode();
                break;
        }
    }
}
