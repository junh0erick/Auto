#include "project.h"
#include "uartp_sw.h"
#include "control_app.h"
#include <stdint.h>

/* ==============================
   CONFIG ANALÓGICA
   ============================== */
#define VDAC_FS_V        (4.08f)
#define VDAC_MAX_CODE    (255.0f)

/* sample time default (si luego lo cambiás por UART, UARTP debería llamar control_set_sample_time) */
#define TS_SEC_DEFAULT   (1e-3)     /* 1 kHz */

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
    /* OJO: esto asume que el ADC ya terminó conversión (EOC) */
    int16 counts = ADC_muestra_GetResult16();
    float y = ADC_muestra_CountsTo_Volts(counts);
    control_sample_isr_push(y);
}

/* ==============================
   ISR Sampling (EOC del ADC)
   ============================== */
CY_ISR(isr_sampling_handler)
{
    /* Esto normalmente:
       - llama al callback my_sample_isr_cb()
       - marca control_sample_pending
       - etc (según tu control_app)
    */
    control_on_sample_isr();
    isr_sampling_ClearPending();
}

CY_ISR(isr_start_sampling_handler)
{
      ADC_muestra_StartConvert();
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

    /* LED: apagado en modo comando, prendido en modo control */
    timer_led_Start();
    timer_uart_Start();
    timer_sampling_Start();
    
    
    /* --- Control app: IO + Ts default --- */
    control_register_io(my_sample_isr_cb, my_write_u);
    control_set_sample_time(TS_SEC_DEFAULT);

    /* --- ISR del ADC EOC (tu esquema de muestreo) --- */
    isr_start_sampling_StartEx(isr_start_sampling_handler);
    
    isr_sampling_StartEx(isr_sampling_handler);

    /* --- UARTP config: UART manda start/stop/coef --- */
    uartp_cfg_t cfg = {
        .stop_step = control_stop_suave_step,
        .start     = control_start,
        .apply_tf  = control_apply_tf,
        .apply_ss  = control_apply_ss
    };
    UARTP_Init(&cfg);

    for (;;)
    {
        switch (UARTP_SysMode)
        {
            case UARTP_SYS_COMMAND:
                /* En comando: procesar frames y aplicar cambios (coef/mode/start/stop/etc) */
                
                (void)UARTP_ProcessOnce();
                registro_led_Write(1);
                break;

            case UARTP_SYS_CONTROL:
                /* En control: correr SOLO cuando hay muestra nueva */
                

                if (control_sample_pending)
                {
                    control_step();
                    
                }

                /* housekeeping de UARTP (telemetría, flags, timeouts, etc) */
                UARTP_ControlTick();
                registro_led_Write(2);
                break;

            default:
                /* fallback seguro */
                
                UARTP_EnterCommandMode();
                registro_led_Write(3);
                break;
        }
    }
}
