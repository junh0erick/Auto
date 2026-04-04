#include "project.h"
#include "control_app.h"
#include "uartp_sw.h"

/* ===== Ajustá esto a tu sampling clock real ===== */
#define FS_HZ   (10000u)            // probá 10 kHz primero
#define TS_S    (1.0f / (float)FS_HZ)

/* Coef corregidos: 0.73536/(z-0.7814) => 0.73536 z^-1 / (1-0.7814 z^-1) */
static const float coeffs_tf[16] = {
    1.0f, 0.0f, 0.0f,0,0,0,
    1.0f, 0.0f, 0.0f,0,0,0,
    0,0,0,0
};

/* ===== Estado de modo ===== */
typedef enum { MODE_WAIT=0, MODE_RUN, MODE_STOPPING } ctrl_mode_t;
static volatile ctrl_mode_t g_mode = MODE_WAIT;
static volatile uint8_t g_req_toggle = 0;

/* ===== ADC counts guardado por ISR (ultraliviano) ===== */
static volatile int16 g_adc_counts = 0;

/* ===== DAC write ===== */
#define VDAC_FS_V        (4.08f)
#define VDAC_MAX_CODE    (255.0f)

static inline uint8 clamp_u8_from_volts(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > VDAC_FS_V) v = VDAC_FS_V;
    float code = (v * VDAC_MAX_CODE) / VDAC_FS_V;
    if (code < 0.0f) code = 0.0f;
    if (code > 255.0f) code = 255.0f;
    return (uint8)(code + 0.5f);
}

static void my_write_u(float u_volts)
{
    VDAC8_esfuerzo_SetValue(clamp_u8_from_volts(u_volts));
}

/* ===== ISR botón (debouncer -> interrupt): solo pide toggle ===== */
CY_ISR(isr_toggle_handler)
{
    g_req_toggle = 1u;
    isr_toggle_ref_ClearPending();
}

/* ===== ISR sampling (EOC del ADC): ULTRALIVIANO ===== */
CY_ISR(isr_sampling_handler)
{
    g_adc_counts = ADC_muestra_GetResult16();
    control_sample_pending = 1u;   // bandera para main
    isr_sampling_ClearPending();
}

int main(void)
{
    CyGlobalIntEnable;

    /* --- Analog init --- */
    VDAC8_esfuerzo_Start();
    
    
    ADC_muestra_Start();

    /* Si tu ADC requiere StartConvert aunque uses SOC, no molesta tenerlo: */
    follower_Start();

    /* --- LED y botón --- */
    LED_BUILTIN_Write(0);
    isr_toggle_ref_StartEx(isr_toggle_handler);

    /* --- Control init --- */
    control_register_io(0, my_write_u);   // NO usamos sample callback (ISR manual)
    control_set_sample_time(TS_S);

    control_apply_tf(coeffs_tf, 16);
    UARTP_Impl = UARTP_IMPL_TF;           // CLAVE: fuerza TF

    /* --- ISR sampling (EOC) --- */
    isr_sampling_StartEx(isr_sampling_handler);

    for (;;)
    {
        /* atender toggle en MAIN (no en ISR) */
        if (g_req_toggle) {
            g_req_toggle = 0u;

            if (g_mode == MODE_WAIT) {
                LED_BUILTIN_Write(1);
                control_start(3.0f);      // tu referencia
                g_mode = MODE_RUN;
            } else {
                LED_BUILTIN_Write(0);
                g_mode = MODE_STOPPING;
            }
        }

        /* si llegó sample */
        if (control_sample_pending)
        {
            /* convertir counts->volts FUERA del ISR */
            int16 c;
            uint8_t intr = CyEnterCriticalSection();
            c = g_adc_counts;
            CyExitCriticalSection(intr);

            control_last_y = ADC_muestra_CountsTo_Volts(c);

            if (g_mode == MODE_RUN) {
                control_step();
            }
            else if (g_mode == MODE_STOPPING) {
                control_sample_pending = 0u;
                if (control_stop_suave_step()) {
                    g_mode = MODE_WAIT;
                }
            }
            else {
                control_sample_pending = 0u;
            }
        }
    }
}
