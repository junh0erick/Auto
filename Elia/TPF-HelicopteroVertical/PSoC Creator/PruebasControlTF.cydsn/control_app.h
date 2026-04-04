#ifndef CONTROL_APP_H
#define CONTROL_APP_H

#include <stdint.h>
#include <stdbool.h>

/* =======================
   Config opcional
   ======================= */
#ifndef CONTROL_LOG_LEN
    #define CONTROL_LOG_LEN (512u)
#endif

#ifndef CONTROL_LOG_FIELDS
    #define CONTROL_LOG_FIELDS (3u)   /* r,y,u */
#endif

/* Saturación / offset físico (por defecto no molestan) */
#ifndef CONTROL_SAT_MIN
    #define CONTROL_SAT_MIN (0.0f)
#endif
#ifndef CONTROL_SAT_MAX
    #define CONTROL_SAT_MAX (4.08f)
#endif
#ifndef CONTROL_U_OFFSET
    #define CONTROL_U_OFFSET (0.0f)
#endif

/* Stop suave (en “unidades del controlador”, antes de offset) */
#ifndef CONTROL_STOP_TARGET
    #define CONTROL_STOP_TARGET (0.0f)
#endif
#ifndef CONTROL_STOP_STEP
    #define CONTROL_STOP_STEP (0.01f)
#endif

/* =======================
   Callbacks IO (genéricos)
   ======================= */
typedef void (*control_write_u_fn_t)(float u_phy);

/* Corre DENTRO del ISR de muestreo.
   Debe ser ultraliviano y terminar llamando:
     control_sample_isr_push(y);
*/
typedef void (*control_sample_isr_fn_t)(void);

/* =======================
   Variables ISR -> main
   ======================= */
extern volatile float   control_last_y;
extern volatile uint8_t control_sample_pending;

/* Referencia “espejo” opcional (debug). La referencia REAL del control es interna (g_ref). */
extern volatile float control_ref_v;

/* =======================
   Registro de IO / ISR glue
   ======================= */
void control_register_io(control_sample_isr_fn_t sample_isr,
                         control_write_u_fn_t    write_u);

/* Llamar dentro del ISR real de muestreo (isr_sampling) */
void control_on_sample_isr(void);

/* Helper: ISR setea las 2 variables pedidas */
void control_sample_isr_push(float y);

/* =======================
   API del control
   ======================= */
void  control_set_reference(float r);
float control_get_reference(void);

void  control_set_sample_time(float Ts);
float control_get_sample_time(void);

bool  control_stop_suave_step(void);

/* START: ref0 VIENE DESDE UART (comando 'i' en MATLAB) */
void  control_start(float ref0);

/* Carga de coeficientes (siempre 16 floats) */
void  control_apply_tf(const float* c, uint16_t n);
void  control_apply_ss(const float* c, uint16_t n);

/* Lo llama main cuando control_sample_pending==1 */
void  control_step(void);

/* Logging */
void     control_log_reset(void);
uint8_t  control_log_ready(void);
const uint8_t* control_log_bytes(uint16_t* nbytes);

#endif /* CONTROL_APP_H */
