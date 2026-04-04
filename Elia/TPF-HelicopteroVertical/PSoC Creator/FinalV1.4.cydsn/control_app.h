/* =========================
   control_app.h
   ========================= */
#ifndef CONTROL_APP_H
#define CONTROL_APP_H

#include <stdint.h>
#include <stdbool.h>

/* =======================
   Config opcional
   ======================= */
#ifndef CONTROL_SAT_MIN
    #define CONTROL_SAT_MIN (0.0f)
#endif
#ifndef CONTROL_SAT_MAX
    #define CONTROL_SAT_MAX (4.08f)
#endif
#ifndef CONTROL_U_OFFSET
    #define CONTROL_U_OFFSET (0.0f)
#endif

#ifndef CONTROL_STOP_TARGET
    #define CONTROL_STOP_TARGET (0.0f)
#endif
#ifndef CONTROL_STOP_STEP
    #define CONTROL_STOP_STEP (0.1f)
#endif



/* Ajustes stop */
#ifndef CONTROL_STOP_TOL_REL
#define CONTROL_STOP_TOL_REL   (0.05f)   /* 5% */
#endif

#ifndef CONTROL_STOP_TOL_ABS
#define CONTROL_STOP_TOL_ABS   (0.005f)  /* 5 mV en u_cmd (ajustá a gusto) */
#endif

#ifndef CONTROL_STOP_DELAY_US
#define CONTROL_STOP_DELAY_US  (1000u)   /* 1 ms entre pasos (ajustá) */
#endif

#ifndef CONTROL_STOP_MAX_ITERS
#define CONTROL_STOP_MAX_ITERS (20000u)  /* failsafe */
#endif

/* =======================
   Callbacks IO
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
extern volatile float   control_ref_v;

/* =======================
   API
   ======================= */
void control_register_io(control_sample_isr_fn_t sample_isr,
                         control_write_u_fn_t    write_u);

void control_on_sample_isr(void);
void control_sample_isr_push(float y);

void  control_set_reference(float r);
float control_get_reference(void);

void  control_set_sample_time(float Ts);
float control_get_sample_time(void);

void  control_start(float ref0);
bool  control_stop_suave_step(void);

void  control_apply_tf(const float* c, uint16_t n);
void  control_apply_ss(const float* c, uint16_t n);

void  control_step(void);
void control_force_min(void);
#endif /* CONTROL_APP_H */
