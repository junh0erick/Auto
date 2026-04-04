/* =========================
   control_app.h
   ========================= */
#ifndef CONTROL_APP_H
#define CONTROL_APP_H

#include <stdint.h>
#include <stdbool.h>

/* =======================
   RANGO DEL ACTUADOR (PWM)
   ======================= */
/* u_phy final que sale a tu callback write_u().
   En tu caso: PWM en microsegundos */
#ifndef CONTROL_SAT_MIN
    #define CONTROL_SAT_MIN (1000.0f)
#endif
#ifndef CONTROL_SAT_MAX
    #define CONTROL_SAT_MAX (2000.0f)
#endif

/* Offset opcional (si quisieras centrar el comando).
   Para PWM no te hace falta: dejalo 0. */
#ifndef CONTROL_U_OFFSET
    #define CONTROL_U_OFFSET (0.0f)
#endif

/* =======================
   RANGOS DE REFERENCIA
   ======================= */
/* --- OPEN LOOP: ref = PWM directo --- */
#ifndef CONTROL_OL_REF_MIN
    #define CONTROL_OL_REF_MIN (1000.0f)
#endif
#ifndef CONTROL_OL_REF_MAX
    #define CONTROL_OL_REF_MAX (2000.0f)
#endif

/* --- CLOSED LOOP: ref = altura (cm) --- */
#ifndef CONTROL_CL_REF_MIN
    #define CONTROL_CL_REF_MIN (5.0f)    /* cm */
#endif
#ifndef CONTROL_CL_REF_MAX
    #define CONTROL_CL_REF_MAX (80.0f)   /* cm */
#endif

/* =======================
   Descenso hasta altura mínima (usando y = altura cm)
   Reusa control_stop_suave_step()
   ======================= */
#ifndef CONTROL_DESC_TARGET_US
    #define CONTROL_DESC_TARGET_US   (1250.0f)   /* donde tu dron baja solo */
#endif

#ifndef CONTROL_DESC_CUTOFF_US
    #define CONTROL_DESC_CUTOFF_US   (1000.0f)   /* cortar cuando llegó */
#endif

#ifndef CONTROL_DESC_MIN_Y_CM
    #define CONTROL_DESC_MIN_Y_CM    (12.0f)     /* altura mínima (cm) */
#endif

#ifndef CONTROL_DESC_STEP_US
    #define CONTROL_DESC_STEP_US     (2.0f)      /* rampa en microsegundos */
#endif

#ifndef CONTROL_DESC_DELAY_US
    #define CONTROL_DESC_DELAY_US    (20000u)    /* 20ms -> 50Hz */
#endif

#ifndef CONTROL_DESC_MAX_ITERS
    #define CONTROL_DESC_MAX_ITERS   (600u)      /* 600*20ms = 12s */
#endif

#ifndef CONTROL_DESC_MIN_HITS
    #define CONTROL_DESC_MIN_HITS    (2u)        /* N lecturas seguidas <= min */
#endif
#ifndef CONTROL_DESC_HOLD_MS
    #define CONTROL_DESC_HOLD_MS     (3000u)   /* esperar 5 segundos */
#endif

#ifndef CONTROL_DESC_HOLD_POLL_US
    #define CONTROL_DESC_HOLD_POLL_US (20000u) /* chequeo cada 20ms */
#endif

/* =======================
   Stop suave (compat)
   ======================= */
/* Si en algún lado todavía usás estos defines viejos, los dejamos.
   OJO: ahora la lógica real está en los CONTROL_DESC_* de arriba. */
#ifndef CONTROL_STOP_TARGET
    #define CONTROL_STOP_TARGET (1000.0f)
#endif
#ifndef CONTROL_STOP_STEP
    #define CONTROL_STOP_STEP (2.0f)
#endif
#ifndef CONTROL_STOP_TOL_REL
    #define CONTROL_STOP_TOL_REL   (0.05f)
#endif
#ifndef CONTROL_STOP_TOL_ABS
    #define CONTROL_STOP_TOL_ABS   (0.5f)
#endif
#ifndef CONTROL_STOP_DELAY_US
    #define CONTROL_STOP_DELAY_US  (1000u)
#endif
#ifndef CONTROL_STOP_MAX_ITERS
    #define CONTROL_STOP_MAX_ITERS (20000u)
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

/* Ahora hace: rampa hacia 1350 y corta a 1000 cuando y<=min */
bool  control_stop_suave_step(void);

void  control_apply_tf(const float* c, uint16_t n);
void  control_apply_ss(const float* c, uint16_t n);

void  control_step(void);
void  control_force_min(void);

#endif /* CONTROL_APP_H */
