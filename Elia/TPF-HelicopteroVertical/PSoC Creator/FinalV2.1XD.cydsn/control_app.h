/* =========================
   control_app.h
   ========================= */
#ifndef CONTROL_APP_H
#define CONTROL_APP_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================
   1) ACTUADOR (PWM absoluto que sale por write_u callback)
   ========================================================= */
#ifndef CONTROL_SAT_MIN
    #define CONTROL_SAT_MIN (1000.0f)   /* us */
#endif

#ifndef CONTROL_SAT_MAX
    #define CONTROL_SAT_MAX (1750.0f)   /* us */
#endif

/* Offset default (fallback). El real se auto-calibra en runtime (g_u0_offset_us) */
#ifndef CONTROL_U_OFFSET
    #define CONTROL_U_OFFSET (1350.0f)  /* us */
#endif


/* =========================================================
   2) REFERENCIAS (según modo)
   =========================================================
   - OPEN LOOP:
       si CONTROL_OL_REF_IS_DELTA=1 -> ref = Δu [us] relativo a u0
       si CONTROL_OL_REF_IS_DELTA=0 -> ref = PWM absoluto [us]
   - CLOSED LOOP:
       ref = altura física [cm]
   ========================================================= */
#ifndef CONTROL_OL_REF_IS_DELTA
    #define CONTROL_OL_REF_IS_DELTA (1u)
#endif

/* Usados SOLO si CONTROL_OL_REF_IS_DELTA = 0 (PWM absoluto) */
#ifndef CONTROL_OL_REF_MIN
    #define CONTROL_OL_REF_MIN (1000.0f) /* us */
#endif
#ifndef CONTROL_OL_REF_MAX
    #define CONTROL_OL_REF_MAX (1700.0f) /* us */
#endif

/* Closed-loop (altura física) */
#ifndef CONTROL_CL_REF_MIN
    #define CONTROL_CL_REF_MIN (12.0f)   /* cm */
#endif
#ifndef CONTROL_CL_REF_MAX
    #define CONTROL_CL_REF_MAX (300.0f)  /* cm */
#endif


/* =========================================================
   3) STOP SUAVE (descenso controlado)
   =========================================================
   - TARGET es Δu relativo (cmd), NO PWM absoluto.
   - CUTOFF es PWM absoluto final de corte.
   ========================================================= */
#ifndef CONTROL_DESC_TARGET_CMD_US
    #define CONTROL_DESC_TARGET_CMD_US   (-50.0f) /* Δu [us] relativo a u0 */
#endif

#ifndef CONTROL_DESC_CUTOFF_US
    #define CONTROL_DESC_CUTOFF_US       (1000.0f) /* PWM absoluto [us] */
#endif

#ifndef CONTROL_DESC_MIN_Y_CM
    #define CONTROL_DESC_MIN_Y_CM        (12.0f)   /* cm */
#endif

#ifndef CONTROL_DESC_STEP_US
    #define CONTROL_DESC_STEP_US         (5.0f)    /* Δu [us] */
#endif

#ifndef CONTROL_DESC_DELAY_US
    #define CONTROL_DESC_DELAY_US        (20000u)  /* 20ms -> 50Hz */
#endif

#ifndef CONTROL_DESC_MAX_ITERS
    #define CONTROL_DESC_MAX_ITERS       (600u)    /* 600*20ms = 12s */
#endif

#ifndef CONTROL_DESC_MIN_HITS
    #define CONTROL_DESC_MIN_HITS        (2u)
#endif

#ifndef CONTROL_DESC_HOLD_MS
    #define CONTROL_DESC_HOLD_MS         (500u)    /* ms */
#endif

#ifndef CONTROL_DESC_HOLD_POLL_US
    #define CONTROL_DESC_HOLD_POLL_US    (20000u)  /* us */
#endif


/* =========================================================
   4) AUTO-CALIBRACIÓN u0 (RÁPIDA)
   =========================================================
   Objetivo: detectar el primer PWM (u_phy) donde hay movimiento real.

   Nota: estos parámetros suponen fs_calib = 1000 Hz.
   Si fs cambia, escalá HOLD/MIN_HITS/MAX_SAMPLES proporcionalmente.
   ========================================================= */
#ifndef CONTROL_ENABLE_AUTOCAL
    #define CONTROL_ENABLE_AUTOCAL (1u)
#endif

/* Barrido PWM absoluto durante la rampa */
#ifndef CONTROL_CALIB_START_US
    #define CONTROL_CALIB_START_US (1250.0f) /* us */
#endif

#ifndef CONTROL_CALIB_STEP_US
    #define CONTROL_CALIB_STEP_US  (5.0f)    /* us por nivel (rápido) */
#endif

#ifndef CONTROL_CALIB_MAX_US
    #define CONTROL_CALIB_MAX_US   (1500.0f) /* us */
#endif

/* Nivel: cuántas muestras se mantiene cada PWM antes de subir */
#ifndef CONTROL_CALIB_HOLD_SAMPLES
    #define CONTROL_CALIB_HOLD_SAMPLES (200u) /* 120 ms por nivel (rápido pero usable) */
#endif

/* Detección de movimiento: |y - y0| >= DY durante MIN_HITS consecutivos */
#ifndef CONTROL_CALIB_DY_CM
    #define CONTROL_CALIB_DY_CM    (2.0f)    /* cm (rápido) */
#endif

#ifndef CONTROL_CALIB_MIN_HITS
    #define CONTROL_CALIB_MIN_HITS (20u)     /* 20 ms consecutivos */
#endif

/* Tiempo máximo total de calibración (en muestras) */
#ifndef CONTROL_CALIB_MAX_SAMPLES
    #define CONTROL_CALIB_MAX_SAMPLES (12000u) /* 12 s (rápido, pero no se corta enseguida) */
#endif

/* Post-move: mantener u0 detectado un rato y luego soltar a Δu=0 */
#ifndef CONTROL_CALIB_POSTMOVE_MS
    #define CONTROL_CALIB_POSTMOVE_MS (500u) /* ms */
#endif


/* =========================================================
   5) ESTABILIZACIÓN POST-CALIB (SETTLE)
   =========================================================
   Dy pequeño por N muestras => estable.
   ========================================================= */
#ifndef CONTROL_U0_SETTLE_SAMPLES
    #define CONTROL_U0_SETTLE_SAMPLES (200u) /* 200 ms estable */
#endif

#ifndef CONTROL_U0_SETTLE_DY_CM
    #define CONTROL_U0_SETTLE_DY_CM   (0.4f) /* cm */
#endif


/* =========================================================
   6) TF (IIR) - orden máximo soportado por el paquete TF
   ========================================================= */
#ifndef CONTROL_TF_MAX_ORDER
    #define CONTROL_TF_MAX_ORDER (10u) /* b0..b10 / a0..a10 */
#endif


/* =========================================================
   7) Callbacks IO
   ========================================================= */
typedef void (*control_write_u_fn_t)(float u_phy);

/* Corre DENTRO del ISR de muestreo.
   Debe ser ultraliviano y terminar llamando:
     control_sample_isr_push(y);
*/
typedef void (*control_sample_isr_fn_t)(void);


/* =========================================================
   8) Variables ISR -> main
   ========================================================= */
extern volatile float   control_last_y;
extern volatile uint8_t control_sample_pending;
extern volatile float   control_ref_v;


/* =========================================================
   9) API
   ========================================================= */
void control_register_io(control_sample_isr_fn_t sample_isr,
                         control_write_u_fn_t    write_u);

void control_on_sample_isr(void);
void control_sample_isr_push(float y);

void  control_set_reference(float r);
float control_get_reference(void);

void  control_set_sample_time(float Ts);
float control_get_sample_time(void);

/* START: inicia control + dispara autocalibración (si enabled) */
void  control_start(float ref0);

/* Stop suave: rampa hacia target y corta */
bool  control_stop_suave_step(void);

/* PAQUETE FIJO 25 floats:
   TF:
     c[0..10]  = b0..b10
     c[11..21] = a0..a10
     c[22]     = order (0..10)
     c[23]     = N
     c[24]     = FsHz
   SS:
     c[0..22]  = matrices/ganancias
     c[23]     = N
     c[24]     = FsHz
*/
void  control_apply_tf(const float* c, uint16_t n);
void  control_apply_ss(const float* c, uint16_t n);

void  control_step(void);
void  control_force_min(void);

/* Debug: leer u0 runtime */
float control_get_u0_offset_us(void);

/* Estado de autocal u0 */
bool control_is_calibrating(void);
bool control_u0_is_valid(void);

#endif /* CONTROL_APP_H */
