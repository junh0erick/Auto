/* ============================================================
   ctrl_pend.h — Two-plant on-PSoC controller  v6.3
   CMSIS-DSP f32 + Q31/Q15/Q7 paths (arm_dot_prod, arm_scale, arm_add)

   Coeficientes (25 floats, 0-indexed):
     TF:  c[0..5]=b,  c[6..11]=a,  c[14]=N, c[15]=Fs_hz,
          c[17]=Nbar   (u += Nbar*ref;  0=desactivado)
          c[18]=offset (u += offset;    0=desactivado)
          ley completa: u = Nbar*ref + C(z)*(ref-y) + offset
     SS:  c[0..3]=A(2x2 row-major), c[4..5]=B, c[6..7]=C,
          c[8]=D, c[9..10]=L, c[11..12]=K, c[13]=Kx,
          c[14]=N, c[15]=Fs_hz, c[16]=q_scale (0=default 1.0)
     OL/OFF: solo c[14]=N, c[15]=Fs_hz importan
     c[17..18]: Nbar y offset — solo activos en modo TF

   q_scale (c[16]): normaliza señales a [-1,1) para Q-format.
     Las señales físicas se dividen por q_scale antes de convertir a Q.
     Los coeficientes (A,B,C,D,K,L,Kx) deben tener |valor| < 1 para
     que el path Q funcione sin saturación. Ver nota en ctrl_pend.c.
   ============================================================ */
#ifndef CTRL_PEND_H
#define CTRL_PEND_H

#include "project.h"

/* ---- Índices de planta ---- */
#define PLANT_INNER   0u
#define PLANT_OUTER   1u
#define PLANT_COUNT   2u

/* ---- Modos de controlador (deben coincidir con MATLAB GUI) ---- */
#define CTRL_MODE_TF          0u
#define CTRL_MODE_SS_PRED_NOI 1u
#define CTRL_MODE_SS_ACT_NOI  2u
#define CTRL_MODE_SS_PRED_I   3u
#define CTRL_MODE_SS_ACT_I    4u
#define CTRL_MODE_OPEN_LOOP   5u
#define CTRL_MODE_OFF         6u

/* ---- Tipo numérico (payload 'p' byte[2], MATLAB dropdown) ---- */
#define CTRL_NUM_F32  0u    /* float32 — default, CMSIS-DSP f32 */
#define CTRL_NUM_F64  1u    /* double  — software (fallback a f32 en PSoC) */
#define CTRL_NUM_F16  2u    /* float16 — no soportado CM3 (fallback a f32) */
#define CTRL_NUM_Q31  3u    /* Q31 fixed-point — implementado, observer Q31 */
#define CTRL_NUM_Q15  4u    /* Q15 fixed-point — implementado, observer Q15 */
#define CTRL_NUM_Q7   5u    /* Q7  fixed-point — implementado, baja precisión */

/* ---- Tamaño del payload de coeficientes ---- */
#define CTRL_COEF_COUNT  25u
#define CTRL_COEF_BYTES  100u   /* 25 * sizeof(float) */

/* ---- Telemetría (escritas en ctrl_step, leídas en main) ---- */
extern volatile float ctrl_telem_u1;
extern volatile float ctrl_telem_y1;
extern volatile float ctrl_telem_u2;
extern volatile float ctrl_telem_y2;
extern volatile float ctrl_telem_x1i;   /* inner xhat[0] */
extern volatile float ctrl_telem_x2i;   /* inner xhat[1] */
extern volatile float ctrl_telem_x1o;   /* outer xhat[0] */
extern volatile float ctrl_telem_x2o;   /* outer xhat[1] */
extern volatile uint8 ctrl_telem_ready;

/* ---- Timer (aplicar en main loop, no desde ISR) ---- */
extern volatile uint8  ctrl_period_pending;
extern volatile uint32 ctrl_period_ticks;

/* ---- API ---- */
void  ctrl_init(void);
void  ctrl_set_mode(uint8 plant_id, uint8 mode);
void  ctrl_apply_coeffs(uint8 plant_id, const float* c, uint16 n);
void  ctrl_start(float ref_inner);
void  ctrl_stop(void);
void  ctrl_step(void);
void  ctrl_update_ref(float ref_inner);
void  ctrl_set_sat(float sat_min, float sat_max);
void  ctrl_set_num_type(uint8 num_type);
uint8 ctrl_get_num_type(void);

#endif /* CTRL_PEND_H */
