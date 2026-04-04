/* ============================================================
   ctrl_pend.h — Two-plant on-PSoC controller
   Plant 0 (inner): u_pwm  → ω_motor   [rad/s]
   Plant 1 (outer): u_ref  → θ_pendulum [rad]

   Coefficient layout (25 floats, 0-indexed):
     TF:  c[0..5]=b,  c[6..11]=a,  c[14]=N, c[15]=Fs_hz
     SS:  c[0..3]=A(2x2 row-major), c[4..5]=B, c[6..7]=C,
          c[8]=D, c[9..10]=L, c[11..12]=K, c[13]=Kx,
          c[14]=N, c[15]=Fs_hz
     OL/OFF: only c[14]=N, c[15]=Fs_hz matter
   ============================================================ */
#ifndef CTRL_PEND_H
#define CTRL_PEND_H

#include "project.h"

/* ---- Plant indices ---- */
#define PLANT_INNER   0u
#define PLANT_OUTER   1u
#define PLANT_COUNT   2u

/* ---- Controller modes (must match MATLAB GUI) ---- */
#define CTRL_MODE_TF          0u
#define CTRL_MODE_SS_PRED_NOI 1u
#define CTRL_MODE_SS_ACT_NOI  2u
#define CTRL_MODE_SS_PRED_I   3u
#define CTRL_MODE_SS_ACT_I    4u
#define CTRL_MODE_OPEN_LOOP   5u
#define CTRL_MODE_OFF         6u

/* ---- Payload sizes ---- */
#define CTRL_COEF_COUNT  25u
#define CTRL_COEF_BYTES  100u   /* 25 * sizeof(float) */

/* ---- Telemetry (written in ctrl_step, read in main) ---- */
extern volatile float ctrl_telem_u1;
extern volatile float ctrl_telem_y1;
extern volatile float ctrl_telem_u2;
extern volatile float ctrl_telem_y2;
extern volatile uint8 ctrl_telem_ready;

/* ---- Timer period update (apply in main loop) ---- */
extern volatile uint8  ctrl_period_pending;
extern volatile uint32 ctrl_period_ticks;   /* = desired_period - 1 */

/* ---- API ---- */
void ctrl_init(void);
void ctrl_set_mode(uint8 plant_id, uint8 mode);
void ctrl_apply_coeffs(uint8 plant_id, const float* c, uint16 n);
void ctrl_start(float ref_inner);       /* ref_inner: initial ω ref or OL u */
void ctrl_stop(void);
void ctrl_step(void);                   /* call once per g_flag_control tick  */
void ctrl_update_ref(float ref_inner);  /* live inner reference update (CTRL mode) */

#endif /* CTRL_PEND_H */
