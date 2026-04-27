/* ============================================================
   uartp_pend.h — Protocolo UART PSoC ↔ MATLAB (péndulo invertido) v6
   Proyecto: Péndulo Invertido — PSoC 5LP (control en PSoC)

   PROTOCOLO COMMAND (antes de iniciar control):
     'r' → 'K'          Reset
     'f' → 'R' → [Fs_inner float32 4B handshake] → 'K'   Set timer (compat)
     'p' → [len 2B LE] → [216B payload] → [xsum 1B] → 'K'/'E'
                         Set all controller params (modes, coeffs, ref, sat)
     'v' → [216B echo] → [xsum 1B]
                         Eco del payload almacenado (verificación post-'p')
     'i' → 'K'          Iniciar STREAM mode
     's' → 'K'          Stop (válido en ambos modos)

   PROTOCOLO STREAM (PSoC-driven, v6):
     PSoC → MATLAB: [y1 f32 4B][y2 f32 4B][u1 f32 4B][u2 f32 4B] = 16B cada Ts
     MATLAB → PSoC: 'u' + [ref_inner f32 4B]  → actualizar referencia inner
                    's'                          → stop + 'K' + vuelve a COMMAND

   PAYLOAD 'p' (216 bytes + 1 checksum XOR):
     [0]       mode_inner  uint8 (CTRL_MODE_*)
     [1]       mode_outer  uint8 (CTRL_MODE_*)
     [2]       num_type    uint8 (CTRL_NUM_F32=0, F64=1, F16=2, Q31=3, Q15=4, Q7=5)
     [3]       flags   uint8  bitmask:
                         bit0=1 → ref en Voltios: PSoC convierte ref V→PWM
                         bit1=1 → salida en Voltios: PSoC aplica PWM_Desde_Voltaje(u) antes de Motor_Control
                         (0x02 = ref rad/s + salida V; 0x01 = ref V + salida PWM; 0x00 = todo PWM)
     [4-103]   coeffs_inner[25] float32 LE  (TF o SS segun modo)
     [104-203] coeffs_outer[25] float32 LE
     [204-207] ref_inner   float32 LE
     [208-211] sat_min     float32 LE
     [212-215] sat_max     float32 LE

   ISR RX (isr_2):
     En STREAM mode: isr_2 llama UARTP_Rx_ISR() al llegar cada byte.
     UARTP_Rx_ISR maneja 's' (stop inmediato) y 'u'+4B (live ref update).
     En COMMAND mode: isr_2 no actúa; main loop usa UARTP_ProcessCommand().
   ============================================================ */
#ifndef UARTP_PEND_H
#define UARTP_PEND_H

#include "project.h"

/* ============================================================
   Modos del sistema
   ============================================================ */
typedef enum {
    UARTP_SYS_COMMAND = 0u,
    UARTP_SYS_CONTROL = 1u   /* = STREAM mode en v6 */
} uartp_sys_mode_t;

extern volatile uartp_sys_mode_t UARTP_SysMode;

/* Referencia inner recibida via 'p', para pasar a ctrl_start() */
extern volatile float UARTP_PendingRef;

/* Flags byte[3] del payload 'p': bit0=ref en V→PWM, bit1=salida en V→PWM */
extern volatile uint8 g_ref_in_volts;

/* ============================================================
   API pública
   ============================================================ */

/* Inicializa UART y pone el sistema en COMMAND mode. */
void UARTP_Init(void);

/* Procesa UN comando desde UART (llamar desde main en COMMAND mode). */
void UARTP_ProcessCommand(void);

/* Gestiona bytes RX en modo STREAM (CONTROL).
   NOTA v6.2: reemplazado por isr_2/UARTP_Rx_ISR.
   Se mantiene como fallback si isr_2 no está disponible. */
void UARTP_ControlRxPoll(void);

/* ISR del receptor UART — conectar con isr_2_StartEx(UARTP_Rx_ISR).
   Actúa solo en STREAM mode: maneja 's' y 'u'+4B inmediatamente.
   En COMMAND mode: no consume bytes (los deja para UARTP_ProcessCommand). */
CY_ISR_PROTO(UARTP_Rx_ISR);

#endif /* UARTP_PEND_H */
