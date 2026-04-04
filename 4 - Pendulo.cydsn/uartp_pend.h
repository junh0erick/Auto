/* ============================================================
   uartp_pend.h — Protocolo UART PSoC ↔ MATLAB (péndulo invertido)
   Proyecto: Péndulo Invertido — PSoC 5LP (control en MATLAB)

   PROTOCOLO COMMAND (antes de iniciar control):
     'r' → 'K'          Reset
     'f' → 'R' → [Fs_inner float32 4B handshake] → 'K'   Set timer
     'i' → 'K'          Iniciar control (entra en CONTROL mode)
     's' → 'K'          Stop (válido en ambos modos)

   PROTOCOLO CONTROL (ciclo de Ts_inner):
     PSoC → MATLAB: [theta int32 LE (4B)][delta_omega int16 LE (2B)] = 6 bytes
     MATLAB → PSoC: [u_pwm int16 LE (2B)] = 2 bytes   (o 's' para stop)
   ============================================================ */
#ifndef UARTP_PEND_H
#define UARTP_PEND_H

#include "project.h"

/* ============================================================
   Modos del sistema
   ============================================================ */
typedef enum {
    UARTP_SYS_COMMAND = 0u,
    UARTP_SYS_CONTROL = 1u
} uartp_sys_mode_t;

extern volatile uartp_sys_mode_t UARTP_SysMode;

/* ============================================================
   API pública
   ============================================================ */

/* Inicializa UART y pone el sistema en COMMAND mode. */
void UARTP_Init(void);

/* Procesa UN comando desde UART (llamar desde main en COMMAND mode). */
void UARTP_ProcessCommand(void);

/* Gestiona bytes RX en modo CONTROL:
   - Si hay 2 bytes de u: actualiza g_last_u_pwm.
   - Si llega 's': aplica Motor_Free(), responde 'K', vuelve a COMMAND. */
void UARTP_ControlRxPoll(void);

#endif /* UARTP_PEND_H */
