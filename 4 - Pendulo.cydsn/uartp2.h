/* ============================================================
   uartp2.h — Two-plant UARTP protocol
   Commands (COMMAND mode):
     'r' → reset/stop          → PSoC: 'K'
     '1' → set mode plant 0    → PSoC: 'R' → host: 4B  → PSoC: 'K'
     '2' → set mode plant 1    → PSoC: 'R' → host: 4B  → PSoC: 'K'
     'a' → coeffs plant 0      → PSoC: 'R' → host: 100B → PSoC: 'K'
     'b' → coeffs plant 1      → PSoC: 'R' → host: 100B → PSoC: 'K'
     'i' → start+ref           → PSoC: 'R' → host: 4B  → PSoC: 'K'
            (enters CONTROL mode)
     's' → stop                → PSoC: 'K'  (also valid in CONTROL)
   Telemetry (CONTROL mode):
     16 bytes/tick: [u1_f32_LE][y1_f32_LE][u2_f32_LE][y2_f32_LE]
   Handshake (payload words, same as helicopterovertical):
     For each 4-byte word:
       PSoC receives 4B, echoes 4B, waits ACK/NAK from host,
       sends ACK if host sent ACK, NAK otherwise.
   ============================================================ */
#ifndef UARTP2_H
#define UARTP2_H

#include "project.h"

typedef enum { UARTP2_CMD = 0, UARTP2_CTRL } uartp2_mode_t;
extern volatile uartp2_mode_t UARTP2_Mode;

void UARTP2_Init(void);
void UARTP2_ProcessOnce(void);  /* call from COMMAND loop */

#endif /* UARTP2_H */
