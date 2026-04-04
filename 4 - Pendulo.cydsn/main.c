/* ============================================================
   main.c — Péndulo Invertido PSoC 5LP (on-PSoC two-plant control)
   CTRL-mode commands: 's' stop, 'f' live inner reference update (4B raw float)

   Architecture:
     PSoC reads encoders, runs TF/SS/OL controllers for both
     plants, drives motor, and streams telemetry to MATLAB.
     MATLAB configures controllers via UARTP2 protocol.

   Plants:
     Plant 0 (inner): u_pwm → ω_motor   [rad/s]   (QuadDec_1)
     Plant 1 (outer): u_ref → θ_pendulum [rad]      (QuadDec_2)

   Protocol: see uartp2.h
   Telemetry: 16 B/tick [y2_f32 u1_f32 y1_f32 u2_f32]
   ============================================================ */
#include "project.h"
#include "pendulo.h"
#include "motor.h"
#include "ctrl_pend.h"
#include "uartp2.h"

int main(void)
{
    CyGlobalIntEnable;

    pendulo_init();
    ctrl_init();
    UARTP2_Init();

    for (;;)
    {
        /* ---- Apply pending timer period (set by ctrl_apply_coeffs) ---- */
        if (ctrl_period_pending) {
            Timer_1_WritePeriod(ctrl_period_ticks);
            ctrl_period_pending = 0u;
        }

        switch (UARTP2_Mode)
        {
            /* ================================================
               COMMAND mode: wait for configuration commands
               ================================================ */
            case UARTP2_CMD:
                UARTP2_ProcessOnce();
                break;

            /* ================================================
               CONTROL mode: run controller + stream telemetry
               ================================================ */
            case UARTP2_CTRL:
                /* Control tick */
                if (g_flag_control) {
                    g_flag_control = 0u;
                    ctrl_step();
                }

                /* Stream telemetry when ready */
                if (ctrl_telem_ready) {
                    float u1, y1, u2, y2;
                    uint8 intr = CyEnterCriticalSection();
                    u1 = ctrl_telem_u1;
                    y1 = ctrl_telem_y1;
                    u2 = ctrl_telem_u2;
                    y2 = ctrl_telem_y2;
                    ctrl_telem_ready = 0u;
                    CyExitCriticalSection(intr);

                    /* Frame order: [y2, u1, y1, u2] — matches MATLAB parseFrame */
                    UART_1_PutArray((uint8*)&y2, 4u);
                    UART_1_PutArray((uint8*)&u1, 4u);
                    UART_1_PutArray((uint8*)&y1, 4u);
                    UART_1_PutArray((uint8*)&u2, 4u);
                }

                /* Check for commands while in CONTROL */
                if (UART_1_GetRxBufferSize() > 0u) {
                    uint8 b = UART_1_ReadRxData();
                    if (b == (uint8)'s') {
                        ctrl_stop();
                        UARTP2_Mode = UARTP2_CMD;
                        UART_1_WriteTxData((uint8)'K');
                    } else if (b == (uint8)'f') {
                        /* Live reference update: 4 raw bytes, no echo-confirm.
                           At 115200 baud all 4 bytes arrive within ~1ms. */
                        uint8 raw[4]; uint8 i; uint32 t; uint8 ok = 1u;
                        for (i = 0u; i < 4u; i++) {
                            t = 0u;
                            while (UART_1_GetRxBufferSize() == 0u) {
                                CyDelay(1u);
                                if (++t >= 100u) { ok = 0u; break; }
                            }
                            if (!ok) break;
                            raw[i] = UART_1_ReadRxData();
                        }
                        if (ok) {
                            uint8 *p; float ref_new;
                            p = (uint8*)&ref_new;
                            p[0]=raw[0]; p[1]=raw[1]; p[2]=raw[2]; p[3]=raw[3];
                            ctrl_update_ref(ref_new);
                            UART_1_WriteTxData((uint8)'K');
                        }
                    }
                    /* ignore other bytes (stale data) */
                }
                break;

            /* ================================================
               Safe default: back to COMMAND
               ================================================ */
            default:
                ctrl_stop();
                UARTP2_Mode = UARTP2_CMD;
                break;
        }
    }
}
