#include "project.h"
#include "uartp_sw.h"
#include "control_app.h"
extern float u_k;
int main(void)
{
    CyGlobalIntEnable;

    uartp_cfg_t cfg = {
        .stop_step = control_stop_suave_step,
        .start     = control_start,
        .apply_tf  = control_apply_tf,
        .apply_ss  = control_apply_ss
    };

    UARTP_Init(&cfg);

    for (;;)
    {
        switch (UARTP_SysMode)
        {
            case UARTP_SYS_COMMAND:
                (void)UARTP_ProcessOnce();
                break;

            case UARTP_SYS_CONTROL:
                control_step();
                UARTP_ControlTick();
                if (UARTP_FlagSendU) {
                    UARTP_UART_PutArray((uint8*)&u_k, 4);
                    UARTP_FlagSendU = 0;
                    }

                    if (UARTP_FlagSendY) {
                    UARTP_UART_PutArray((uint8*)&control_last_y, 4);
                    UARTP_FlagSendY = 0;
                }
                    UARTP_ControlTick();

                break;

            default:
                UARTP_EnterCommandMode();
                break;
        }
        
        
    }
}
