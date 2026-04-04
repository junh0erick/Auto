#ifndef UARTP_SW_H
#define UARTP_SW_H

#include "project.h"
#include <stdint.h>
#include <stdbool.h>

/* ================= UART mapping ================= */
#ifndef UARTP_UART_Start
#define UARTP_UART_Start()            UART_Start()
#endif
#ifndef UARTP_UART_ClearRxBuffer
#define UARTP_UART_ClearRxBuffer()    UART_ClearRxBuffer()
#endif
#ifndef UARTP_UART_ClearTxBuffer
#define UARTP_UART_ClearTxBuffer()    UART_ClearTxBuffer()
#endif
#ifndef UARTP_UART_GetRxBufferSize
#define UARTP_UART_GetRxBufferSize()  UART_GetRxBufferSize()
#endif
#ifndef UARTP_UART_ReadRxData
#define UARTP_UART_ReadRxData()       UART_ReadRxData()
#endif
#ifndef UARTP_UART_PutChar
#define UARTP_UART_PutChar(c)         UART_PutChar((c))
#endif
#ifndef UARTP_UART_PutArray
#define UARTP_UART_PutArray(p,n)      UART_PutArray((p),(n))
#endif

/* ================= ISR mapping ================= */
#ifndef UARTP_ISR_RX_StartEx
#define UARTP_ISR_RX_StartEx(f)       isr_rx_StartEx((f))
#endif
#ifndef UARTP_ISR_RX_Enable
#define UARTP_ISR_RX_Enable()         isr_rx_Enable()
#endif
#ifndef UARTP_ISR_RX_Disable
#define UARTP_ISR_RX_Disable()        isr_rx_Disable()
#endif

/* ================= Protocol ================= */
#define UARTP_CMD_STOP     ((uint8_t)'s')
#define UARTP_CMD_SETMODE  ((uint8_t)'m')
#define UARTP_CMD_INIT     ((uint8_t)'i')
#define UARTP_CMD_COEFFS   ((uint8_t)'c')
#define UARTP_CMD_PERIOD  ((uint8_t)'p')

#define UARTP_RSP_READY_RX ((uint8_t)'R')
#define UARTP_RSP_OK       ((uint8_t)'K')
#define UARTP_RSP_ERR      ((uint8_t)'!')

/* ================= Sizes ================= */
#define UARTP_COEF_COUNT   (16u)
#define UARTP_COEF_BYTES   (UARTP_COEF_COUNT * 4u)

/* ================= Modes ================= */
typedef enum {
    UARTP_SYS_COMMAND = 0,
    UARTP_SYS_CONTROL = 1
} uartp_sysmode_t;

typedef enum {
    UARTP_CTRL_STOPPED  = 0,
    UARTP_CTRL_STOPPING = 1,
    UARTP_CTRL_RUNNING  = 2
} uartp_ctrl_state_t;

typedef enum {
    UARTP_IMPL_TF = 0,
    UARTP_IMPL_SS_PRED_NOI = 1,
    UARTP_IMPL_SS_ACT_NOI  = 2,
    UARTP_IMPL_SS_PRED_I   = 3,
    UARTP_IMPL_SS_ACT_I    = 4,
    UARTP_IMPL_OPENLOOP    = 5
} uartp_impl_t;

/* ================= Callbacks ================= */
typedef bool (*uartp_stop_step_fn_t)(void);
typedef void (*uartp_start_fn_t)(float u0);
typedef void (*uartp_apply_fn_t)(const float* c, uint16_t n);

typedef struct {
    uartp_stop_step_fn_t stop_step;
    uartp_start_fn_t     start;
    uartp_apply_fn_t     apply_tf;
    uartp_apply_fn_t     apply_ss;
} uartp_cfg_t;

/* ================= Globals ================= */
extern volatile uartp_sysmode_t    UARTP_SysMode;
extern volatile uartp_ctrl_state_t UARTP_CtrlState;
extern volatile uartp_impl_t       UARTP_Impl;

extern volatile uint8_t UARTP_StopRequested;

/* streaming / decimation */
extern volatile uint8_t UARTP_N;
extern volatile uint8_t UARTP_Ncnt;
extern volatile uint8_t UARTP_FlagSendU;
extern volatile uint8_t UARTP_FlagSendY;

extern float UARTP_Coeffs[UARTP_COEF_COUNT];

/* ================= API ================= */
void UARTP_Init(const uartp_cfg_t* cfg);
bool UARTP_ProcessOnce(void);
void UARTP_ControlTick(void);
void UARTP_OnSampleIsr(void);

#endif /* UARTP_SW_H */
