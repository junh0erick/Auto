/* uartp_sw.h */
#ifndef UARTP_SW_H
#define UARTP_SW_H

#include "project.h"
#include "cytypes.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/* ===== UART mapping (ajustá si tu componente no se llama UART) ===== */
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
#ifndef UARTP_UART_ReadRxStatus
  #define UARTP_UART_ReadRxStatus()     UART_ReadRxStatus()
#endif
#ifndef UARTP_UART_PutChar
  #define UARTP_UART_PutChar(c)         UART_PutChar((c))
#endif
#ifndef UARTP_UART_PutArray
  #define UARTP_UART_PutArray(p,n)      UART_PutArray((p),(n))
#endif

#if defined(UART_TX_BUFFER_SIZE) && (UART_TX_BUFFER_SIZE > 0)
  #ifndef UARTP_UART_GetTxBufferSize
    #define UARTP_UART_GetTxBufferSize()  UART_GetTxBufferSize()
  #endif
#endif

/* ===== ISR mapping (tu componente ISR se llama isr_rx?) ===== */
#ifndef UARTP_ISR_RX_StartEx
  #define UARTP_ISR_RX_StartEx(f)       isr_rx_StartEx((f))
#endif
#ifndef UARTP_ISR_RX_Enable
  #define UARTP_ISR_RX_Enable()         isr_rx_Enable()
#endif
#ifndef UARTP_ISR_RX_Disable
  #define UARTP_ISR_RX_Disable()        isr_rx_Disable()
#endif

/* ===== sampling callbacks (abstracto) ===== */
typedef void (*uartp_sampling_enable_fn_t)(void);
typedef void (*uartp_sampling_disable_fn_t)(void);

/* AHORA: la librería pasa Fs (Hz) y el MAIN decide cómo convertirlo a Period/ticks */
typedef void (*uartp_sampling_change_fs_fn_t)(float fs_hz);

typedef void (*uartp_sampling_clear_flags_fn_t)(void);

/* ===== Timer UART OneShot para timeouts (sin CyDelay) ===== */
#ifndef UARTP_TIMER_UART_Start
  #define UARTP_TIMER_UART_Start()              timer_uart_Start()
#endif
#ifndef UARTP_TIMER_UART_Stop
  #define UARTP_TIMER_UART_Stop()               timer_uart_Stop()
#endif
#ifndef UARTP_TIMER_UART_WritePeriod
  #define UARTP_TIMER_UART_WritePeriod(p)       timer_uart_WritePeriod((p))
#endif
#ifndef UARTP_TIMER_UART_ReadStatusRegister
  #define UARTP_TIMER_UART_ReadStatusRegister() timer_uart_ReadStatusRegister()
#endif
#ifndef UARTP_TIMER_UART_ReadCounter
  #define UARTP_TIMER_UART_ReadCounter()        timer_uart_ReadCounter()
#endif
#ifndef UARTP_TIMER_UART_STATUS_TC
  #define UARTP_TIMER_UART_STATUS_TC            (0x01u)
#endif

/* ===== comandos ===== */
#define UARTP_CMD_RESET     ((uint8)'r')
#define UARTP_CMD_STOP      ((uint8)'s')
#define UARTP_CMD_INIT      ((uint8)'i')
#define UARTP_CMD_COEFFS    ((uint8)'c')
#define UARTP_CMD_TXCOEF    ((uint8)'t')
#define UARTP_CMD_SETMODE   ((uint8)'m')

/* ===== respuestas ===== */
#define UARTP_RSP_READY_RX  ((uint8)'R')
#define UARTP_RSP_READY_TX  ((uint8)'S')
#define UARTP_RSP_OK        ((uint8)'K')
#define UARTP_RSP_ERR       ((uint8)'!')

/* ===== control por word (eco-confirmado) ===== */
#define UARTP_CTL_ACK       ((uint8)'A')
#define UARTP_CTL_NAK       ((uint8)'N')
#define UARTP_WORD_BYTES    (4u)

/* ===== tamaños fijos ===== */
#define UARTP_COEF_COUNT    (16u)
#define UARTP_COEF_BYTES    (UARTP_COEF_COUNT*4u)
#define UARTP_INIT_BYTES    (4u)
#define UARTP_MODE_BYTES    (4u)

/* ===== robustez ===== */
#define UARTP_STEP_TIMEOUT_MS   (300u)
#define UARTP_ABORT_TIMEOUTS    (6u)
#define UARTP_MAX_WORD_RETRIES  (50u)

/* ===== modos globales ===== */
typedef enum { UARTP_SYS_COMMAND=0, UARTP_SYS_CONTROL=1 } uartp_sysmode_t;
typedef enum { UARTP_CTRL_STOPPED=0, UARTP_CTRL_STOPPING=1, UARTP_CTRL_RUNNING=2 } uartp_ctrl_state_t;

typedef enum {
    UARTP_IMPL_TF            = 0,
    UARTP_IMPL_SS_PRED_NOI   = 1,
    UARTP_IMPL_SS_ACT_NOI    = 2,
    UARTP_IMPL_SS_PRED_I     = 3,
    UARTP_IMPL_SS_ACT_I      = 4,
    UARTP_IMPL_OPENLOOP      = 5
} uartp_impl_t;

/* ===== callbacks ===== */
typedef bool (*uartp_stop_step_fn_t)(void);
typedef void (*uartp_start_fn_t)(float u0);
typedef void (*uartp_apply_tf_fn_t)(const float* c, uint16 n);
typedef void (*uartp_apply_ss_fn_t)(const float* c, uint16 n);
typedef void (*uartp_force_min_fn_t)(void);

typedef struct {
    uartp_stop_step_fn_t stop_step;
    uartp_force_min_fn_t force_min;
    uartp_start_fn_t     start;
    uartp_apply_tf_fn_t  apply_tf;
    uartp_apply_ss_fn_t  apply_ss;

    uartp_sampling_enable_fn_t        sampling_enable;
    uartp_sampling_disable_fn_t       sampling_disable;
    uartp_sampling_change_fs_fn_t     sampling_change_fs;     /* <- Fs (Hz) */
    uartp_sampling_clear_flags_fn_t   sampling_clear_flags;   /* puede ser NULL */
} uartp_cfg_t;

/* ===== globals (extern) ===== */
extern volatile uartp_sysmode_t     UARTP_SysMode;
extern volatile uartp_ctrl_state_t  UARTP_CtrlState;

extern volatile uartp_impl_t        UARTP_Impl;
extern volatile uint8               UARTP_ModeValid;
extern volatile uint8               UARTP_CoeffsValid;

extern volatile uint8               UARTP_StopRequested;
extern volatile uint8               UARTP_LastCmd;

extern float                        UARTP_Coeffs[UARTP_COEF_COUNT];
extern float                        UARTP_InitValue;

/* c15 = N (float->u16), c16 = FsHz (float32) */
extern volatile uint16              UARTP_StreamN;
extern float                        UARTP_StreamFsHz;

/* ===== API ===== */
void UARTP_Init(const uartp_cfg_t* cfg);
bool UARTP_ProcessOnce(void);
void UARTP_ControlTick(void);
void UARTP_EnterCommandMode(void);
void UARTP_EnterControlMode(void);

/* ===== Telemetry streaming (u,y) ===== */
void UARTP_Telemetry_Reset(void);
void UARTP_Telemetry_Push(float u, float y);

#endif
