#include "uartp_sw.h"
#include <string.h>

/* ================= Globals ================= */
volatile uartp_sysmode_t    UARTP_SysMode   = UARTP_SYS_COMMAND;
volatile uartp_ctrl_state_t UARTP_CtrlState = UARTP_CTRL_STOPPED;
volatile uartp_impl_t       UARTP_Impl      = UARTP_IMPL_TF;

volatile uint8_t UARTP_StopRequested = 0u;

/* streaming */
volatile uint8_t UARTP_N = 0u;
volatile uint8_t UARTP_Ncnt = 0u;
volatile uint8_t UARTP_FlagSendU = 0u;
volatile uint8_t UARTP_FlagSendY = 0u;

float UARTP_Coeffs[UARTP_COEF_COUNT];

static uartp_cfg_t g_cfg;

/* ================= Helpers ================= */
static void flush_rx(void)
{
    while (UARTP_UART_GetRxBufferSize() > 0u)
        (void)UARTP_UART_ReadRxData();
}

static bool read_n_bytes(uint8_t* dst, uint8_t n)
{
    uint16_t timeout = 400u;
    uint8_t i = 0u;

    while (i < n && timeout--) {
        if (UARTP_UART_GetRxBufferSize() > 0u)
            dst[i++] = UARTP_UART_ReadRxData();
        CyDelay(1u);
    }
    return (i == n);
}

/* ================= RX ISR (SOLO STOP) ================= */
CY_ISR(UARTP_RxIsr)
{
    if (UARTP_SysMode != UARTP_SYS_CONTROL)
        return;

    while (UARTP_UART_GetRxBufferSize() > 0u) {
        uint8_t b = UARTP_UART_ReadRxData();
        if (b == UARTP_CMD_STOP) {
            UARTP_StopRequested = 1u;
            UARTP_CtrlState = UARTP_CTRL_STOPPING;
            UARTP_UART_PutChar(UARTP_RSP_OK);
        }
    }
}

/* ================= Sampling hook ================= */
void UARTP_OnSampleIsr(void)
{
    if (UARTP_SysMode != UARTP_SYS_CONTROL)
        return;

    if (UARTP_N == 0u)
        return;

    UARTP_Ncnt++;
    if (UARTP_Ncnt >= UARTP_N) {
        UARTP_Ncnt = 0u;
        UARTP_FlagSendU = 1u;
        UARTP_FlagSendY = 1u;
    }
}

/* ================= Init ================= */
void UARTP_Init(const uartp_cfg_t* cfg)
{
    UARTP_UART_Start();
    UARTP_UART_ClearRxBuffer();
    UARTP_UART_ClearTxBuffer();
    flush_rx();

    if (cfg) g_cfg = *cfg;

    UARTP_SysMode   = UARTP_SYS_COMMAND;
    UARTP_CtrlState = UARTP_CTRL_STOPPED;
    UARTP_StopRequested = 0u;

    UARTP_N = 0u;
    UARTP_Ncnt = 0u;
    UARTP_FlagSendU = 0u;
    UARTP_FlagSendY = 0u;

    memset(UARTP_Coeffs, 0, sizeof(UARTP_Coeffs));

    UARTP_ISR_RX_StartEx(UARTP_RxIsr);
    UARTP_ISR_RX_Disable();   /* 🔒 clave */
}

/* ================= COMMAND mode ================= */
bool UARTP_ProcessOnce(void)
{
    if (UARTP_SysMode != UARTP_SYS_COMMAND)
        return false;

    if (UARTP_UART_GetRxBufferSize() == 0u)
        return false;

    uint8_t cmd = UARTP_UART_ReadRxData();

    switch (cmd)
    {
        case UARTP_CMD_SETMODE:
        {
            uint8_t raw[4];
            UARTP_UART_PutChar(UARTP_RSP_READY_RX);
            if (!read_n_bytes(raw, 4)) {
                UARTP_UART_PutChar(UARTP_RSP_ERR);
                return true;
            }

            UARTP_Impl = (uartp_impl_t)raw[0];
            UARTP_N    = raw[1];
            UARTP_Ncnt = 0u;

            UARTP_UART_PutChar(UARTP_RSP_OK);
            return true;
        }

        case UARTP_CMD_COEFFS:
        {
            UARTP_UART_PutChar(UARTP_RSP_READY_RX);
            if (!read_n_bytes((uint8_t*)UARTP_Coeffs, UARTP_COEF_BYTES)) {
                UARTP_UART_PutChar(UARTP_RSP_ERR);
                return true;
            }

            if (UARTP_Impl == UARTP_IMPL_TF) {
                if (g_cfg.apply_tf) g_cfg.apply_tf(UARTP_Coeffs, UARTP_COEF_COUNT);
            } else {
                if (g_cfg.apply_ss) g_cfg.apply_ss(UARTP_Coeffs, UARTP_COEF_COUNT);
            }

            UARTP_UART_PutChar(UARTP_RSP_OK);
            return true;
        }

        case UARTP_CMD_INIT:
        {
            union { uint8_t b[4]; float f; } u;
            UARTP_UART_PutChar(UARTP_RSP_READY_RX);
            if (!read_n_bytes(u.b, 4)) {
                UARTP_UART_PutChar(UARTP_RSP_ERR);
                return true;
            }

            if (g_cfg.start) g_cfg.start(u.f);

            UARTP_SysMode   = UARTP_SYS_CONTROL;
            UARTP_CtrlState = UARTP_CTRL_RUNNING;

            UARTP_ISR_RX_Enable();   /* 🔓 ahora sí */

            UARTP_UART_PutChar(UARTP_RSP_OK);
            return true;
        }

        case UARTP_CMD_PERIOD:
        {
            uint8_t raw[2];
            if (!read_n_bytes(raw, 2)) {
                UARTP_UART_PutChar(UARTP_RSP_ERR);
                return true;
            }

            uint16_t period = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
            if (period == 0u) {
                UARTP_UART_PutChar(UARTP_RSP_ERR);
                return true;
            }

            timer_sampling_WritePeriod(period - 1u);
            UARTP_UART_PutChar(UARTP_RSP_OK);
            return true;
        }

        case UARTP_CMD_STOP:
            UARTP_UART_PutChar(UARTP_RSP_OK);
            return true;

        default:
            flush_rx();
            UARTP_UART_PutChar(UARTP_RSP_ERR);
            return true;
    }
}

/* ================= CONTROL tick ================= */
void UARTP_ControlTick(void)
{
    if (UARTP_CtrlState == UARTP_CTRL_STOPPING) {
        bool done = true;
        if (g_cfg.stop_step) done = g_cfg.stop_step();

        if (done) {
            UARTP_CtrlState = UARTP_CTRL_STOPPED;
            UARTP_SysMode   = UARTP_SYS_COMMAND;

            UARTP_ISR_RX_Disable();  /* 🔒 otra vez */

            UARTP_Ncnt = 0u;
            UARTP_FlagSendU = 0u;
            UARTP_FlagSendY = 0u;
        }
    }
}
