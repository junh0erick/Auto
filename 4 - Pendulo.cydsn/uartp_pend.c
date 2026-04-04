/* ============================================================
   uartp_pend.c — Protocolo UART PSoC ↔ MATLAB
   Proyecto: Péndulo Invertido — PSoC 5LP (control en MATLAB)
   ============================================================ */
#include "uartp_pend.h"
#include "pendulo.h"
#include "motor.h"

/* ============================================================
   Estado global del protocolo
   ============================================================ */
volatile uartp_sys_mode_t UARTP_SysMode = UARTP_SYS_COMMAND;

/* ============================================================
   Estado interno del parser RX en modo CONTROL
   Máquina de estados de 2 bytes:
     IDLE       → espera primer byte de u_pwm (o 's' para stop)
     GOT_LSB    → ya tiene u_lsb, espera u_msb
   ============================================================ */
#define RX_IDLE    0u
#define RX_GOT_LSB 1u
static uint8 s_rx_state = RX_IDLE;
static uint8 s_rx_lsb   = 0u;

/* ============================================================
   LL helpers (handshake word-by-word, igual que uartp_sw.c)
   ============================================================ */
#define WORD_TIMEOUT_MS  500u
#define MAX_RETRIES      50u

static void ll_putchar(uint8 c)
{
    UART_1_WriteTxData(c);
}

/* Espera un byte en RX con timeout (devuelve 0xFF si timeout). */
static uint8 ll_wait_byte(uint32 timeout_ms)
{
    uint32 cnt = 0u;
    while (UART_1_GetRxBufferSize() == 0u)
    {
        CyDelay(1u);
        if (++cnt >= timeout_ms) return 0xFFu;
    }
    return UART_1_ReadRxData();
}

/* Recibe una word de 4 bytes con eco-confirmación (igual que MATLAB).
   Devuelve 1 si OK, 0 si timeout/retries agotados. */
static uint8 ll_recv_word4(uint8 *out4)
{
    uint8 i;
    uint8 w[4];
    uint8 ctl;
    uint8 tries = 0u;

    for (;;)
    {
        if (++tries > MAX_RETRIES) return 0u;

        /* Recibir 4 bytes */
        for (i = 0u; i < 4u; i++)
        {
            w[i] = ll_wait_byte(WORD_TIMEOUT_MS);
            if (w[i] == 0xFFu) return 0u;  /* timeout */
        }

        /* Echo (igual que MATLAB espera) */
        for (i = 0u; i < 4u; i++) ll_putchar(w[i]);

        /* Esperar ACK o NAK de MATLAB */
        ctl = ll_wait_byte(WORD_TIMEOUT_MS);
        if (ctl == 0xFFu) return 0u;

        if (ctl == (uint8)'A')
        {
            /* MATLAB confirmó → enviar ACK y retornar */
            ll_putchar((uint8)'A');
            out4[0] = w[0]; out4[1] = w[1];
            out4[2] = w[2]; out4[3] = w[3];
            return 1u;
        }
        else
        {
            /* MATLAB envió NAK → re-enviar NAK y reintentar */
            ll_putchar((uint8)'N');
        }
    }
}

/* ============================================================
   UARTP_Init()
   ============================================================ */
void UARTP_Init(void)
{
    UART_1_Start();
    UARTP_SysMode = UARTP_SYS_COMMAND;
    s_rx_state = RX_IDLE;
    s_rx_lsb   = 0u;
}

/* ============================================================
   UARTP_ProcessCommand()
   Procesa UN comando en modo COMMAND.
   Llamar repetidamente desde el loop principal.
   ============================================================ */
void UARTP_ProcessCommand(void)
{
    uint8 cmd;
    uint8 payload[4];
    float fs_hz;
    uint8 *pf;

    if (UART_1_GetRxBufferSize() == 0u) return;

    cmd = UART_1_ReadRxData();

    switch (cmd)
    {
        /* ---- Reset ---- */
        case (uint8)'r':
            Motor_Free();
            g_last_u_pwm   = 0;
            g_flag_control = 0u;
            UARTP_SysMode  = UARTP_SYS_COMMAND;
            ll_putchar((uint8)'K');
            break;

        /* ---- Set Fs_inner ---- */
        case (uint8)'f':
            ll_putchar((uint8)'R');                 /* ready to receive */
            if (ll_recv_word4(payload))
            {
                /* Interpreta 4 bytes como float32 little-endian */
                pf    = (uint8*)&fs_hz;
                pf[0] = payload[0]; pf[1] = payload[1];
                pf[2] = payload[2]; pf[3] = payload[3];
                pendulo_timer_set_fs(fs_hz);
                ll_putchar((uint8)'K');
            }
            else
            {
                ll_putchar((uint8)'!');
            }
            break;

        /* ---- Iniciar control ---- */
        case (uint8)'i':
            Motor_Free();
            g_last_u_pwm   = 0;
            g_flag_control = 0u;
            s_rx_state     = RX_IDLE;
            s_rx_lsb       = 0u;
            UARTP_SysMode  = UARTP_SYS_CONTROL;
            ll_putchar((uint8)'K');
            break;

        /* ---- Stop (también válido en COMMAND, por si llega tarde) ---- */
        case (uint8)'s':
            Motor_Free();
            g_last_u_pwm  = 0;
            UARTP_SysMode = UARTP_SYS_COMMAND;
            ll_putchar((uint8)'K');
            break;

        default:
            /* Byte desconocido: ignorar */
            break;
    }
}

/* ============================================================
   UARTP_ControlRxPoll()
   Gestiona los bytes RX durante el modo CONTROL.
   Llamar desde el loop principal (NO desde ISR).

   Protocolo de recepción:
     Bytes de u_pwm: int16 little-endian (2 bytes)
     Byte de stop:   's' = 0x73  (llega solo, no como parte de u)

   MATLAB garantiza secuencia [0x00, 0x00, 's'] para stop limpio:
     PSoC lee [0x00, 0x00] como u=0, luego 's' como stop.
   ============================================================ */
void UARTP_ControlRxPoll(void)
{
    uint8 b;

    while (UART_1_GetRxBufferSize() > 0u)
    {
        b = UART_1_ReadRxData();

        if (s_rx_state == RX_IDLE)
        {
            if (b == (uint8)'s')
            {
                /* Stop command recibido */
                Motor_Free();
                g_last_u_pwm  = 0;
                UARTP_SysMode = UARTP_SYS_COMMAND;
                ll_putchar((uint8)'K');
                return;
            }
            /* Primer byte del par u_pwm */
            s_rx_lsb   = b;
            s_rx_state = RX_GOT_LSB;
        }
        else  /* RX_GOT_LSB */
        {
            /* Segundo byte: completar int16 */
            g_last_u_pwm = (int16)((uint16)s_rx_lsb | ((uint16)b << 8));
            s_rx_state   = RX_IDLE;
        }
    }
}
