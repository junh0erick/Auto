#ifndef UARTP_DMA_H
#define UARTP_DMA_H

#include <project.h>
#include <string.h>

/* =========================
   bool (PSoC Creator suele compilar gnu89)
   ========================= */
#ifndef UARTP_BOOL_DEFINED
#define UARTP_BOOL_DEFINED
typedef uint8 bool;
#endif

#ifndef true
#define true  ((bool)1u)
#endif
#ifndef false
#define false ((bool)0u)
#endif

/* =========================
   Configuración (solo números en macros)
   ========================= */
#ifndef UARTP_SOF
#define UARTP_SOF                 0x7Eu
#endif

#ifndef UARTP_MAX_PAYLOAD
#define UARTP_MAX_PAYLOAD         512u     /* bytes */
#endif

#ifndef UARTP_RX_PP_SIZE
#define UARTP_RX_PP_SIZE          512u     /* bytes, debe ser par */
#endif

#ifndef UARTP_TX_BUF_SIZE
#define UARTP_TX_BUF_SIZE         (UARTP_MAX_PAYLOAD + 5u) /* SOF+CMD+LEN(2)+PAYLOAD+CKSUM */
#endif

#if ((UARTP_RX_PP_SIZE % 2u) != 0u)
#error "UARTP_RX_PP_SIZE must be even"
#endif

/* =========================
   Tipos
   ========================= */
typedef void (*uartp_handler_t)(uint8 cmd, const uint8* payload, uint16 len_bytes);

/* Firma típica generada por Creator para DMA Initialize */
typedef uint8 (*uartp_dma_init_fn)(uint8 bytesPerBurst,
                                  uint8 requestsPerBurst,
                                  uint16 upperSrcAddress,
                                  uint16 upperDestAddress);

typedef struct
{
    volatile uint8* uart_rxdata_ptr;   /* Ej: (volatile uint8*)UART_RXDATA_PTR */
    volatile uint8* uart_txdata_ptr;   /* Ej: (volatile uint8*)UART_TXDATA_PTR */

    uartp_dma_init_fn dma_rx_init;     /* Ej: DMA_RX_DmaInitialize */
    uartp_dma_init_fn dma_tx_init;     /* Ej: DMA_TX_DmaInitialize */

    uint8 bytes_per_burst;             /* 1 */
    uint8 requests_per_burst;          /* 1 */
} uartp_cfg_t;

/* =========================
   API
   ========================= */
CY_ISR_PROTO(UARTP_DmaRxIsr);
CY_ISR_PROTO(UARTP_DmaTxIsr);

bool  UARTP_Init(const uartp_cfg_t* cfg);

void  UARTP_RegisterHandler(uint8 cmd, uartp_handler_t h);
void  UARTP_SetDefaultHandler(uartp_handler_t h);

bool  UARTP_Send(uint8 cmd, const void* payload, uint16 len_bytes);

void  UARTP_Task(void);

bool  UARTP_TxBusy(void);

uint8 UARTP_ChecksumXor(uint8 cmd, uint16 len_bytes, const uint8* payload);

#endif /* UARTP_DMA_H */
