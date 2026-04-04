#include "project.h"

/* =========================================================
   Variables globales
   ========================================================= */
#define REG_LED_MAX     (7u)    /* máximo valor antes de volver a 0 */
static uint8_t reg_led_val = 0u;

/* =========================================================
   Prototipos
   ========================================================= */
void registro_led_init(void);
void registro_led_next(void);
CY_ISR_PROTO(isr_toggle_ref_Handler);

/* =========================================================
   Funciones
   ========================================================= */

/* Inicializa el registro y la interrupción */
void registro_led_init(void)
{
    /* Valor inicial */
    reg_led_val = 0u;
    registro_led_Write(reg_led_val);

    /* Configurar e iniciar interrupción */
    isr_toggle_ref_StartEx(isr_toggle_ref_Handler);
    CyGlobalIntEnable;
}

/* Avanza al siguiente valor del registro LED */
void registro_led_next(void)
{
    reg_led_val++;
    if (reg_led_val > REG_LED_MAX)
        reg_led_val = 0u;

    registro_led_Write(reg_led_val);
}

/* ISR asociada al flanco (por ejemplo, un botón o timer) */
CY_ISR(isr_toggle_ref_Handler)
{
    /* Limpiar bandera (si corresponde, depende de la fuente del ISR) */
    isr_toggle_ref_ClearPending();

    /* Llamar a la función que cambia el modo */
    registro_led_next();
}

/* =========================================================
   main()
   ========================================================= */
int main(void)
{
    CyGlobalIntEnable;
    timer_led_Start();
    registro_led_init();

    for(;;)
    {
        /* Loop principal vacío, todo manejado por interrupciones */
    }
}
