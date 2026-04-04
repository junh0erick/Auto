#include "project.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdarg.h>

/* ===================== ESC PWM config ===================== */
#define ESC_MIN_US      (1000u)
#define ESC_MAX_US      (2000u)
#define ESC_ARM_MS      (6000u)

#define LOOP_DT_MS      (20u)   // 50 Hz update
#define RAMP_STEP_US    (5u)    // suavidad de cambio

static uint16 current_us = ESC_MIN_US;
static uint16 target_us  = ESC_MIN_US;

/* ===================== Control mode ===================== */
typedef enum {
    CTRL_KBD = 0,
    CTRL_POT = 1
} ctrl_mode_t;

static ctrl_mode_t g_mode = CTRL_KBD;

/* Pot behavior */
#define POT_DEADBAND_US        (3u)      // ignora jitter chico
#define POT_LPF_SHIFT          (3u)      // IIR: 1/8 (suave)
#define POT_MIN_ARM_US         (1020u)   // pot debe estar <= esto para habilitar
#define POT_MIN_ARM_HOLD_MS    (500u)    // tiempo en mínimo para aceptar

/* ADC resolution (tu config es 12 bits) */
#define ADC_MAX_COUNTS         (4095u)

static void process_cons_rx(void);

/* ===================== UART Console helpers ===================== */
static void cons_init(void)
{
    UART_CONS_Start();
}

static void cons_puts(const char *s)
{
    UART_CONS_PutString(s);
}

static void cons_printf(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    UART_CONS_PutString(buf);
}

/* ===================== ESC helpers ===================== */
static uint16 clamp_u16(uint16 x, uint16 lo, uint16 hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void esc_write_us(uint16 us)
{
    us = clamp_u16(us, ESC_MIN_US, ESC_MAX_US);
    PWM_ESC_WriteCompare(us);   // CMP Type = Less => pulso ~us microsegundos
}

static void esc_arm(void)
{
    target_us  = ESC_MIN_US;
    current_us = ESC_MIN_US;
    esc_write_us(ESC_MIN_US);
    cons_puts("[ESC] Arming: holding 1000us...\r\n");
    CyDelay(ESC_ARM_MS);
    cons_puts("[ESC] Armed.\r\n");
}

static void print_help(void)
{
    cons_puts("\r\n=== ESC Console (KitProg USB-UART) ===\r\n");
    cons_puts("Comandos:\r\n");
    cons_puts("  help              -> esta ayuda\r\n");
    cons_puts("  status            -> imprime current/target/mode\r\n");
    cons_puts("  arm               -> fuerza 1000us por unos segundos\r\n");
    cons_puts("  stop              -> target=1000\r\n");
    cons_puts("  set ####          -> set 1000..2000 (ej: set 1500)\r\n");
    cons_puts("  up ####           -> suma #### us (ej: up 50)\r\n");
    cons_puts("  dn ####           -> resta #### us (ej: dn 50)\r\n");
    cons_puts("  kick              -> 1300us por 250ms y vuelve\r\n");
    cons_puts("  mode pot|kbd      -> cambia modo de control\r\n");
    cons_puts("Tip: tambien podes mandar solo un numero: 1500\r\n\r\n");
}

static void print_status(void)
{
    const char *m = (g_mode == CTRL_POT) ? "POT" : "KBD";
    cons_printf("[STATUS] current=%u us | target=%u us | mode=%s\r\n", current_us, target_us, m);
}

/* ===================== String helpers ===================== */
static void trim(char *s)
{
    while (isspace((unsigned char)*s)) memmove(s, s+1, strlen(s));
    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n-1])) s[--n] = '\0';
}

static int parse_int(const char *s)
{
    while (isspace((unsigned char)*s)) s++;
    return atoi(s);
}

/* ===================== POT helpers ===================== */
static uint16 map_adc_to_us(uint16 adc)
{
    if (adc > ADC_MAX_COUNTS) adc = ADC_MAX_COUNTS;

    // us = 1000 + adc*(1000)/4095
    uint32 span = (uint32)(ESC_MAX_US - ESC_MIN_US); // 1000
    uint32 us   = (uint32)ESC_MIN_US + ((uint32)adc * span) / (uint32)ADC_MAX_COUNTS;

    return (uint16)us;
}

static uint16 pot_get_target_us(void)
{
    int16 raw = ADC_SAR_1_GetResult16();
    if (raw < 0) raw = 0;
    uint16 adc = (uint16)raw;

    // Filtro IIR para evitar jitter
    static uint32 filt = 0;
    if (filt == 0) {
        filt = ((uint32)adc << 8);
    } else {
        uint32 x = ((uint32)adc << 8);
        filt = filt + ((x - filt) >> POT_LPF_SHIFT);
    }

    uint16 adc_f = (uint16)(filt >> 8);
    return map_adc_to_us(adc_f);
}

/* ===================== KICK ===================== */
static void do_kick(void)
{
    uint16 saved = target_us;
    cons_puts("[ESC] KICK 1300us 250ms\r\n");
    esc_write_us(1300);
    CyDelay(250);
    esc_write_us(current_us);
    target_us = saved;
}

/* ===================== Mode selection on boot ===================== */
static void select_control_mode_on_boot(void)
{
    cons_puts("\r\n=== Seleccion de control ESC ===\r\n");
    cons_puts("Escribi 'pot' para potenciómetro o 'kbd' para teclado.\r\n");
    cons_puts("Timeout 5s: default teclado.\r\n> ");

    char line[32];
    uint8 idx = 0;

    // timeout 5000 ms
    for (uint16 ms = 0; ms < 5000u; ms++)
    {
        while (UART_CONS_GetRxBufferSize() != 0u)
        {
            uint8 ch = (uint8)UART_CONS_GetChar();

            if (ch == '\r' || ch == '\n') {
                line[idx] = '\0';
                trim(line);

                for (uint8 i=0; line[i]; i++)
                    line[i] = (char)tolower((unsigned char)line[i]);

                if (!strcmp(line, "pot")) {
                    g_mode = CTRL_POT;
                    cons_puts("\r\n[MODE] POT seleccionada.\r\n");
                    return;
                } else if (!strcmp(line, "kbd")) {
                    g_mode = CTRL_KBD;
                    cons_puts("\r\n[MODE] KBD seleccionada.\r\n");
                    return;
                } else {
                    cons_puts("\r\n[ERR] escribi pot o kbd\r\n> ");
                    idx = 0;
                }
            } else {
                if (idx < sizeof(line)-1) line[idx++] = (char)ch;
            }
        }
        CyDelay(1);
    }

    g_mode = CTRL_KBD;
    cons_puts("\r\n[MODE] Timeout -> KBD por default.\r\n");
}

/* ===================== POT arming gate (safety) ===================== */
static void pot_wait_minimum_then_enable(void)
{
    cons_puts("[POT] Baja el pot al MINIMO para habilitar.\r\n");

    uint16 hold = 0;
    while (hold < POT_MIN_ARM_HOLD_MS)
    {
        uint16 us = pot_get_target_us();

        if (us <= POT_MIN_ARM_US) {
            hold += LOOP_DT_MS;
        } else {
            hold = 0;
        }

        // Mantener ESC seguro
        target_us  = ESC_MIN_US;
        current_us = ESC_MIN_US;
        esc_write_us(ESC_MIN_US);

        // Permití cancelar con teclado si querés
        process_cons_rx(); // (declarada abajo) -> forward decl no hace falta si movés esta func debajo.
        CyDelay(LOOP_DT_MS);
    }

    cons_puts("[POT] OK. Pot en minimo. Control habilitado.\r\n");
}

/* ===================== Command parser ===================== */
static void handle_line(char *line)
{
    trim(line);
    if (line[0] == '\0') return;

    char cmd[64];
    strncpy(cmd, line, sizeof(cmd)-1);
    cmd[sizeof(cmd)-1] = '\0';
    for (size_t i=0; cmd[i]; i++) cmd[i] = (char)tolower((unsigned char)cmd[i]);

    if (!strcmp(cmd, "help"))   { print_help();   return; }
    if (!strcmp(cmd, "status")) { print_status(); return; }
    if (!strcmp(cmd, "arm"))    { esc_arm();      return; }
    if (!strcmp(cmd, "stop"))   { target_us = ESC_MIN_US; cons_puts("[CMD] stop -> target=1000\r\n"); return; }
    if (!strcmp(cmd, "kick"))   { do_kick();      return; }

    if (!strncmp(cmd, "mode", 4)) {
        char *p = line + 4;
        while (*p && isspace((unsigned char)*p)) p++;

        char arg[8];
        strncpy(arg, p, sizeof(arg)-1);
        arg[sizeof(arg)-1] = '\0';
        trim(arg);
        for (size_t i=0; arg[i]; i++) arg[i] = (char)tolower((unsigned char)arg[i]);

        if (!strcmp(arg, "pot")) {
            g_mode = CTRL_POT;
            cons_puts("[MODE] POT\r\n");
            // Seguridad: exigir mínimo
            // (si no querés esto, comentá la siguiente línea)
            // pot_wait_minimum_then_enable();
        } else if (!strcmp(arg, "kbd")) {
            g_mode = CTRL_KBD;
            cons_puts("[MODE] KBD\r\n");
        } else {
            cons_puts("[ERR] mode: usa 'mode pot' o 'mode kbd'\r\n");
        }
        return;
    }

    // En modo POT, permito comandos, pero el pot va a sobreescribir target_us en el loop.
    if (!strncmp(cmd, "set", 3)) {
        int v = parse_int(line + 3);
        if (v < (int)ESC_MIN_US || v > (int)ESC_MAX_US) { cons_puts("[ERR] set: usa 1000..2000\r\n"); return; }
        target_us = (uint16)v;
        cons_printf("[CMD] set -> target=%u\r\n", target_us);
        return;
    }

    if (!strncmp(cmd, "up", 2)) {
        int dv = parse_int(line + 2);
        int v  = (int)target_us + dv;
        v = (v < (int)ESC_MIN_US) ? (int)ESC_MIN_US : v;
        v = (v > (int)ESC_MAX_US) ? (int)ESC_MAX_US : v;
        target_us = (uint16)v;
        cons_printf("[CMD] up -> target=%u\r\n", target_us);
        return;
    }

    if (!strncmp(cmd, "dn", 2)) {
        int dv = parse_int(line + 2);
        int v  = (int)target_us - dv;
        v = (v < (int)ESC_MIN_US) ? (int)ESC_MIN_US : v;
        v = (v > (int)ESC_MAX_US) ? (int)ESC_MAX_US : v;
        target_us = (uint16)v;
        cons_printf("[CMD] dn -> target=%u\r\n", target_us);
        return;
    }

    if (isdigit((unsigned char)cmd[0])) {
        int v = atoi(cmd);
        if (v < (int)ESC_MIN_US || v > (int)ESC_MAX_US) { cons_puts("[ERR] numero invalido: usa 1000..2000\r\n"); return; }
        target_us = (uint16)v;
        cons_printf("[CMD] set -> target=%u\r\n", target_us);
        return;
    }

    cons_puts("[ERR] comando no reconocido. Escribi: help\r\n");
}

/* ===================== UART RX line buffer ===================== */
static void process_cons_rx(void)
{
    static char line[96];
    static uint8 idx = 0;

    while (UART_CONS_GetRxBufferSize() != 0u)
    {
        uint8 ch = (uint8)UART_CONS_GetChar();

        if (ch == '\r' || ch == '\n') {
            line[idx] = '\0';
            handle_line(line);
            idx = 0;
        } else {
            if (idx < sizeof(line)-1) {
                line[idx++] = (char)ch;
            } else {
                idx = 0;
                cons_puts("[ERR] linea demasiado larga\r\n");
            }
        }
    }
}

/* ===================== main ===================== */
int main(void)
{
    CyGlobalIntEnable;

    PWM_ESC_Start();
    esc_write_us(ESC_MIN_US);

    cons_init();

    /* ADC para potenciómetro */
    ADC_SAR_1_Start();
    ADC_SAR_1_StartConvert();   // Free running

    cons_puts("\r\n[BOOT] PSoC ESC Console via KitProg USB-UART.\r\n");
    cons_puts("[BOOT] PuTTY @115200. Escribi: help\r\n");
    print_help();

    /* Seleccion de modo al inicio */
    select_control_mode_on_boot();

    /* Seguridad si se eligió POT: exigir mínimo */
    if (g_mode == CTRL_POT) {
        // Nota: esta función usa process_cons_rx() arriba.
        // Si te da warning por orden de declaración, mové pot_wait_minimum_then_enable()
        // debajo de process_cons_rx().
        pot_wait_minimum_then_enable();
    }

    for (;;)
    {
        process_cons_rx();

        /* Si modo POT: target viene del ADC */
        if (g_mode == CTRL_POT) {
            uint16 pot_us = pot_get_target_us();

            int diff = (int)pot_us - (int)target_us;
            if (diff < 0) diff = -diff;

            if (diff >= (int)POT_DEADBAND_US) {
                target_us = pot_us;
            }
        }

        /* Rampa suave hacia target */
        if (current_us < target_us) {
            uint16 next = current_us + RAMP_STEP_US;
            current_us = (next > target_us) ? target_us : next;
        } else if (current_us > target_us) {
            uint16 next = current_us - RAMP_STEP_US;
            current_us = (next < target_us) ? target_us : next;
        }

        esc_write_us(current_us);
        CyDelay(LOOP_DT_MS);
    }
}
