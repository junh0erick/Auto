#include "control_app.h"
#include "project.h"

/* TF */
float tf_num[6] = {0};
float tf_den[6] = {0};

/* SS */
float ss_A[2][2] = {{0}};
float ss_B[2] = {0};
float ss_C[2] = {0};
float ss_D = 0.0f;
float ss_L[2] = {0};   /* observer gains */
float ss_K[2] = {0};   /* state feedback */
float ss_Ki  = 0.0f;
uint8_t ss_has_integrator = 0u;

/* estado mínimo placeholder */
static volatile float g_u = 0.0f;

static float read_y(void)
{
    /* TODO */
    return 0.0f;
}

static void write_u(float u)
{
    (void)u;
    /* TODO */
}

/* stop suave step-by-step */
bool control_stop_suave_step(void)
{
    const float step = 0.01f;

    if (g_u > 0.0f) {
        g_u -= step;
        if (g_u < 0.0f) g_u = 0.0f;
        write_u(g_u);
        return false;
    }
    if (g_u < 0.0f) {
        g_u += step;
        if (g_u > 0.0f) g_u = 0.0f;
        write_u(g_u);
        return false;
    }

    write_u(0.0f);
    return true;
}

void control_start(float u0)
{
    g_u = u0;
    /* TODO: init estados del controlador/observador/integrador */
    write_u(g_u);
}

void control_apply_tf(const float* c, uint16_t n)
{
    if (!c || n < 16u) return;

    /* c[0..5]=num, c[6..11]=den (resto ignorado) */
    for (uint8_t i=0; i<6; i++) {
        tf_num[i] = c[i];
        tf_den[i] = c[i+6];
    }
}

void control_apply_ss(const float* c, uint16_t n)
{
    if (!c || n < 16u) return;

    /* A11,A12,A21,A22 */
    ss_A[0][0] = c[0];
    ss_A[0][1] = c[1];
    ss_A[1][0] = c[2];
    ss_A[1][1] = c[3];

    /* B1,B2 */
    ss_B[0] = c[4];
    ss_B[1] = c[5];

    /* C1,C2 */
    ss_C[0] = c[6];
    ss_C[1] = c[7];

    /* D */
    ss_D = c[8];

    /* L observer */
    ss_L[0] = c[9];
    ss_L[1] = c[10];

    /* K feedback */
    ss_K[0] = c[11];
    ss_K[1] = c[12];

    /* Ki */
    ss_Ki = c[13];
    ss_has_integrator = (ss_Ki != 0.0f) ? 1u : 0u;

    /* c[14], c[15] reservados */
}

void control_step(void)
{
    float y = read_y();
    (void)y;

    /* TODO: algoritmo real.
       Según UARTP_Impl:
         - TF: usar tf_num/tf_den (compensador)
         - SS: usar A,B,C,D + observer (actual/predictor) + K y Ki (si aplica)
       Por ahora solo re-aplica g_u
    */
    write_u(g_u);
}
