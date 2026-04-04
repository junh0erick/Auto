#include "project.h"
#include "arm_math.h"
#include <stdbool.h>

volatile float g_cmsis_out[4];

bool cmsis_smoke_test(void)
{
    float A_data[4] = {1, 2,
                       3, 4};
    float B_data[4] = {5, 6,
                       7, 8};
    float C_data[4] = {0};

    arm_matrix_instance_f32 A, B, C;
    arm_mat_init_f32(&A, 2, 2, A_data);
    arm_mat_init_f32(&B, 2, 2, B_data);
    arm_mat_init_f32(&C, 2, 2, C_data);

    if (arm_mat_mult_f32(&A, &B, &C) != ARM_MATH_SUCCESS) return false;

    // Resultado esperado:
    // [19 22
    //  43 50]
    g_cmsis_out[0] = C_data[0];
    g_cmsis_out[1] = C_data[1];
    g_cmsis_out[2] = C_data[2];
    g_cmsis_out[3] = C_data[3];

    // Chequeo simple sin fabsf (para evitar includes extra)
    if ((C_data[0] < 18.9f) || (C_data[0] > 19.1f)) return false;
    if ((C_data[1] < 21.9f) || (C_data[1] > 22.1f)) return false;
    if ((C_data[2] < 42.9f) || (C_data[2] > 43.1f)) return false;
    if ((C_data[3] < 49.9f) || (C_data[3] > 50.1f)) return false;

    return true;
}
