#include "LQR.h"
#include "VMC.h"
#include <string.h>
float K[4]; // LQR增益矩阵
float Wheel_position = 0.0f; // 电机位置
float Wheel_Speed = 0.0f; // 电机速度
float Wheel_acceleration = 0.0f; // 电机加速度
static void mat4_copy(const float src[4][4], float dst[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            dst[i][j] = src[i][j];
        }
    }
}

static void mat4_add(const float a[4][4], const float b[4][4], float out[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out[i][j] = a[i][j] + b[i][j];
        }
    }
}

static void mat4_sub(const float a[4][4], const float b[4][4], float out[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out[i][j] = a[i][j] - b[i][j];
        }
    }
}

static void mat4_mul(const float a[4][4], const float b[4][4], float out[4][4]) {
    float tmp[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            tmp[i][j] = 0.0f;
            for (int k = 0; k < 4; k++) {
                tmp[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    mat4_copy(tmp, out);
}

static void mat4_transpose(const float a[4][4], float out[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out[i][j] = a[j][i];
        }
    }
}

static void mat4_scale(const float a[4][4], float scale, float out[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out[i][j] = a[i][j] * scale;
        }
    }
}

void LQR_GetMatrices(float A[4][4], float B[4], float Q[4][4], float *R_out) {
    // A 矩阵
    memset(A, 0, sizeof(float) * 16);
    A[0][1] = 1.0f;
    A[1][0] = A_10;
    A[2][3] = 1.0f;
    A[3][0] = A_30;

    // B 向量
    B[0] = B_00;
    B[1] = B_10;
    B[2] = B_20;
    B[3] = B_30;

    // Q 矩阵对角
    memset(Q, 0, sizeof(float) * 16);
    Q[0][0] = Q_00;
    Q[1][1] = Q_11;
    Q[2][2] = Q_22;
    Q[3][3] = Q_33;

    if (R_out) *R_out = R;
}

void LQR_ComputeK() {
    float A[4][4], B[4], Q[4][4];
    float Rv;
    LQR_GetMatrices(A, B, Q, &Rv);
    float Rinv = 1.0f / Rv;

    float P[4][4] = {0};
    for (int i = 0; i < 4; i++) P[i][i] = Q[i][i];

    float Pnext[4][4];
    float At[4][4];
    mat4_transpose(A, At);

    for (int iter = 0; iter < 500; iter++) {
        float temp1[4][4], temp2[4][4], temp3[4][4], temp4[4][4];

        // A^T*P + P*A
        mat4_mul(At, P, temp1);
        mat4_mul(P, A, temp2);
        mat4_add(temp1, temp2, temp3);

        // P*B*(R^-1)*(B^T*P)
        float PB[4];
        for (int i = 0; i < 4; i++) {
            PB[i] = 0.0f;
            for (int j = 0; j < 4; j++) {
                PB[i] += P[i][j] * B[j];
            }
        }

        float BTP[4];
        for (int j = 0; j < 4; j++) {
            BTP[j] = 0.0f;
            for (int i = 0; i < 4; i++) {
                BTP[j] += B[i] * P[i][j];
            }
        }

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                temp4[i][j] = PB[i] * BTP[j] * Rinv;
            }
        }

        // Pnext = A^T*P + P*A - P*B*R^-1*B^T*P + Q
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                Pnext[i][j] = temp3[i][j] - temp4[i][j] + Q[i][j];
            }
        }

        // 判断收敛：无穷范数差
        float maxDiff = 0.0f;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                float d = Pnext[i][j] - P[i][j];
                if (d < 0.0f) d = -d;
                if (d > maxDiff) maxDiff = d;
                P[i][j] = Pnext[i][j];
            }
        }

        if (maxDiff < 1e-6f) break;
    }

    // K = R^-1 * B^T * P
    for (int j = 0; j < 4; j++) {
        float tmp = 0.0f;
        for (int i = 0; i < 4; i++) {
            tmp += B[i] * P[i][j];
        }
        K[j] = Rinv * tmp;
    }
}

uint32_t uart2_speed_callback(uint8_t *buf, uint32_t len) {
    // 例：ASCII "SPD:123.4\n" 或 "HIP:12.3,45.6\n"
    if (len == 0 || len > 251) return 0; // 风险保护

    buf[len] = '\0';
    float v = 0.0f;
    float p = 0.0f;
    float a = 0.0f;
    float alpha_raw = 0.0f;
    float beta_raw = 0.0f;

    if (sscanf((char*)buf, "SPD:%f", &v) == 1) {
        Wheel_Speed = v;
    }
    else if (sscanf((char*)buf, "POS:%f", &p) == 1) {
        Wheel_position = p;
    }
    else if (sscanf((char*)buf, "ACC:%f", &a) == 1) {
        Wheel_acceleration = a;
    }
    else {
        float aL,bL,aR,bR;
        // 左前左后右前右后
        if (sscanf((char*)buf, "HIP:%f,%f,%f,%f", &aL, &bL, &aR, &bR) == 4) {
            VMC_SetHipRawAnglesLeft((double)aL, (double)bL);
            VMC_SetHipRawAnglesRight((double)aR, (double)bR);
        }
        else {
            return 0; // 无效数据
        }
    }

    return 1;
}

void process_uart2_fifo(void) {
    uint32_t avail = fifo_used(uart_buffer_s[2]);
    if (avail == 0) return;

    static uint8_t tmp[256];
    uint32_t readlen = avail;
    if (readlen > sizeof(tmp) - 1) readlen = sizeof(tmp) - 1;
    if (fifo_read_buffer(uart_buffer_s[2], tmp, &readlen, FIFO_READ_AND_CLEAN) ==
        FIFO_SUCCESS) {
        uart2_speed_callback(tmp, readlen);
    }
}

void sendTorqueToMotor(float torque)
{
    // 1. 转成协议字符串（例SPD: 扭矩）
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "CMD:TORQUE:%.2f\r\n", torque);
    if (len <= 0 || len >= (int)sizeof(buf)) return;

    // 2. 通过UART2发送
    UartSendArray[2]((uint8_t*)buf, (uint16_t)len);
}

