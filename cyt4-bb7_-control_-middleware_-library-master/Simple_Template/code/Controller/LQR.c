#include "LQR.h"
#include "VMC.h"
#include <string.h>
float K[4]; // LQR增益矩阵
float left_Wheel_position = 0.0f; // 左侧电机位置
float right_Wheel_position = 0.0f; // 右侧电机位置
float left_Wheel_Speed = 0.0f; // 左侧电机速度
float right_Wheel_Speed = 0.0f; // 右侧电机速度
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

static void mat4_mul_vec(const float a[4][4], const float b[4], float out[4]) {
    for (int i = 0; i < 4; i++) {
        out[i] = 0.0f;
        for (int j = 0; j < 4; j++) {
            out[i] += a[i][j] * b[j];
        }
    }
}

static float vec4_dot(const float a[4], const float b[4]) {
    float sum = 0.0f;
    for (int i = 0; i < 4; i++) {
        sum += a[i] * b[i];
    }
    return sum;
}

static void mat4_identity(float out[4][4]) {
    memset(out, 0, sizeof(float) * 16);
    for (int i = 0; i < 4; i++) {
        out[i][i] = 1.0f;
    }
}

void LQR_GetMatrices(float A[4][4], float B[4], float Q[4][4], float *R_out) {
    // A 矩阵
    A[0][0] = A_00; A[0][1] = A_01; A[0][2] = A_02; A[0][3] = A_03;
    A[1][0] = A_10; A[1][1] = A_11; A[1][2] = A_12; A[1][3] = A_13;
    A[2][0] = A_20; A[2][1] = A_21; A[2][2] = A_22; A[2][3] = A_23;
    A[3][0] = A_30; A[3][1] = A_31; A[3][2] = A_32; A[3][3] = A_33;

    // B 向量
    B[0] = B_00;
    B[1] = B_10;
    B[2] = B_20;
    B[3] = B_30;

    // Q 矩阵对角
    Q[0][0] = Q_00; Q[0][1] = 0.0f; Q[0][2] = 0.0f; Q[0][3] = 0.0f;
    Q[1][0] = 0.0f; Q[1][1] = Q_11; Q[1][2] = 0.0f; Q[1][3] = 0.0f;
    Q[2][0] = 0.0f; Q[2][1] = 0.0f; Q[2][2] = Q_22; Q[2][3] = 0.0f;
    Q[3][0] = 0.0f; Q[3][1] = 0.0f; Q[3][2] = 0.0f; Q[3][3] = Q_33;

    if (R_out) *R_out = R;
}

void LQR_ComputeK() {
    K[0] = -0.000f; 
    // K[1] = 1.5085f;
    //K[0] = -1.500f;
    K[1] = 1.90430f;
    K[2] = 8.3446f;
    K[3] = 1.5400f;
}

uint32_t LQR_process_speed(char *buf, uint32_t len) {
    static bool first_run = true;
    float init_position_left = 0.0f; // 初始左轮位置
    float init_position_right = 0.0f; // 初始右轮位置
    if (buf == NULL || len == 0) {
        return 0;
    }

    if (len >= 64U) {
        len = 63U;
    }
    buf[len] = '\0';
    int32_t left_v = 0;
    int32_t right_v = 0;
    int32_t left_p = 0;
    int32_t right_p = 0;
    char *start = buf;
    while ((start = strstr(start, "SPD:")) != NULL) {
        if (sscanf(start, "SPD:%d,%d,POS:%d,%d", &left_v, &right_v, &left_p, &right_p) == 4) {
            left_Wheel_Speed = left_v*0.035f*2*3.1415f/60.0f; // 转速转换为线速度，单位 m/s
            left_Wheel_position = left_p*19.2f*0.035f*2*3.1415f/360-init_position_left;
            right_Wheel_Speed = -right_v*0.035f*2*3.1415f/60.0f;
            right_Wheel_position = -right_p*19.2f*0.035f*2*3.1415f/360-init_position_right;
            
            m7_0_to_m7_1_data[4] = 1;
            m7_0_to_m7_1_data[2] = left_Wheel_position;
            m7_0_to_m7_1_data[3] = right_Wheel_position;
            SCB_CleanInvalidateDCache_by_Addr(&m7_0_to_m7_1_data, sizeof(m7_0_to_m7_1_data));      
            
            if (first_run) {
                init_position_left = left_Wheel_position;
                init_position_right = right_Wheel_position;
                first_run = false;
            }
            return 1;
        }
        start += 4;
    }
    return 0;
}

void uart4_callback(void) {
    static char receive_speed_data[64]; // 定义静态缓冲区，保持跨回调数据
    uint8_t temp_dat;
    static int data_count = 0;

    if (uart_query_byte(UART_4, &temp_dat)) {
        if (temp_dat != '\r' && temp_dat != '\n') {
            if (data_count < (int)(sizeof(receive_speed_data) - 1)) {
                receive_speed_data[data_count++] = (char)temp_dat; // 存储接收到的字节
            } else {
                data_count = 0; // 超长数据重置
            }
        }

        if (temp_dat == '\r' || temp_dat == '\n' || data_count >= (int)(sizeof(receive_speed_data) - 1)) {
            if (data_count > 0) {
                LQR_process_speed(receive_speed_data, (uint32_t)data_count);
            }
            data_count = 0;
        }
    }
}

void sendSpeedToMotor(float left_sp,float right_sp)
{
    char buf[64];
    int left_duty = (int)(left_sp * 1340.0f); // 将速度转换为占空比百分比
    int right_duty = (int)(right_sp * 1340.0f);
    if (left_duty > 8000) left_duty = 8000;
    if (left_duty < -8000) left_duty = -8000;
    if (right_duty > 8000) right_duty = 8000;
    if (right_duty < -8000) right_duty = -8000;
     // 1. 格式化字符串，假设电机驱动器接受 "SET-DUTY,left_duty,right_duty" 格式的命令
    int len = snprintf(buf, sizeof(buf), "SET-DUTY,%d,%d\r\n", left_duty, right_duty);
    //int len = snprintf(buf, sizeof(buf), "SET-DUTY,%d,%d\r\n", 0, 0);
    // 2. 通过UART4发送
    UartSendArray[4]((uint8_t*)buf, (uint16_t)len);
}

