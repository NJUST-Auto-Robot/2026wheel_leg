#include "mahony.h"
#include <math.h>

// Mahony滤波参数
#define TWO_KP_DEF  (2.0f * 1.0f)  // 2 * proportional gain
#define TWO_KI_DEF  (2.0f * 0.0f)  // 2 * integral gain

// 四元数结构体
typedef struct {
    float q0, q1, q2, q3;
} quaternion_t;

// Mahony滤波状态
typedef struct {
    quaternion_t q;      // 四元数
    float integralFBx;   // 积分项
    float integralFBy;
    float integralFBz;
    float twoKp;         // 比例增益
    float twoKi;         // 积分增益
} mahony_state_t;

// 全局Mahony状态实例
static mahony_state_t mahony_state;

// 辅助函数
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// 初始化Mahony滤波
void mahony_init(void) {
    mahony_state.q.q0 = 1.0f;
    mahony_state.q.q1 = 0.0f;
    mahony_state.q.q2 = 0.0f;
    mahony_state.q.q3 = 0.0f;
    mahony_state.integralFBx = 0.0f;
    mahony_state.integralFBy = 0.0f;
    mahony_state.integralFBz = 0.0f;
    mahony_state.twoKp = TWO_KP_DEF;
    mahony_state.twoKi = TWO_KI_DEF;
}

// 更新Mahony滤波
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 将陀螺仪数据转换为弧度每秒
    gx *= (float)M_PI / 180.0f;
    gy *= (float)M_PI / 180.0f;
    gz *= (float)M_PI / 180.0f;

    // 计算加速度计的倒数模长
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 计算重力向量在机体坐标系下的投影
    halfvx = mahony_state.q.q1 * mahony_state.q.q3 - mahony_state.q.q0 * mahony_state.q.q2;
    halfvy = mahony_state.q.q0 * mahony_state.q.q1 + mahony_state.q.q2 * mahony_state.q.q3;
    halfvz = mahony_state.q.q0 * mahony_state.q.q0 - 0.5f + mahony_state.q.q3 * mahony_state.q.q3;

    // 计算误差
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // 积分误差
    if (mahony_state.twoKi > 0.0f) {
        mahony_state.integralFBx += mahony_state.twoKi * halfex * dt;
        mahony_state.integralFBy += mahony_state.twoKi * halfey * dt;
        mahony_state.integralFBz += mahony_state.twoKi * halfez * dt;
        gx += mahony_state.integralFBx;
        gy += mahony_state.integralFBy;
        gz += mahony_state.integralFBz;
    } else {
        mahony_state.integralFBx = 0.0f;
        mahony_state.integralFBy = 0.0f;
        mahony_state.integralFBz = 0.0f;
    }

    // 应用比例反馈
    gx += mahony_state.twoKp * halfex;
    gy += mahony_state.twoKp * halfey;
    gz += mahony_state.twoKp * halfez;

    // 积分四元数率
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = mahony_state.q.q0;
    qb = mahony_state.q.q1;
    qc = mahony_state.q.q2;

    mahony_state.q.q0 += (-qb * gx - qc * gy - mahony_state.q.q3 * gz);
    mahony_state.q.q1 += (qa * gx + qc * gz - mahony_state.q.q3 * gy);
    mahony_state.q.q2 += (qa * gy - qb * gz + mahony_state.q.q3 * gx);
    mahony_state.q.q3 += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = invSqrt(mahony_state.q.q0 * mahony_state.q.q0 + mahony_state.q.q1 * mahony_state.q.q1 +
                        mahony_state.q.q2 * mahony_state.q.q2 + mahony_state.q.q3 * mahony_state.q.q3);
    mahony_state.q.q0 *= recipNorm;
    mahony_state.q.q1 *= recipNorm;
    mahony_state.q.q2 *= recipNorm;
    mahony_state.q.q3 *= recipNorm;
}

// 从四元数计算欧拉角
void mahony_get_euler(euler_angles_t *euler) {
    float q0 = mahony_state.q.q0;
    float q1 = mahony_state.q.q1;
    float q2 = mahony_state.q.q2;
    float q3 = mahony_state.q.q3;

    // 计算欧拉角 (弧度)
    euler->roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / (float)M_PI;
    euler->pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / (float)M_PI;
    euler->yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / (float)M_PI;
}

// 调用接口函数
void imu_mahony_init(void) {
    mahony_init();
}

void imu_mahony_update(imu_data_t *data, float dt, euler_angles_t *euler) {

    mahony_update(data->gyro[0], data->gyro[1], data->gyro[2],
                  data->acc[0], data->acc[1], data->acc[2], dt);
    mahony_get_euler(euler);
}
