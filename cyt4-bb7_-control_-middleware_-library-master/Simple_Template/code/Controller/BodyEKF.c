#include "BodyEKF.h"

// 线性卡尔曼滤波器的角度融合增益，用于把俯仰角信息映射到速度修正
static const float ANGLE_VEL_GAIN = 0.4f;

static BodyEkf_t body_ekf = {
    .pos   = 0.0f,
    .vel   = 0.0f,
    .P     = {{0.05f, 0.0f}, {0.0f, 1.0f}},
    .Q_pos = 0.001f,
    .Q_vel = 0.05f,
    .R_pos = 0.05f,
    .R_vel = 0.1f,
};

/**
 * @brief 初始化车身 EKF
 */
void BodyEkfInit(float initial_pos, float initial_vel) {
    body_ekf.pos = initial_pos;
    body_ekf.vel = initial_vel;
    body_ekf.P[0][0] = 0.05f;
    body_ekf.P[0][1] = 0.0f;
    body_ekf.P[1][0] = 0.0f;
    body_ekf.P[1][1] = 1.0f;
    body_ekf.Q_pos = 0.001f;
    body_ekf.Q_vel = 0.05f;
    body_ekf.R_pos = 0.05f;
    body_ekf.R_vel = 0.1f;
}

/**
 * @brief 更新车身 EKF
 *
 * 该滤波器融合车轮速度测量和车身俯仰角信息，近似估计车身线速度。
 */
void BodyEkfUpdate(float position_measure,
                   float velocity_measure,
                   float pitch_angle_rad,
                   float pitch_rate_rad_s,
                   float dt) {
    // 预测步骤：一阶运动模型
    float x_pred_pos = body_ekf.pos + body_ekf.vel * dt;
    float x_pred_vel = body_ekf.vel;

    // 预测协方差
    float p00 = body_ekf.P[0][0] + dt * (body_ekf.P[1][0] + body_ekf.P[0][1]) + dt * dt * body_ekf.P[1][1] + body_ekf.Q_pos;
    float p01 = body_ekf.P[0][1] + dt * body_ekf.P[1][1];
    float p10 = body_ekf.P[1][0] + dt * body_ekf.P[1][1];
    float p11 = body_ekf.P[1][1] + body_ekf.Q_vel;

    // 观测残差：车轮速度 + 车身俯仰角修正
    float y_vel = velocity_measure - x_pred_vel;
    float y_angle = pitch_angle_rad - ANGLE_VEL_GAIN * x_pred_vel;

    // 观测协方差矩阵 S
    float S00 = p11 + body_ekf.R_vel;
    float S01 = p11 * ANGLE_VEL_GAIN;
    float S10 = S01;
    float S11 = (ANGLE_VEL_GAIN * ANGLE_VEL_GAIN) * p11 + body_ekf.R_pos;

    float det = S00 * S11 - S01 * S10;
    if (det == 0.0f) {
        det = 1e-6f;
    }

    float invS00 = S11 / det;
    float invS01 = -S01 / det;
    float invS10 = -S10 / det;
    float invS11 = S00 / det;

    // 卡尔曼增益 K = P_pred * H^T * inv(S)
    float K10 = p10 * invS00 + p11 * invS10;
    float K11 = p10 * invS01 + p11 * invS11;
    float K00 = 0.0f; // 位置观测对位置的直接修正较弱，主要依赖速度观测
    float K01 = 0.0f;

    // 更新状态
    body_ekf.pos = x_pred_pos + K10 * y_vel + K11 * y_angle;
    body_ekf.vel = x_pred_vel + K10 * y_vel + K11 * y_angle;

    // 更新协方差
    float KH00 = 0.0f;
    float KH01 = 0.0f;
    float KH10 = 0.0f;
    float KH11 = 0.0f;
    body_ekf.P[0][0] = p00 - K10 * p10;
    body_ekf.P[0][1] = p01 - K10 * p11;
    body_ekf.P[1][0] = p10 - K11 * p10;
    body_ekf.P[1][1] = p11 - K11 * p11;
}

float BodyEkfGetPos(void) {
    return body_ekf.pos;
}

float BodyEkfGetVel(void) {
    return body_ekf.vel;
}
