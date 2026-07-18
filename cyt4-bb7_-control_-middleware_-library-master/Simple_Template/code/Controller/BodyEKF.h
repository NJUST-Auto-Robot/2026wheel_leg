#ifndef _BODY_EKF_H_
#define _BODY_EKF_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 车身速度估计器状态结构
 *
 * pos: 车身位置估计值
 * vel: 车身线速度估计值
 * P:   估计协方差矩阵
 * Q_pos: 位置过程噪声
 * Q_vel: 速度过程噪声
 * R_pos: 位置观测噪声
 * R_vel: 速度观测噪声
 */
typedef struct {
    float pos;
    float vel;
    float P[2][2];
    float Q_pos;
    float Q_vel;
    float R_pos;
    float R_vel;
} BodyEkf_t;

/**
 * @brief 初始化车身 EKF
 *
 * @param initial_pos 初始位置
 * @param initial_vel 初始速度
 */
void BodyEkfInit(float initial_pos, float initial_vel);

/**
 * @brief 更新车身 EKF
 *
 * @param position_measure 车轮位置测量值（由左右轮均值得到）
 * @param velocity_measure 车轮速度测量值（由左右轮速度均值得到）
 * @param pitch_angle_rad  车身俯仰角（弧度）
 * @param pitch_rate_rad_s 俯仰角速度（弧度/秒）
 * @param dt              采样时间间隔
 */
void BodyEkfUpdate(float position_measure,
                   float velocity_measure,
                   float pitch_angle_rad,
                   float pitch_rate_rad_s,
                   float dt);

/**
 * @brief 获取估计的车身位置
 */
float BodyEkfGetPos(void);

/**
 * @brief 获取估计的车身速度
 */
float BodyEkfGetVel(void);

#ifdef __cplusplus
}
#endif

#endif // _BODY_EKF_H_
