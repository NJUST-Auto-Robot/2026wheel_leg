#ifndef _IMU_DATA_H_
#define _IMU_DATA_H_ 

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

// 宏定义
#define IMU_FUSION_SAMPLE_RATE 100.0f          // 采样频率 (Hz)
#define IMU_FUSION_COMPLEMENTARY_ALPHA 0.98f   // 互补滤波alpha值
#define IMU_FUSION_GYRO_SCALE 2000.0f          // 陀螺仪量程 (°/s)
#define IMU_FUSION_ACC_SCALE 2.0f              // 加速度计量程 (g)

typedef enum {
    x = 0,
    y = 1,
    z = 2
} Axis_t;

// 结构体定义
typedef struct {
    float acc[3];   // 加速度计数据 (g) x,y,z
    float gyro[3];  // 陀螺仪数据 (°/s) x,y,z
} imu_data_t;

typedef struct {
    float roll;   // 横滚角 (°)
    float pitch;  // 俯仰角 (°)
    float yaw;    // 偏航角 (°)
    float last_roll;  
    float last_pitch;  
    float last_yaw;
    float roll_in_cordinate;   // 坐标系转换后的横滚角 (°)
    float pitch_in_cordinate;  // 坐标系转换后的俯仰角 (°)
    float yaw_in_cordinate;    // 坐标系转换后的偏航角 (°)  
    float roll_acceleration; // 横滚角加速度 (°/s?)
    float pitch_acceleration; // 俯仰角加速度 (°/s?)
    float yaw_acceleration;   // 偏航角加速度 (°/s?)
} euler_angles_t;

typedef struct {
    imu_data_t raw_data;        // 当前IMU数据
    euler_angles_t angles;      // 当前欧拉角
    float dt;                   // 时间间隔
    bool  imu_data_ready;       // 数据就绪标志
    int   imu_data_true;        // 初始化状态
    float yaw_offset_sum;       // 偏航角数值之和
    float yaw_offset_average;   // 偏航角补偿值平均值
} imu_state_t;

extern imu_state_t imu660rb; // 全局融合状态实例

// 函数声明
void imu_init(imu_state_t *state);
void imu_read_data(imu_state_t *state);
void imu_tx_data(imu_state_t *state);
void imu_data_check(imu_state_t *state);
void imu_cordinate_convert(imu_state_t *state);
void imu_zero_calibration(imu_state_t *state);
void imu_acceleration_compute(imu_state_t *state);
#ifdef __cplusplus
}
#endif

#endif



