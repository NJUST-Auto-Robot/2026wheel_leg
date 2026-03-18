/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-05
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: AAARobot.hpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#pragma once
// 引用的C头文件
#include "Module/MPU6050/mpu6050.h"
#include "Module/PS2/ps2.h"
#include "Module/Servo/servo.h"
#include "cmsis_os.h"
#include "zf_common_headfile.h"
#include "zf_device_gnss.h"

#if __cplusplus
// 引用的C++头文件
#include "Algorithm/Filters/filters.hpp"
#include "Algorithm/PID/pid.hpp"
#include "Module/FS_I6X/FS_I6X.hpp"
#include "Module/IPC/ipc.hpp"
#include "Module/MotorDrive/MotorDrive.hpp"
#include "Module/RefereSystem/RefereSystem.hpp"
#include "System/usr_system.hpp"
#endif

// 是否使用霍尔测速闭环
#define HALL_CLOSED (0)

typedef enum {
  // 手动操控模式
  manual_ctrl,
  collect_points,

  // 自动运行模式
  auto_yaw_ctrl,     // 相位猛冲模式
  auto_azimuth_ctrl, // 根据GPS方位角控制
  auto_sound_ctrl,   // 根据声音解算控制
} move_mode_t;

typedef struct {
  double lat_val; // 纬度
  double lon_val; // 经度
} lon_lat_t;

// 比赛环境
typedef enum {
  outdoor_com, // 户外
  indoor_com,  // 室内
} competition_env_t;

// 导航模式
// navigationMode
typedef enum {
  gps_navi,   // GPS导航
  sound_navi, // 声音导航
} navi_mode_t;

// 倒车策略
// 1: 倒车角度限幅
// 2: 直接倒车一定时间
#define RESERVE_STRATEGY (2)
#if RESERVE_STRATEGY == 1
#define REVERSE_ANGLE_LIMIT (0.3)
#endif

/**
 * @brief 电驱数据解包
 *
 * @param data
 * @param len
 * @return uint32_t
 */
uint32_t escDataUnpack(uint8_t *data, uint32_t len);
#define ESC_LOST_THRESHOLD 300

#if __cplusplus
// 电驱速度反馈数据帧结构
typedef struct __escData_t {
  float speed;
  float power;
  uint8_t tail0;
  uint8_t tail1;
  uint8_t tail2;
  uint8_t tail3;
  LinkageStatus_Typedef link_status;
} escData_t;

class AAARobot {
private:
  /* data */
public:
  escData_t esc_data;              // 电驱反馈数据
  MedianFilter<40> esc_spd_filter; // 电驱速度数据滤波器
  float gps_to_snd_dis = 3;        // gps变到声音的距离
  float snd_to_gps_dis = 5;        // 声音变到gps的距离
  float drv_gps_norm = 0;          // gps电机的归一化数值
  float drv_snd_norm = 0;          // 声音电机的归一化数值
  float drv_rvs_norm = 0;          // 倒车电机的归一化数值

#if RESERVE_STRATEGY == 2
  uint16_t reserve_strategy_cnt = 0; // 倒车计数器
#endif
  uint8_t closed_spd_en = 0;                   // 速度闭环使能
  uint8_t is_spd_closed;                       // 正在速度闭环标志
  float closed_spd_set = 0;                    // 闭环速度的设置值
  float closed_gps_spd_tgt = 0;                // 闭环GPS速度的目标值
  float closed_snd_spd_tgt = 0;                // 闭环声音速度的目标值
  float closed_rvs_spd_tgt = 0;                // 闭环倒车速度的目标值
  PID_t closed_spd_pid;                        // 闭环速度的pid
  uint8_t pre_is_reversing = 0;                // 上一次的倒车标志
  uint8_t is_reversing = 0;                    // 倒车标志位
  uint8_t is_reverse_enable = 0;               // 倒车使能
  uint32_t *pSound_settlement = &ipc_get_data; // 声音结算角
  float handle_snd_azimuth = 0;        // 实际用于处理的声音方位角
  MedianFilter<10> snd_azimuth_filter; // 声音方位角中指滤波器
  navi_mode_t navi_mode;               // 导航模式
  competition_env_t com_env;           // 比赛环境
  RefereeInf_t *pReferee_inf = &referee_inf; // 裁判系统数据指针
  FS_I6X_Classdef *pFSI6X_data = &remote;    // FSI6X数据的指针
  ps2_data_t *pPS2_data = &ps2_data;         // ps2数据的指针
  move_mode_t move_mode;                     // 运动状态
  PID_t steer_pid[2];                        // 转向pid
  motor_dir_t motor_dir;                     // 运动方向
  float motor_norm_duty = 0;                 // 电机的归一化数值
  float *pServo_norm_duty = &g_servo_norm_ang; // 舵机的归一化数值的指针
  struct mpu_rec_s MPU_data;                   // 陀螺仪数据
  float filtered_yaw = 0;                      // 滤波得到的yaw数据
  MedianFilter<5> mpu_filter;                  // 陀螺仪中值滤波器
  gnss_info_struct gnss_data;                  // 北斗信息
  UTM_t UTM_data;                              // utm信息的指针
  lon_lat_t goal_point_s[8];                   // 目标点集合
  UTM_t goal_utm[100];                         // 目标utm集合
  uint16_t where_goal_point; // 当前目标点在集合里面的索引值
  uint16_t where_goal_max;   // 用来记录上一个值的最大值
  double target_azimuth = 0; // 目标方位角
  float how_far_goal = 0;    // 距离目标点的距离
  uint32_t remoteIsMidCNT = 0;
  uint32_t remoteIsDownCNT = 0;
  uint8_t exitFromAuto = 0;
  float steerPID_kp[2] = {0};
  float steerPID_ki[2] = {0};
  float steerPID_kd[2] = {0};
  float steerPID_max_out[2] = {0};
  float closedSpdPID_kp = 0;
  float closedSpdPID_ki = 0;
  float closedSpdPID_kd = 0;
  float closedSpdPID_max_out;
  uint32_t switch_reverse_cnt = 0;
  uint32_t switch_straight_cnt = 0;

  AAARobot(/* args */) {}
  ~AAARobot() {}
  void init();
  void runTask();
  move_mode_t getMoveMode() { return move_mode; }
  void setMoveMode(move_mode_t _mode) { move_mode = _mode; }
  void cleanSteerPidIntegral() {
    steer_pid[gps_navi].clean_integral();
    steer_pid[sound_navi].clean_integral();
  }
  void cleanSteerPidOut() {
    steer_pid[gps_navi].clean_out();
    steer_pid[sound_navi].clean_out();
  }
  void gnssDataUpdateFromUBX();
  void updatePIDParam() {
    steer_pid[navi_mode].SetPIDParam(
        steerPID_kp[navi_mode], steerPID_ki[navi_mode], steerPID_kd[navi_mode],
        100, steerPID_max_out[navi_mode]);

    steer_pid[sound_navi].SetPIDParam(
        steerPID_kp[sound_navi], steerPID_ki[sound_navi],
        steerPID_kd[sound_navi], 100, steerPID_max_out[sound_navi]);

    closed_spd_pid.SetPIDParam(closedSpdPID_kp, closedSpdPID_ki,
                               closedSpdPID_kd, 100, closedSpdPID_max_out);
  }
  uint16_t run_task_timer_cnt = 0;
  void clearTargetPoints();
};

extern AAARobot car;

extern "C" void IMUTask(void *);
extern "C" void vEscDataCheckLink(uint32_t current_check_time);
#endif
