/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-05
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: AAARobot.cpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#include "AAARobot/AAARobot.hpp"
#include "Module/UBX/ubx_decoder.h"

#define AUTO_YAW_SUSTAIN_CNT (1000)

AAARobot car;
/*----------------------------------------------------------------*/
// 主控任务
TaskHandle_t carRunTaskHandle;
void carMainRunTask(void *);
/*----------------------------------------------------------------*/
// manual状态任务
TaskHandle_t carManualTaskHandle;
void carManualTask(void *);
/*----------------------------------------------------------------*/
// yaw_ctrl状态任务
TaskHandle_t carAutoYawTaskHandle;
void carAutoYawTask(void *);
// /*----------------------------------------------------------------*/
// collect_points任务
TaskHandle_t carCollectPointsTaskHandle;
void carCollectPointsTask(void *);
// /*----------------------------------------------------------------*/
// gps导航目标点任务
TaskHandle_t carAutoAzimuthTaskHandle;
void carAutoAzimuthTask(void *);
// /*----------------------------------------------------------------*/
// 声音解算导航目标点任务
TaskHandle_t carAutoSoundTaskHandle;
void carAutoSoundTask(void *);
// /*----------------------------------------------------------------*/
// IMU任务
TaskHandle_t IMUTaskHandle;
void IMUTask(void *);
/**
 * @brief 初始化函数
 *
 */
void AAARobot::init() {

  closed_spd_en = 0;
  is_spd_closed = closed_spd_en;

// 闭环的变量设置
#if HALL_CLOSED
  closed_gps_spd_tgt = 4000;  // 闭环GPS速度的目标值
  closed_snd_spd_tgt = 2000;  // 闭环声音速度的目标值
  closed_rvs_spd_tgt = -2000; // 闭环声音倒车速度的目标值
#else                         /*HALL_CLOSED*/
  closed_gps_spd_tgt = 3; // 闭环GPS速度的目标值
  closed_snd_spd_tgt = 2; // 闭环声音速度的目标值
  closed_rvs_spd_tgt = 2; // 闭环声音倒车速度的目标值
#endif                        /*HALL_CLOSED*/

  // 开环的变量设置
  drv_gps_norm = 0.3;
  drv_snd_norm = 0.15;
  drv_rvs_norm = -0.2;

  gps_to_snd_dis = 3;
  snd_to_gps_dis = 5;

  steerPID_kp[gps_navi] = 0.01;
  steerPID_ki[gps_navi] = 0.001;
  steerPID_kd[gps_navi] = 0.003;
  steerPID_max_out[gps_navi] = 1;
  steerPID_kp[sound_navi] = 0.035;
  steerPID_ki[sound_navi] = 0.002;
  steerPID_kd[sound_navi] = 0.0;
  steerPID_max_out[sound_navi] = 1;
#if HALL_CLOSED
  closedSpdPID_kp = 0.0003;
  closedSpdPID_ki = 0.0001;
  closedSpdPID_kd = 0;
  closedSpdPID_max_out = 1;
#else /*！HALL_CLOSED*/
  closedSpdPID_kp = 0.003;
  closedSpdPID_ki = 0.0001;
  closedSpdPID_kd = 0;
  closedSpdPID_max_out = 1;
#endif

  steer_pid[gps_navi].SetPIDParam(steerPID_kp[gps_navi], steerPID_ki[gps_navi],
                                  steerPID_kd[gps_navi], 100,
                                  steerPID_max_out[gps_navi]);
  steer_pid[sound_navi].SetPIDParam(
      steerPID_kp[sound_navi], steerPID_ki[sound_navi], steerPID_kd[sound_navi],
      100, steerPID_max_out[sound_navi]);

  closed_spd_pid.SetPIDParam(closedSpdPID_kp, closedSpdPID_ki, closedSpdPID_kd,
                             100, closedSpdPID_max_out);
#if !HALL_CLOSED
  closed_spd_pid.setDeadZone(0.02);
#endif

  // 初始化成-1，代表未定点
  for (uint64_t i = 0; i < sizeof(goal_point_s) / sizeof(goal_point_s[0]);
       i++) {
    goal_point_s[i].lon_val = -1.f;
    goal_point_s[i].lat_val = -1.f;
  }

  // 定义比赛环境
  com_env = outdoor_com;
  // com_env = indoor_com;

  // （不）使能倒车
  // is_reverse_enable = 0;
  is_reverse_enable = 1;

  // 创建任务
  xTaskCreate(carManualTask, "ManualTask", Large_Stack_Size, NULL,
              PriorityNormal, &carManualTaskHandle);
  xTaskCreate(carAutoYawTask, "AutoYawTask", Small_Stack_Size, NULL,
              PriorityNormal, &carAutoYawTaskHandle);
  xTaskCreate(carCollectPointsTask, "CollectTask", Large_Stack_Size, NULL,
              PriorityNormal, &carCollectPointsTaskHandle);
  xTaskCreate(carAutoAzimuthTask, "AutoAzimuthTask", Large_Stack_Size, NULL,
              PriorityNormal, &carAutoAzimuthTaskHandle);
  xTaskCreate(carAutoSoundTask, "AutoSoundTaskHandle", Normal_Stack_Size,
              &is_reverse_enable, PriorityNormal, &carAutoSoundTaskHandle);
  xTaskCreate(carMainRunTask, "RunTask", Large_Stack_Size, NULL, PriorityNormal,
              &carRunTaskHandle);
  xTaskCreate(IMUTask, "IMUTask", Normal_Stack_Size, NULL, PriorityRealtime,
              &IMUTaskHandle);
}

/**
 * @brief 手动遥控任务
 *
 */
void carManualTask(void *arg) {
  /* USER CODE BEGIN */

  /* 客制化准备 */

  /* Infinite loop */
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // pid数据更新
    car.cleanSteerPidIntegral();
    car.cleanSteerPidOut();

    // 关闭倒车模式，清除相关标志位
    car.is_reversing = 0;         // 切换到前进
    car.switch_reverse_cnt = 0;   // 倒车切换计数器清零
    car.switch_straight_cnt = 0;  // 前进切换计数器清零
    car.reserve_strategy_cnt = 0; // 倒车策略计数器清零

    // 电机数据处理，并做遥控断链保护
    if (car.closed_spd_en && car.is_spd_closed) { // 速度闭环

/*更新当前值*/
#if HALL_CLOSED
      car.closed_spd_set =
          (car.pFSI6X_data->GetStatus() == ESTABLISHED &&
           car.pFSI6X_data->Get_SWA() == SW_DOWN)
              ? car.closed_gps_spd_tgt * car.pFSI6X_data->Get_LY_Norm()
              : 0;
      car.closed_spd_pid.currentUpdate(car.esc_data.speed);
#else  /*!HALL_CLOSED*/
      car.closed_spd_set =
          (car.pFSI6X_data->GetStatus() == ESTABLISHED &&
           car.pFSI6X_data->Get_SWA() == SW_DOWN)
              ? car.closed_gps_spd_tgt * fabs(car.pFSI6X_data->Get_LY_Norm())
              : 0;
      car.closed_spd_pid.currentUpdate(car.gnss_data.speed);
#endif /*HALL_CLOSED*/

      /*更新目标值*/
      car.closed_spd_pid.targetUpdate(car.closed_spd_set);

/*更新输出*/
#if HALL_CLOSED
      car.motor_norm_duty = car.closed_spd_pid.Adjust();
#else  /*!HALL_CLOSED*/
      car.motor_norm_duty = car.closed_spd_pid.Adjust() *
                            func_sign(car.pFSI6X_data->Get_LY_Norm());
#endif /*HALL_CLOSED*/

    } else // 速度开环
    {
      car.motor_norm_duty = (car.pFSI6X_data->GetStatus() == ESTABLISHED &&
                             car.pFSI6X_data->Get_SWA() == SW_DOWN)
                                ? car.pFSI6X_data->Get_LY_Norm()
                                : 0;
    }

    // 舵机数据处理
#if USE_FS_I6X
    *car.pServo_norm_duty = (car.pFSI6X_data->GetStatus() == ESTABLISHED &&
                             car.pFSI6X_data->Get_SWA() == SW_DOWN)
                                ? car.pFSI6X_data->Get_RX_Norm()
                                : 0;
#else
    *car.pServo_norm_duty = car.pPS2_data->norm_PS_RX;
#endif
  }
  /* USER CODE END */
}

/**
 * @brief 收集点任务
 *
 */
void carCollectPointsTask(void *arg) {
  /* USER CODE BEGIN */

  /* 客制化准备 */

  /* Infinite loop */
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // 防止越界访问
    if (car.where_goal_point >=
        sizeof(car.goal_point_s) / sizeof(car.goal_point_s[0])) {
      car.where_goal_point =
          (sizeof(car.goal_point_s) / sizeof(car.goal_point_s[0])) - 1;
      continue;
    }

#if USE_UTM
    car.goal_utm[car.where_goal_point].x = car.pUTM_data->x;
    car.goal_utm[car.where_goal_point].y = car.pUTM_data->y;

#else
    car.goal_point_s[car.where_goal_point].lon_val = car.gnss_data.longitude;
    car.goal_point_s[car.where_goal_point].lat_val = car.gnss_data.latitude;
#endif
    if (car.where_goal_point == car.where_goal_max)
      car.where_goal_max++;
    ++car.where_goal_point;
  }
  /* USER CODE END */
}

/**
 * @brief autoYaw任务
 *
 */
void carAutoYawTask(void *) {
  /* USER CODE BEGIN */

  /* 客制化准备 */
  /* Infinite loop */
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // 电机数据处理
    if (car.closed_spd_en && car.is_spd_closed == 1) { // 闭环
      car.closed_spd_set = car.closed_gps_spd_tgt;     // 固定速度
      car.closed_spd_pid.targetUpdate(car.closed_spd_set);

#if HALL_CLOSED
      car.closed_spd_pid.currentUpdate(car.esc_data.speed);
#else  /*!HALL_CLOSED*/
      car.closed_spd_pid.currentUpdate(car.gnss_data.speed);
#endif /*HALL_CLOSED*/
      car.motor_norm_duty = car.closed_spd_pid.Adjust();
    } else {                                  // 开环
      car.motor_norm_duty = car.drv_gps_norm; // 电机数据处理
    }

    // 舵机数据处理
    *car.pServo_norm_duty = 0;
  }
}

/**
 * @brief 自动方位角任务
 *
 */
void carAutoAzimuthTask(void *arg) {
  /* USER CODE BEGIN */

  /* 客制化准备 */
  /* Infinite loop */
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /*因为每个机器人此任务的逻辑大概都不一样，故省略*/

    /* USER CODE END */
  }
}

/**
 * @brief
 *
 * @param arg
 */
void carAutoSoundTask(void *pIs_reverse_enable) {
  /* 客制化准备 */

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /*因为每个机器人此任务的逻辑大概都不一样，故省略*/
  }
}

/**
 * @brief 车子运行任务
 *
 * @param arg
 */
void carMainRunTask(void *arg) {
  TickType_t xLastWaitTime;
  const TickType_t xFreq = 1;
  xLastWaitTime = xTaskGetTickCount();
  float tmp = 0;

  for (;;) {
    vTaskDelayUntil(&xLastWaitTime, xFreq);

    /*更新传感器数据*/
    // 更新gps数据
    ++car.run_task_timer_cnt;
    if (car.run_task_timer_cnt % 40 == 0) {
      car.gnssDataUpdateFromUBX();
    }

    // 更新硅麦结算角数据// 切换成-180~180度
    tmp = (*car.pSound_settlement) > 180 ? (*car.pSound_settlement) - 360.f
                                         : (*car.pSound_settlement) - 0.f;
    car.handle_snd_azimuth = car.snd_azimuth_filter.f(tmp);

    // 通过当前状态确定车子状态
    car.runTask();

    // 根据状态激活相应的任务
    switch (car.getMoveMode()) {
    case manual_ctrl:
      xTaskNotifyGive(carManualTaskHandle);
      break;
    case auto_yaw_ctrl:
      xTaskNotifyGive(carAutoYawTaskHandle);
      break;
    case collect_points:
      car.setMoveMode(manual_ctrl); // 只执行一次，记一个点后就换回手动模式
      xTaskNotifyGive(carCollectPointsTaskHandle); // 记录点的位置
      break;
    case auto_azimuth_ctrl:
      xTaskNotifyGive(carAutoAzimuthTaskHandle);
      break;
    case auto_sound_ctrl:
      xTaskNotifyGive(carAutoSoundTaskHandle);
      break;
    default:
      break;
    }
  }
}

/**
 * @brief 车子运行任务实现
 *
 */
void AAARobot::runTask() {
  // 踩点模式，直接踩点
  if (move_mode == collect_points) {
    return;
  }

  // 不是自动模式或数据出错，统一设置为手动模式
  if (/**/
      // pReferee_inf->link_status == LOST ||
      pReferee_inf->ctrl_board_index == 0 || pFSI6X_data->Get_SWA() == SW_UP
      /**/) {
    // 设置为手动模式
    move_mode = manual_ctrl;

    // 直接函数返回
    return;
  }

  // 环境判断分类讨论
  switch (com_env) {
    /******************************室内比赛******************************/
  case indoor_com:
    // 室内模式：手动模式 -》 声音导航模式
    if (move_mode == manual_ctrl) {
      move_mode = auto_sound_ctrl;
    }
    break;
    /******************************户外比赛******************************/
  case outdoor_com:
    // 当gnss失效
    if (gnss_data.state == 0) {
      // 不允许自动模式
      // move_mode = manual_ctrl;
      // 换回开环
      is_spd_closed = 0;
      // 直接函数返回
      return;
    }
    // 假如gps回复，可以恢复闭环
    if (car.closed_spd_en == 1 && car.is_spd_closed == 0)
      car.is_spd_closed = 1;

    // 更新距离数据
    where_goal_point = pReferee_inf->ctrl_board_index - 1;
    if (goal_point_s[where_goal_point].lat_val == -1.f ||
        goal_point_s[where_goal_point].lon_val == -1.f) {
      // 目标点数据为空，不允许自动模式
      move_mode = manual_ctrl;
      // 直接函数返回
      return;
    }

    how_far_goal =
        get_two_points_distance(gnss_data.latitude, gnss_data.longitude,
                                goal_point_s[where_goal_point].lat_val,
                                goal_point_s[where_goal_point].lon_val);

    // 户外模式：手动模式 -》 相位猛冲模式
    if (move_mode == manual_ctrl) {
      move_mode = auto_yaw_ctrl;
    }

    // 户外模式：相位猛冲模式 -》 GPS导航模式
    else if (move_mode == auto_yaw_ctrl && gnss_data.speed > 1 &&
             gnss_data.pre_direction != gnss_data.direction) {
      move_mode = auto_azimuth_ctrl;
    }

    // 户外模式：GPS导航模式 -》 声音导航模式
    else if (move_mode == auto_azimuth_ctrl && how_far_goal < gps_to_snd_dis) {
      move_mode = auto_sound_ctrl;
    }

    // 户外模式：声音导航模式 -》 GPS导航模式
    else if (move_mode == auto_sound_ctrl &&
             (how_far_goal >= snd_to_gps_dis ||
              pReferee_inf->ctrl_board_index !=
                  pReferee_inf->pre_ctrl_board_index)) {
      move_mode = auto_yaw_ctrl;
      is_reversing = 0;         // 切换到前进
      switch_reverse_cnt = 0;   // 倒车切换计数器清零
      switch_straight_cnt = 0;  // 前进切换计数器清零
      reserve_strategy_cnt = 0; // 倒车策略计数器清零
    }

    // 更新控制信标序号
    pReferee_inf->pre_ctrl_board_index = pReferee_inf->ctrl_board_index;
    break;
  default:
    break;
  }
}

/**
 * @brief 从UBX更新GPS数据
 *
 */
void AAARobot::gnssDataUpdateFromUBX() {
  // 如果150ms还没有收到一次GPS数据包，大抵是似了
  if (abs((int64_t)(GetSystemTimer() - lastUBXReceivedTime)) > 120 * 1000) {
    gnss_data.state = 0;
    return;
  }
  // 进行state判断
  gnss_data.state = UBXMsg.fix_type == 3 ? 1 : 0;

  // 经纬度
  gnss_data.pre_latitude = gnss_data.latitude;
  gnss_data.pre_longitude = gnss_data.longitude;
  gnss_data.latitude = UBXPositionData.latitude;
  gnss_data.longitude = UBXPositionData.longitude;

  // 速度
  gnss_data.speed = UBXPositionData.groundSpeed;

  // 航向角
  gnss_data.pre_direction = gnss_data.direction;
  gnss_data.direction = UBXPositionData.headingOfVehicle;

  // 卫星数目
  gnss_data.satellite_used = UBXMsg.numOfSatellites;

  // 误差
  gnss_data.positionError = UBXPositionData.positionError;

  // 时间
  gnss_data.time.year = UBXMsg.year;
  gnss_data.time.month = UBXMsg.month;
  gnss_data.time.day = UBXMsg.day;
  gnss_data.time.hour = UBXMsg.hour;
  gnss_data.time.minute = UBXMsg.min;
  gnss_data.time.second = UBXMsg.second;
}

/**
 * @brief IMU数据更新
 *
 */
void IMUTask(void *) {
  TickType_t lastWakeTime = 0;
  for (;;) {

    /*因为每个机器人此任务的逻辑大概都不一样，故省略*/

    vTaskDelayUntil(&lastWakeTime, 1);
  }
}

/**
 * @brief 设置车子的移动模式
 *
 */
extern "C" void setCarMoveMode(move_mode_t _mode) { car.setMoveMode(_mode); }

extern "C" move_mode_t getCarMoveMode(void) { return car.getMoveMode(); }

/**
 * @brief 清除目标点
 *
 */
void AAARobot::clearTargetPoints() {
  for (uint8_t i = 0; i < where_goal_max; i++) {
    goal_point_s[i].lat_val = -1;
    goal_point_s[i].lon_val = -1;
  }
  where_goal_point = 0;
  where_goal_max = 0;
}

/**
 * @brief 电调数据解包函数
 *
 * @param data
 * @param len
 * @return uint32_t
 */
uint32_t escDataUnpack(uint8_t *data, uint32_t len) {
  escData_t temp_pack;
  if (len != sizeof(escData_t))
    return pdFALSE;

  memcpy(&temp_pack, data, len);

  if (temp_pack.tail0 != 0x00 || temp_pack.tail1 != 0x00 ||
      temp_pack.tail2 != 0x80 || temp_pack.tail3 != 0x7f)
    return pdFALSE;

  car.esc_data.speed = -car.esc_spd_filter.f(temp_pack.speed);
  car.esc_data.power = temp_pack.power;
  car.esc_data.tail0 = car.esc_data.tail1 = car.esc_data.tail2 =
      car.esc_data.tail3 = 0x66;
  return pdPASS;
}

/**
 * @brief 连接检测函数
 *
 * @param current_check_time 当前系统时间，单位为ms
 */
void vEscDataCheckLink(uint32_t current_check_time) {
  static uint32_t last_recv_time = 0;
  static float last_speed = 0;

  /*开始检测*/
  if (car.esc_data.tail0 == 0x66 && car.esc_data.tail1 == 0x66 &&
      car.esc_data.tail2 == 0x66 && car.esc_data.tail3 == 0x66) {
    car.esc_data.link_status = ESTABLISHED;
    last_recv_time = current_check_time;
  } else if (current_check_time - last_recv_time > ESC_LOST_THRESHOLD) {
    car.esc_data.link_status = LOST;
  }
  car.esc_data.tail0 = car.esc_data.tail1 = car.esc_data.tail2 =
      car.esc_data.tail3 = LOST_FILL_BYTE;

  if (car.esc_data.speed != 0 && car.esc_data.speed == last_speed)
    car.esc_data.link_status = LOST;
  last_speed = car.esc_data.speed;
}
