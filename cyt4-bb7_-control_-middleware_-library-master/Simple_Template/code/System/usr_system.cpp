/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: usr_system.cpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-06
 * @FilePath: usr_system.cpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\System\usr_system.cpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\System\usr_system.cpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#include "System/usr_system.hpp"
#include "System/usr_uart.hpp"
#include "zf_common_headfile.h"
#include <math.h>
// 任务所需包含头文件
#include "algorithm/mahony/mahony.h"
#include "Module/IMU660RB/imu_data.h"
#include "Controller/LQR.h"
#include "Controller/Motor.h"
#include "Controller/VMC.h"

// 外设宏定义
#define TEST_LED (P19_0)
float u_balance = 0.0f; // 控制输入变量
// 自定义类变量
USR_SYSTEM usr_sys;

// 全局变量：目标线速度 (单位 m/s)，供 ControlTask 和 VMCTask 共用
float target_linear_speed = 0.0f; 

// 用于存储估算的车身高度 (单位 m)
static float current_height = 0.0f; 
static float vertical_velocity = 0.0f; // 垂直速度估算

// 任务专属变量
// 启动任务
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
void defaultTask(void *argument);

// 启动任务
osThreadId_t IMU660RBTaskHandle;
const osThreadAttr_t IMU660RBTask_attributes = {
    .name = "IMU660RBTask ",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityRealtime1,
};
void IMU660RBTask(void *argument);

osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
    .name = "ControlTask" ,
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
void ControlTask(void *argument);

osThreadId_t VMCTaskHandle;
const osThreadAttr_t VMCTask_attributes = {
    .name = "VMCTask" ,
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
void VMCTask(void *argument);

uint16 delay_time = 0;
uint8 led_state = 0;
/**
 * @brief 用户系统初始化
 *
 */
void usrSystemInit(void) {
  // 外设初始化
  usr_sys.peripheralInit();
  // 任务初始化
  usr_sys.TaskCreate();
}

/**
 * @brief 用户外设初始化
 *
 */
void USR_SYSTEM::peripheralInit(void) {

  // 初始化所有串口以及fifo，并使能接收中断(除了串口2)
  usrUartInit();

  // 设置周期中断1ms，用于串口空闲中断判断
  pit_us_init(PIT_CH0, 1000);

  // 初始化 TEST_LED 输出 默认高电平 推挽输出模式
  gpio_init(TEST_LED, GPO, GPIO_LOW, GPO_PUSH_PULL);

  // 初始化IMU660RB
  imu_init(&imu660rb);

  // 计算LQR增益矩阵
  LQR_ComputeK();                            

  // 速度 PID（线速度闭环）参数（宏定义）
  MotorPID_Init(MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_I_TERM_MAX, MOTOR_PID_OUT_MAX);
}   

/**
 * @brief 用户任务创建
 *
 */
void USR_SYSTEM::TaskCreate() {

  // 创建默认任务
  defaultTaskHandle = osThreadNew(defaultTask, NULL, &defaultTask_attributes);

  // 创建IMU惯导任务
  IMU660RBTaskHandle = osThreadNew(IMU660RBTask, NULL, &IMU660RBTask_attributes);

  // 创建控制任务
  ControlTaskHandle = osThreadNew(ControlTask, NULL, &ControlTask_attributes);

  // 创建 VMC 控制任务
  VMCTaskHandle = osThreadNew(VMCTask, NULL, &VMCTask_attributes);

}

/**
 * @brief 启动默认任务
 *
 * @param argument 传参
 */
void defaultTask(void *argument) {
  /* USER CODE BEGIN 5 */
  while(1)
  { 
    gpio_toggle_level(P19_0);
    
    vTaskDelay(500);
  /* USER CODE END 5 */
  }
}

/**
 * @brief 启动 IMU660RB 任务
 *
 * @param argument 传参
 */
void IMU660RBTask(void *argument) {
  /* USER CODE BEGIN 5 */
  while(1)
  { 
     if(imu660rb.imu_data_ready)
     {  
        imu_mahony_update(&imu660rb.raw_data, imu660rb.dt, &imu660rb.angles);
        if(imu660rb.imu_data_true >= 0) {
          imu_data_check(&imu660rb);           // 检查数据有效性
        }  
        if(imu660rb.imu_data_true == -1) {
          imu_cordinate_convert(&imu660rb); // 坐标系转换
          imu_acceleration_compute(&imu660rb); // 计算欧拉角加速度
          imu_zero_calibration(&imu660rb);  // 零偏校准

          float acc_z_body = imu660rb.raw_data.acc[2]; 
          // 简单补偿重力 (假设竖直向上为正，pitch=0 时 az 应抵消 g)
          float acc_vertical = acc_z_body * cosf(imu660rb.angles.pitch_in_cordinate) * cosf(imu660rb.angles.roll_in_cordinate) - 9.8f; 
          
          // 积分计算速度和高度 (加入简单阻尼防止发散)
          vertical_velocity += acc_vertical * imu660rb.dt;
          vertical_velocity *= 0.98f; // 阻尼系数
          current_height += vertical_velocity * imu660rb.dt;
          
          //  如果速度接近 0 且高度低于阈值，强制归零高度防止漂移
          if (fabsf(vertical_velocity) < 0.01f && current_height < 0.05f) {
              current_height = 0.0f;
              vertical_velocity = 0.0f;
          }

          imu660rb.imu_data_true = -2;      // 数据处理完成，设置状态为 -2，等待发送
        }
        if(imu660rb.imu_data_true == -2) {
          imu_tx_data(&imu660rb);           // 发送 IMU 数据
          imu660rb.imu_data_true = -1;
        }
        imu660rb.imu_data_ready = false;    // 读取数据后，重置数据就绪标志
     }
     
    vTaskDelay(10);
  }
  /* USER CODE END 5 */
}

void ControlTask(void *argument) {
  /* USER CODE BEGIN 5 */
  while(1)
  { 
    // 在这里可以添加平衡控制的代码，例如使用LQR算法计算控制输入，并通过PWM输出控制电机
    float x_ref[] = {0.0f, 0.0f, 0.0f, 0.0f}; // 目标状态向量
    float x_current[] = {Wheel_position, Wheel_Speed, imu660rb.angles.pitch_in_cordinate, imu660rb.angles.pitch_acceleration}; // 当前状态向量
    float x_error[4];
    for (int i = 0; i < 4; i++) {
        x_error[i] = x_current[i] - x_ref[i]; // 计算状态误差
    }
    u_balance = -K[0] * x_error[0] - K[1] * x_error[1] - K[2] * x_error[2] - K[3] * x_error[3]; // 计算平衡扭矩

    // 线速度闭环扭矩（pid）
    float u_total = MotorComputeTotalTorque(target_linear_speed, Wheel_Speed, u_balance);

    // TODO: 把 u_total 发送到电机驱动，例如 PWM 直接输出
    sendTorqueToMotor(u_total);

    vTaskDelay(10);
  }
  /* USER CODE END 5 */
}
void VMCTask(void *argument) {
  /* USER CODE BEGIN 5 */
  while(1)
  {

    double x_fk_left = 0.0, y_fk_left = 0.0;
    double x_fk_right = 0.0, y_fk_right = 0.0;
    
    // 读取当前左右腿关节角度
    double alpha_left_rad, beta_left_rad, alpha_right_rad, beta_right_rad;
    VMC_GetHipAnglesRadLeft(&alpha_left_rad, &beta_left_rad);
    VMC_GetHipAnglesRadRight(&alpha_right_rad, &beta_right_rad);

    VMC_FKResult_t fk_result_left;
    VMC_FKResult_t fk_result_right;

    int ret_left = VMC_ForwardKinematics(&VMC_Left, alpha_left_rad, beta_left_rad, &fk_result_left);
    int ret_right = VMC_ForwardKinematics(&VMC_Right, alpha_right_rad, beta_right_rad, &fk_result_right);

    if (ret_left != 0 || ret_right != 0) {
        // 计算失败，使用默认值或跳过
        x_fk_left = 0.0; y_fk_left = -VMC_Left.l5;
        x_fk_right = 0.0; y_fk_right = -VMC_Right.l5;
    } else {
        // 从结构体中提取计算结果
        x_fk_left = fk_result_left.x;
        y_fk_left = fk_result_left.y;
        
        x_fk_right = fk_result_right.x;
        y_fk_right = fk_result_right.y;
    }


    const float target_height = 0.0f; 
    double height_error      = (double)target_height - (double)current_height; // 使用估算高度
    double height_error_dot  = -vertical_velocity; 
    
    double roll_error        = -imu660rb.angles.roll_in_cordinate;              // 目标水平为 0
    double roll_error_dot    = -imu660rb.angles.roll_acceleration;             // 期望侧倾速度为 0

    double velocity_error    = (double)target_linear_speed - (double)Wheel_Speed;              

    if (VMC_ComputeVirtualForces(&VMC_VirtualForceParams,
                                 height_error, height_error_dot,
                                 roll_error, roll_error_dot,
                                 velocity_error,
                                 &fz_left_cmd, &fz_right_cmd, &fx_cmd) != 0) {
      // 出错时跳过本次计算
      vTaskDelay(10);
      continue;
    }

    VMC_LeftState.alpha = alpha_left_rad;
    VMC_LeftState.beta  = beta_left_rad;
    VMC_RightState.alpha = alpha_right_rad;
    VMC_RightState.beta  = beta_right_rad;

    if (VMC_ComputeLegTheta(&VMC_Left, &VMC_LeftState, x_fk_left, y_fk_left) != 0) {
      vTaskDelay(10);
      continue;
    }
    if (VMC_ComputeLegTheta(&VMC_Right, &VMC_RightState, x_fk_right, y_fk_right) != 0) {
      vTaskDelay(10);
      continue;
    }

    double tau_alpha_left = 0.0, tau_beta_left = 0.0;
    double tau_alpha_right = 0.0, tau_beta_right = 0.0;

    if (VMC_ComputeJointTorqueFromFootForce(&VMC_Left,
                                           VMC_LeftState.alpha, VMC_LeftState.beta,
                                           VMC_LeftState.theta1, VMC_LeftState.theta2,
                                           fx_cmd, fz_left_cmd,
                                           &tau_alpha_left, &tau_beta_left) != 0) {
      vTaskDelay(10);
      continue;
    }

    if (VMC_ComputeJointTorqueFromFootForce(&VMC_Right,
                                           VMC_RightState.alpha, VMC_RightState.beta,
                                           VMC_RightState.theta1, VMC_RightState.theta2,
                                           fx_cmd, fz_right_cmd,
                                           &tau_alpha_right, &tau_beta_right) != 0) {
      vTaskDelay(10);
      continue;
    }

    // tau_alpha 是后髋关节，tau_beta 是前髋关节
    // 左前：tau_beta_left, 左后：tau_alpha_left, 右前：tau_beta_right, 右后：tau_alpha_right

    // 发送四个扭矩到串口 2，经过后期电机执行器解包执行
    char torque_buf[128];
    int tlen = snprintf(torque_buf, sizeof(torque_buf),
                        "CMD:TORQUE:LF:%.2f LB:%.2f RF:%.2f RB:%.2f\r\n",
                        tau_beta_left, tau_alpha_left, tau_beta_right, tau_alpha_right);
    if (tlen > 0 && tlen < (int)sizeof(torque_buf)) {
      UartSendArray[2]((uint8_t*)torque_buf, (uint16_t)tlen);
    }

    vTaskDelay(10);
  }
  /* USER CODE END 5 */
}

