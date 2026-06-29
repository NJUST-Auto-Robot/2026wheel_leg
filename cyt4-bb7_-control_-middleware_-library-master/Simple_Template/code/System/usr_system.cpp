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
#include "Controller/CRSF.h"
#include "Controller/CRC8.h"

// 外设宏定义
#define TEST_LED (P19_0)
float left_u_balance = 0.0f; // 控制输入变量
float right_u_balance = 0.0f; // 控制输入变量
float left_vref = 0.0f; // 左轮速度参考值
float right_vref = 0.0f; // 右轮速度参考值
bool control_ready = false;
// 自定义类变量
USR_SYSTEM usr_sys;

// 全局变量：目标线速度 (单位 m/s)，供 ControlTask 和 VMCTask 共用
float target_linear_speed = 0.0f; 

// 用于存储估算的车身高度 (单位 m)
static float current_height = 0.0f; 
static float vertical_velocity = 0.0f; // 垂直速度估算
  // 定义 remap 宏，用于线性映射
  #define REMAP_VALUE(val, in_min, in_max, out_min, out_max) \
      ((float)(val - in_min) * (float)(out_max - out_min) / (float)(in_max - in_min) + (float)out_min)
// 任务专属变量
// 启动任务
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
void defaultTask(void *argument);

osThreadId_t ComunicateTaskHandle;
const osThreadAttr_t ComunicateTask_attributes = {
    .name = "ComunicateTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
void ComunicateTask(void *argument);

osThreadId_t IMU660RBTaskHandle;
const osThreadAttr_t IMU660RBTask_attributes = {
    .name = "IMU660RBTask ",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
void IMU660RBTask(void *argument);

osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
    .name = "ControlTask" ,
    .stack_size = 128 * 10,
    .priority = (osPriority_t)osPriorityRealtime,
};
void ControlTask(void *argument);

osThreadId_t VMCTaskHandle;
const osThreadAttr_t VMCTask_attributes = {
    .name = "VMCTask" ,
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
void VMCTask(void *argument);

osThreadId_t CRSFTaskHandle;
const osThreadAttr_t CRSFTask_attributes = {
    .name = "CRSFTask" ,
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
void CRSFTask(void *argument);


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

  Crc8_init(0xD5);
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
  //VMCTaskHandle = osThreadNew(VMCTask, NULL, &VMCTask_attributes);

  // 创建 CRSF 任务
  //CRSFTaskHandle = osThreadNew(CRSFTask, NULL, &CRSFTask_attributes);

  ComunicateTaskHandle = osThreadNew(ComunicateTask, NULL, &ComunicateTask_attributes);
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

     if(imu660rb.imu_data_ready)//约19.2ms读取一次IMU数据
     {  
        imu_read_data(&imu660rb);       
        if(imu660rb.imu_data_true >= 0) {
          imu_data_check(&imu660rb);        // 检查数据有效性
        }  
        if(imu660rb.imu_data_true == -1) {
          imu_cordinate_convert(&imu660rb); // 坐标系转换
          //imu_tx_data(&imu660rb);
        }
        
        imu660rb.imu_data_ready = false;    // 读取数据后，重置数据就绪标志
     }
     
    vTaskDelay(5);
  }
  /* USER CODE END 5 */
}

void ControlTask(void *argument) {
  /* USER CODE BEGIN 5 */
  static int offset_flag= 0;
  while(1)
  { 
    // 在这里可以添加平衡控制的代码，例如使用LQR算法计算控制输入，并通过PWM输出控制电机
    float x_ref[] = {0.0f, 0.0f, 1.5f, 0.0f}; // 目标状态向量
    float left_x_current[] = {left_Wheel_position, left_Wheel_Speed, imu660rb.angles.pitch, imu660rb.angles.pitch_acc}; // 当前状态向量
    float right_x_current[] = {right_Wheel_position, right_Wheel_Speed, imu660rb.angles.pitch, imu660rb.angles.pitch_acc}; // 当前状态向量
    float left_x_error[4];
    float right_x_error[4];
  
    for (int i = 0; i < 4; i++) {
        left_x_error[i] = left_x_current[i] - x_ref[i]; // 计算状态误差
        right_x_error[i] = right_x_current[i] - x_ref[i]; // 计算状态误差
    }
    left_u_balance = -K[0] * left_x_error[0] - K[1] * left_x_error[1] - K[2] * left_x_error[2] - K[3] * left_x_error[3]; // 计算平衡扭矩
    right_u_balance = -K[0] * right_x_error[0] - K[1] * right_x_error[1] - K[2] * right_x_error[2] - K[3] * right_x_error[3]; // 计算平衡扭矩

    // TODO: 把 u_total 发送到电机驱动，例如 PWM 直接输出
    left_vref = left_u_balance*0.1587; // 计算左轮速度参考值
    right_vref =  right_u_balance*0.1587; // 计算右轮速度参考值
    control_ready= true; // 控制准备就绪
    vTaskDelay(10);
  }
  /* USER CODE END 5 */
}
void VMCTask(void *argument) {
  /* USER CODE BEGIN 5 */
  while(1)
  {

    // double x_fk_left = 0.0, y_fk_left = 0.0;
    // double x_fk_right = 0.0, y_fk_right = 0.0;
    
    // // 读取当前左右腿关节角度
    // double alpha_left_rad, beta_left_rad, alpha_right_rad, beta_right_rad;
    // VMC_GetHipAnglesRadLeft(&alpha_left_rad, &beta_left_rad);
    // VMC_GetHipAnglesRadRight(&alpha_right_rad, &beta_right_rad);

    // VMC_FKResult_t fk_result_left;
    // VMC_FKResult_t fk_result_right;

    // int ret_left = VMC_ForwardKinematics(&VMC_Left, alpha_left_rad, beta_left_rad, &fk_result_left);
    // int ret_right = VMC_ForwardKinematics(&VMC_Right, alpha_right_rad, beta_right_rad, &fk_result_right);

    // if (ret_left != 0 || ret_right != 0) {
    //     // 计算失败，使用默认值或跳过
    //     x_fk_left = 0.0; y_fk_left = -VMC_Left.l5;
    //     x_fk_right = 0.0; y_fk_right = -VMC_Right.l5;
    // } else {
    //     // 从结构体中提取计算结果
    //     x_fk_left = fk_result_left.x;
    //     y_fk_left = fk_result_left.y;
        
    //     x_fk_right = fk_result_right.x;
    //     y_fk_right = fk_result_right.y;
    // }


    // const float target_height = 0.0f; 
    // double height_error      = (double)target_height - (double)current_height; // 使用估算高度
    // double height_error_dot  = -vertical_velocity; 
    
    // double roll_error        = -imu660rb.angles.roll_in_cordinate;              // 目标水平为 0
    // double roll_error_dot    = -imu660rb.angles.roll_acceleration;             // 期望侧倾速度为 0

    // double velocity_error    = (double)target_linear_speed - (double)Wheel_Speed;              

    // if (VMC_ComputeVirtualForces(&VMC_VirtualForceParams,
    //                              height_error, height_error_dot,
    //                              roll_error, roll_error_dot,
    //                              velocity_error,
    //                              &fz_left_cmd, &fz_right_cmd, &fx_cmd) != 0) {
    //   // 出错时跳过本次计算
    //   vTaskDelay(10);
    //   continue;
    // }

    // VMC_LeftState.alpha = alpha_left_rad;
    // VMC_LeftState.beta  = beta_left_rad;
    // VMC_RightState.alpha = alpha_right_rad;
    // VMC_RightState.beta  = beta_right_rad;

    // if (VMC_ComputeLegTheta(&VMC_Left, &VMC_LeftState, x_fk_left, y_fk_left) != 0) {
    //   vTaskDelay(10);
    //   continue;
    // }
    // if (VMC_ComputeLegTheta(&VMC_Right, &VMC_RightState, x_fk_right, y_fk_right) != 0) {
    //   vTaskDelay(10);
    //   continue;
    // }

    // double tau_alpha_left = 0.0, tau_beta_left = 0.0;
    // double tau_alpha_right = 0.0, tau_beta_right = 0.0;

    // if (VMC_ComputeJointTorqueFromFootForce(&VMC_Left,
    //                                        VMC_LeftState.alpha, VMC_LeftState.beta,
    //                                        VMC_LeftState.theta1, VMC_LeftState.theta2,
    //                                        fx_cmd, fz_left_cmd,
    //                                        &tau_alpha_left, &tau_beta_left) != 0) {
    //   vTaskDelay(10);
    //   continue;
    // }

    // if (VMC_ComputeJointTorqueFromFootForce(&VMC_Right,
    //                                        VMC_RightState.alpha, VMC_RightState.beta,
    //                                        VMC_RightState.theta1, VMC_RightState.theta2,
    //                                        fx_cmd, fz_right_cmd,
    //                                        &tau_alpha_right, &tau_beta_right) != 0) {
    //   vTaskDelay(10);
    //   continue;
    // }

    // // tau_alpha 是后髋关节，tau_beta 是前髋关节
    // // 左前：tau_beta_left, 左后：tau_alpha_left, 右前：tau_beta_right, 右后：tau_alpha_right

    // // 发送四个扭矩到串口 2，经过后期电机执行器解包执行
    // char torque_buf[128];
    // int tlen = snprintf(torque_buf, sizeof(torque_buf),
    //                     "CMD:TORQUE:LF:%.2f LB:%.2f RF:%.2f RB:%.2f\r\n",
    //                     tau_beta_left, tau_alpha_left, tau_beta_right, tau_alpha_right);
    // if (tlen > 0 && tlen < (int)sizeof(torque_buf)) {
    //   UartSendArray[2]((uint8_t*)torque_buf, (uint16_t)tlen);
    // }

    // vTaskDelay(10);
  }
  /* USER CODE END 5 */
}

void CRSFTask(void *argument) {
  /* USER CODE BEGIN 5 */
   
  while(1)
  { 
    // Crsf_Data_procees(); // 处理接收数据

    // if (CRSF_CH.ConnectState == SBUS_SIGNAL_OK) {
    //   if (CRSF_CH.CH2 >= 992) {
    //       target_linear_speed = REMAP_VALUE(CRSF_CH.CH2, 992, 1811, 0, 2);
    //   } else {
    //       target_linear_speed = REMAP_VALUE(CRSF_CH.CH2, 174, 992, -2, 0);
    //   }

    // } else {
    //   // 信号丢失或故障，速度归零
    //   target_linear_speed = 0.0f;
    // }

    // // 通过串口5发送目标线速度
    // {
    //   char speed_buf[32];
    //   int speed_len = snprintf(speed_buf, sizeof(speed_buf), "target_linear_speed=%.2f\r\n", target_linear_speed);
    //   if (speed_len > 0 && speed_len < (int)sizeof(speed_buf)) {
    //     //UartSendArray[4]((uint8_t*)speed_buf, (uint16_t)speed_len);
    //   }
    // }

    // vTaskDelay(10);
  }
  /* USER CODE END 5 */
}

void ComunicateTask(void *argument) {
  /* USER CODE BEGIN 5 */
  while(1)
  { 
    char com_buf[32];
    int com_len = snprintf(com_buf, sizeof(com_buf), "GET-SPEED\r\n");
    if (com_len > 0 && com_len < (int)sizeof(com_buf)) {
      UartSendArray[4]((uint8_t*)com_buf, (uint16_t)com_len);
    }
    if(control_ready) {
      sendSpeedToMotor(left_vref, right_vref);
      control_ready = false;
    }
    vTaskDelay(10);
  }
  /* USER CODE END 5 */
}