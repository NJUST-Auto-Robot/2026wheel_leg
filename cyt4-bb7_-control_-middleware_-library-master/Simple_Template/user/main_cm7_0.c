/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-02
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-05
 * @FilePath: main_cm7_0.c
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 *
 *
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *            佛祖保佑     永不宕机     永无BUG
 */
#include "System/usr_uart.hpp"
#include "zf_common_headfile.h"
#include <math.h>
// 任务所需包含头文件
#include "Module/IMU660RB/imu_data.h"
#include "Controller/LQR.h"
#include "Controller/Motor.h"
#include "Controller/VMC.h"
#include "Controller/CRSF.h"
#include "Controller/CRC8.h"
uint8_t debug_rx_buff[1] = {0};

// 外设宏定义
#define TEST_LED (P19_0)
bool debug_print_f = false;
float left_u_balance = 0.0f; // 控制输入变量
float right_u_balance = 0.0f; // 控制输入变量
float left_vref = 0.0f; // 左轮速度参考值
float right_vref = 0.0f; // 右轮速度参考值
bool control_ready = false;

// 自定义类变量
//USR_SYSTEM usr_sys;

// 全局变量：目标线速度 (单位 m/s)，供 ControlTask 和 VMCTask 共用
float target_linear_speed_l = 0.0f; 
float target_linear_speed_r = 0.0f; 
float target_linear_speed = 0.0f; 

// 用于存储估算的车身高度 (单位 m)
static float current_height = 0.0f; 
static float vertical_velocity = 0.0f; // 垂直速度估算

#define RAD_PER_DEG (3.14159265358979323846f / 180.0f)

typedef struct {
    float pos;
    float vel;
    float P[2][2];
    float Q_pos;
    float Q_vel;
    float R_pos;
    float R_vel;
} BodyEkf_t;

static BodyEkf_t body_ekf = {
    .pos    = 0.0f,
    .vel    = 0.0f,
    .P      = {{0.05f, 0.0f}, {0.0f, 1.0f}},
    .Q_pos  = 0.001f,
    .Q_vel  = 0.05f,
    .R_pos  = 0.05f,
    .R_vel  = 0.1f,
};

static void BodyEkfInit(float initial_pos, float initial_vel) {
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

static void BodyEkfUpdate(float position_measure, float velocity_measure, float dt) {
    float x_pred_pos = body_ekf.pos + body_ekf.vel * dt;
    float x_pred_vel = body_ekf.vel;

    float p00 = body_ekf.P[0][0] + dt * (body_ekf.P[1][0] + body_ekf.P[0][1]) + dt * dt * body_ekf.P[1][1] + body_ekf.Q_pos;
    float p01 = body_ekf.P[0][1] + dt * body_ekf.P[1][1];
    float p10 = body_ekf.P[1][0] + dt * body_ekf.P[1][1];
    float p11 = body_ekf.P[1][1] + body_ekf.Q_vel;

    float y0 = position_measure - x_pred_pos;
    float y1 = velocity_measure - x_pred_vel;

    float S00 = p00 + body_ekf.R_pos;
    float S01 = p01;
    float S10 = p10;
    float S11 = p11 + body_ekf.R_vel;

    float det = S00 * S11 - S01 * S10;
    if (det == 0.0f) {
        det = 1e-6f;
    }

    float invS00 = S11 / det;
    float invS01 = -S01 / det;
    float invS10 = -S10 / det;
    float invS11 = S00 / det;

    float K00 = p00 * invS00 + p01 * invS10;
    float K01 = p00 * invS01 + p01 * invS11;
    float K10 = p10 * invS00 + p11 * invS10;
    float K11 = p10 * invS01 + p11 * invS11;

    body_ekf.pos = x_pred_pos + K00 * y0 + K01 * y1;
    body_ekf.vel = x_pred_vel + K10 * y0 + K11 * y1;

    body_ekf.P[0][0] = (1.0f - K00) * p00 - K01 * p10;
    body_ekf.P[0][1] = (1.0f - K00) * p01 - K01 * p11;
    body_ekf.P[1][0] = -K10 * p00 + (1.0f - K11) * p10;
    body_ekf.P[1][1] = -K10 * p01 + (1.0f - K11) * p11;
}

  // 定义 remap 宏，用于线性映射
  #define REMAP_VALUE(val, in_min, in_max, out_min, out_max) \
      ((float)(val - in_min) * (float)(out_max - out_min) / (float)(in_max - in_min) + (float)out_min)

int main(void) {
  clock_init(SYSTEM_CLOCK_250M);  // 时钟配置及系统初始化<务必保留>
  debug_init();                   // 调试串口信息初始化
  uart_rx_interrupt(DEBUG_UART_INDEX, 1);                                           // 开启 UART_INDEX 的接收中断
  usrUartInit();

  // 设置周期中断1ms，用于串口空闲中断判断
  pit_us_init(PIT_CH0, 1000);

  // 初始化 TEST_LED 输出 默认高电平 推挽输出模式
  gpio_init(TEST_LED, GPO, GPIO_LOW, GPO_PUSH_PULL);

  // 初始化IMU660RB
  //imu_init(&imu660rb);

  // 计算LQR增益矩阵
  LQR_ComputeK();                            

  // 速度 PID（线速度闭环）参数（宏定义）
  MotorPID_Init(MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_I_TERM_MAX, MOTOR_PID_OUT_MAX);

  Crc8_init(0xD5);
  
  //usrSystemInit();                // 用户系统初始化 包括外设和任务创建
  //osKernelInitialize();           // 初始化FreeRTOS内核
  //osKernelStart();                // 开启FreeRTOS内核调 
  
  while (true) {
    /*假如FreeRTOS调度成功，那么不会运行这里面的代码*/
    
    // 在这里可以添加平衡控制的代码，例如使用LQR算法计算控制输入，并通过PWM输出控制电机
    float x_l_ref[] = {0.0f, -target_linear_speed_l, 3.65f*3.1715f/180.0f, 0.0f}; // 目标状态向量
    float x_r_ref[] = {0.0f, -target_linear_speed_r, 3.65f*3.1715f/180.0f, 0.0f}; // 目标状态向量
    float left_x_current[] = {left_Wheel_position, right_Wheel_Speed, imu660rb.angles.pitch*3.1415f/180.0f, imu660rb.angles.pitch_acc*3.1415f/180.0f}; // 当前状态向量
    float right_x_current[] = {right_Wheel_position, left_Wheel_Speed, imu660rb.angles.pitch*3.1415f/180.0f, imu660rb.angles.pitch_acc*3.1415f/180.0f}; // 当前状态向量
    float left_x_error[4];
    float right_x_error[4];
  
    for (int i = 0; i < 4; i++) {
        left_x_error[i]  = left_x_current[i]  - x_l_ref[i]; // 计算状态误差
        right_x_error[i] = right_x_current[i] - x_r_ref[i]; // 计算状态误差
    }
    left_vref  =  K[0] * left_x_error[0]  - K[1] * left_x_error[1]  - K[2] * left_x_error[2]  - K[3] * left_x_error[3]; // 计算平衡扭矩
    right_vref =  K[0] * right_x_error[0] - K[1] * right_x_error[1] - K[2] * right_x_error[2] - K[3] * right_x_error[3];; // 计算右轮速度参考值
    control_ready= true; // 控制准备就绪
    
    static bool init_flag = false;
    if(init_flag == false) {
      // 发送初始化命令到电机驱动器
      char init_buf[32];
      int init_len = snprintf(init_buf, sizeof(init_buf), "SET-ANGLE-ZERO\r\n");
      if (init_len > 0 && init_len < (int)sizeof(init_buf)) {
        UartSendArray[4]((uint8_t*)init_buf, (uint16_t)init_len);
      }
      init_flag = true;
    }
    char com_buf[32];
    int com_len = snprintf(com_buf, sizeof(com_buf), "GET-SPEED\r\n");
    if (com_len > 0 && com_len < (int)sizeof(com_buf)) {
      UartSendArray[4]((uint8_t*)com_buf, (uint16_t)com_len);
    }
    if(control_ready) {
      sendSpeedToMotor(left_vref, right_vref);
      control_ready = false;
    }
  }
}
