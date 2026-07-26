/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "MachineVision/auto_get_best_threshold.h"
#include "MachineVision/find_block.h"
#include "MachineVision/find_center_line.h"
#include "Simple_PID/PID.h"

#define M7_1_TO_M7_0_DATA_LENGTH               (5)                                           // 数组数据长度

#pragma location = 0x28001000                                                   // 将下面这个数组定义到指定的RAM地址，便于其他核心直接访问(开源库默认在 0x28001000 地址保留了8kb的空间用于数据交互)
                                                                                // 此处为0x28001014的原因是前面放了一个M0的数组
float m7_1_to_m7_0_data[M7_1_TO_M7_0_DATA_LENGTH] = {0.2, 0.2, 0.2, 0.3, 0.4};                        // 定义 M7_1 演示数据数组 浮点数类型

#define M7_0_TO_M7_1_DATA_LENGTH               (5)                                           // 数组数据长度

#pragma location = 0x28002000                                                   // 将下面这个数组定义到指定的RAM地址，便于其他核心直接访问(开源库默认在 0x28001000 地址保留了8kb的空间用于数据交互)
                                                                                // 此处为0x28001014的原因是前面放了一个M0的数组
float m7_0_to_m7_1_data[M7_0_TO_M7_1_DATA_LENGTH] = {0.2, 0.2, 0.2, 0.3, 0.4};                        // 定义 M7_1 演示数据数组 浮点数类型


//wifi——spi告诉模块网络连接部分宏定义
#define WIFI_SSID_TEST          "AutoRobot_201"
#define WIFI_PASSWORD_TEST      "AuTo201#"                  // 如果需要连接的WIFI 没有密码则需要将 这里 替换为 NULL
#define TCP_TARGET_IP           "192.168.1.216"             // 连接目标的 IP
#define TCP_TARGET_PORT         "8086"                      // 连接目标的端口
#define WIFI_LOCAL_PORT         "6666"                      // 本机的端口 0：随机  可设置范围2048-65535  默认 6666

// 图像备份数组，在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
uint8 image_copy[MT9V03X_H][MT9V03X_W];
//视觉处理部分定义
uint8_t best_threshold = 0;
uint8_t last_threshold = 0;
uint8_t is_threshold_stable = 0;
uint8_t the_flag_of_camera_init = 1;
uint8_t rect_max_threshold = 160;
uint8_t rect_min_threshold = 120;
uint8_t center_x = 93;
uint8_t task_flag = 1;
//循中线pid控制变量
float line_speed_left = 0.0f;
float line_speed_right = 0.0f;

float l_speed_l_1 = 0.0f;
float l_speed_l_2 = 0.0f;
float l_speed_l_3 = 0.0f;
float l_speed_l_4 = 0.0f;

float l_speed_r_1 = 0.0f;
float l_speed_r_2 = 0.0f;
float l_speed_r_3 = 0.0f;
float l_speed_r_4 = 0.0f;
//角度pid控制变量
float is_yaw_ok = 0.0f;
float target_yaw = 0.0f;
float actual_yaw = 0.0f;
float true_yaw = 0.0f;
float have_yaw = 0.0f;
uint8_t yaw_is_stable = 0;

float a_speed_l = 0.0f;
float a_speed_r = 0.0f;
//距离pid控制变量
float is_distance_ok = 0.0f;
float target_left_distacne = 0.0f;
float actual_left_distance = 0.0f;
float last_left_distance = 0.0f;
float true_left_distance = 0.0f;

float target_right_distacne = 0.0f;
float actual_right_distance = 0.0f;
float last_right_distance = 0.0f;
float true_right_distance = 0.0f;
uint8_t left_distance_is_stable = 0;
uint8_t right_distance_is_stable = 0;

float d_speed_l = 0.0f;
float d_speed_r = 0.0f;

int main(void)
{   
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init(); 

    // 此处编写用户代码 例如外设初始化代码等
    //uart_tx_interrupt(UART_INDEX, 1);                                           // 开启 发送中断
    //uart_rx_interrupt(UART_INDEX, 1);                                           // 开启 UART_INDEX 的接收中断
    PID_Init(&center_PID_1);
    PID_Init(&center_PID_2);
    PID_Init(&center_PID_3);
    PID_Init(&center_PID_4);
    
    PID_Set(&center_PID_1, 1.14, 0.01, 0.0);
    PID_Set(&center_PID_2, 1.14, 0.01, 0.0);
    PID_Set(&center_PID_3, 1.14, 0.01, 0.0);
    PID_Set(&center_PID_4, 1.14, 0.01, 0.0);
    

    PID_Init(&Angle_PID);
    PID_Set(&Angle_PID, 1.14, 0.01, 0.0);
    
    PID_Init(&Left_Distance_PID);
    PID_Init(&Right_Distance_PID);
    PID_Set(&Left_Distance_PID, 1.14, 0.01, 0.0);
    PID_Set(&Right_Distance_PID, 1.14, 0.01, 0.0);
    
    //wifi——spi模块初始化
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST));
    //等待网络连接
    if(1 != WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                                                              // 指定使用TCP方式通讯
            TCP_TARGET_IP,                                                      // 指定远端的IP地址，填写上位机的IP地址
            TCP_TARGET_PORT,                                                    // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI_LOCAL_PORT))                                                   // 指定本机的端口号
        {
            ;
        }
    }
    //摄像头初始化
    mt9v03x_init();
    // 逐飞助手初始化 数据传输使用高速WIFI SPI
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    // 发送总钻风图像信息(仅包含原始图像信息)
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);

    
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
      // 此处编写需要循环执行的代码

        //uart_write_buffer(UART_INDEX, &send_data, 1);
      
      
      //获取从0核获得的角度数据核距离数据
      SCB_CleanInvalidateDCache_by_Addr(&m7_1_to_m7_0_data, sizeof(m7_1_to_m7_0_data));      
      is_yaw_ok = m7_0_to_m7_1_data[1];
      is_distance_ok = m7_0_to_m7_1_data[4];
      if(is_yaw_ok == 1.0f)
      {
        actual_yaw = m7_0_to_m7_1_data[0];
        is_yaw_ok = 0.0f;
      }
      if(is_distance_ok == 1.0f)
      {
        actual_left_distance = m7_0_to_m7_1_data[2];
        actual_right_distance = m7_0_to_m7_1_data[3];
        is_distance_ok = 0.0f;
      }

      if(mt9v03x_finish_flag)
     {
          mt9v03x_finish_flag = 0;

          // 在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
          memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
          //测试图传是否正常
          //GARY_TO_BINARY((uint8_t*)image_copy, MT9V03X_W, MT9V03X_H, 66);
          //Draw_Block((uint8_t*)image_copy, MT9V03X_W, MT9V03X_H, &try_block);
          if(task_flag == 1)
          {
            ///////////////////////////////////////////////////////////////////////////////角度距离pid///////////////////////////////////////////////////////////////////////////////////////
            /*target_yaw = 45.0f;
            if(have_yaw != 0.0f)
            {
              //此处用于判断是否稳定，可用于结束角度pid控制——连续20次目标角度和实际角度的误差小于1度认为稳定
              if(fabsf(target_yaw - true_yaw) < 1.0f && yaw_is_stable < 20)
              {
                yaw_is_stable++;
              }
              else if(fabsf(target_yaw - true_yaw) > 1.0f && yaw_is_stable < 20)
              {
                yaw_is_stable = 0;
              }
              
              true_yaw = actual_yaw - have_yaw;//设想的角度pid控制是以自身当前初始姿态为0度，控制转过target_yaw度
              
              a_speed_l = PID_Control_Angles_And_Distance(&Angle_PID, target_yaw, true_yaw, ANGLE_PID);//pid计算
              a_speed_r = -a_speed_l;
              
              m7_1_to_m7_0_data[0] = 0.6 + a_speed_l;//传速度值给0核
              m7_1_to_m7_0_data[1] = 0.6 + a_speed_r;
              SCB_CleanInvalidateDCache_by_Addr(&m7_1_to_m7_0_data, sizeof(m7_1_to_m7_0_data));
            }
            else
            {
              have_yaw = actual_yaw;
            }*/
            ///////////////////////////////////////////////////////////////////////////////左轮距离pid///////////////////////////////////////////////////////////////////////////////////////
            target_left_distacne = 5.0f;
            if(last_left_distance != 0.0f)
            {
              //此处用于判断是否稳定，可用于结束左轮距离pid控制——连续20次目标距离和实际距离的误差小于0.1认为稳定
              if(fabsf(target_left_distacne - true_left_distance) < 0.1f && left_distance_is_stable < 20)
              {
                left_distance_is_stable++;
              }
              else if(fabsf(target_left_distacne - true_left_distance) > 0.1f && left_distance_is_stable < 20)
              {
                left_distance_is_stable = 0;
              }
              
              true_left_distance += actual_left_distance - last_left_distance;//设想的距离pid控制是以自身当前初始位置为起点，控制转过target_left_distacne度
              last_left_distance = actual_left_distance;
              
              d_speed_l = PID_Control_Angles_And_Distance(&Left_Distance_PID, target_left_distacne, true_left_distance, DISTANCE_PID);//pid计算
              
            }
            else
            {
              last_left_distance = actual_left_distance;
            }
            ///////////////////////////////////////////////////////////////////////////////右轮距离pid///////////////////////////////////////////////////////////////////////////////////////
            target_right_distacne = 5.0f;
            if(last_right_distance != 0.0f)
            {
              //此处用于判断是否稳定，可用于结束角度pid控制——连续20次目标角度和实际角度的误差小于1度认为稳定
              if(fabsf(target_right_distacne - true_right_distance) < 0.1f && right_distance_is_stable < 20)
              {
                right_distance_is_stable++;
              }
              else if(fabsf(target_right_distacne - true_right_distance) > 0.1f && right_distance_is_stable < 20)
              {
                right_distance_is_stable = 0;
              }
              
              true_right_distance += actual_right_distance - last_right_distance;//设想的角度pid控制是以自身当前初始姿态为0度，控制转过target_yaw度
              last_right_distance = actual_right_distance;
              
              d_speed_r = PID_Control_Angles_And_Distance(&Right_Distance_PID, target_right_distacne, true_right_distance, DISTANCE_PID);//pid计算
              
            }
            else
            {
              last_right_distance = actual_right_distance;
            }
              m7_1_to_m7_0_data[0] = 0.2 + d_speed_l;//传速度值给0核
              m7_1_to_m7_0_data[1] = 0.2 + d_speed_r;//传速度值给0核
              SCB_CleanInvalidateDCache_by_Addr(&m7_1_to_m7_0_data, sizeof(m7_1_to_m7_0_data));
          }
          
          
          //用于科目一找障碍
          //Find_Block_Pro_Max((uint8_t*)image_copy, MT9V03X_W, MT9V03X_H, &block, 170, 150, 70, 40, 48, 40);
          //GARY_TO_BINARY_Pro((uint8_t*)image_copy, MT9V03X_W, MT9V03X_H, 170, 150);
          //image_copy[block.cy[0]][block.cx[0]] = 0;
          //Draw_Max_Block((uint8_t*)image_copy, MT9V03X_W, MT9V03X_H, &block);
          //Draw_Merge_Block((uint8_t*)image_copy, MT9V03X_W, MT9V03X_H, &block);
          //Draw_Block((uint8_t*)image_copy, MT9V03X_W, MT9V03X_H, &block);
          
          //用于科目二行驶至矩形框内合适位置
          else if(task_flag == 2)
          {
            find_center_line((uint8_t*)image_copy, MT9V03X_W, MT9V03X_H, &line, rect_max_threshold, rect_min_threshold);
            
            l_speed_l_1 = PID_Control(&center_PID_1, center_x, line.center_line_center_x[0]);
            l_speed_l_2 = PID_Control(&center_PID_2, center_x, line.center_line_center_x[10]);
            l_speed_l_3 = PID_Control(&center_PID_3, center_x, line.center_line_center_x[20]);
            l_speed_l_4 = PID_Control(&center_PID_4, center_x, line.center_line_center_x[30]);
            
            l_speed_r_1 = -l_speed_l_1;
            l_speed_r_2 = -l_speed_l_2;
            l_speed_r_3 = -l_speed_l_3;
            l_speed_r_4 = -l_speed_l_4;
            
            line_speed_left = 0.6 + (l_speed_l_1 + l_speed_l_2 + l_speed_l_3 + l_speed_l_4) / 4;
            line_speed_right = 0.6 + (l_speed_r_1 + l_speed_r_2 + l_speed_r_3 + l_speed_r_4) / 4;
            
            m7_1_to_m7_0_data[0] = line_speed_left;
            m7_1_to_m7_0_data[1] = line_speed_right;
            SCB_CleanInvalidateDCache_by_Addr(&m7_1_to_m7_0_data, sizeof(m7_1_to_m7_0_data));
            
            image_copy[line.top_point_y][94] = 255;
            draw_center_line((uint8_t*)image_copy, MT9V03X_W, MT9V03X_H, &line);
            
            if(line.top_point_y >= 55 && line.top_point_y <= 65)
              task_flag = 3;
          }
          else if(task_flag == 3)
          {
            m7_1_to_m7_0_data[0] = 0.3;
            m7_1_to_m7_0_data[1] = 0.3;
            SCB_CleanInvalidateDCache_by_Addr(&m7_1_to_m7_0_data, sizeof(m7_1_to_m7_0_data));
          }
          // 发送图像
          seekfree_assistant_camera_send();

          //SCB_CleanInvalidateDCache_by_Addr(&m7_1_to_m7_0_data, sizeof(m7_1_to_m7_0_data));
      }
    }
}

// **************************** 代码区域 ****************************
uint8_t Angle_Control(float target)
{
    target_yaw = target;
    if(have_yaw != 0.0f)
    {
      //此处用于判断是否稳定，可用于结束角度pid控制——连续20次目标角度和实际角度的误差小于1度认为稳定
      if(fabsf(target_yaw - true_yaw) < 1.0f && yaw_is_stable < 20)
      {
        yaw_is_stable++;
      }
      else if(fabsf(target_yaw - true_yaw) > 1.0f && yaw_is_stable < 20)
      {
        yaw_is_stable = 0;
      }
      
      true_yaw = actual_yaw - have_yaw;//设想的角度pid控制是以自身当前初始姿态为0度，控制转过target_yaw度
      
      a_speed_l = PID_Control_Angles_And_Distance(&Angle_PID, target_yaw, true_yaw, ANGLE_PID);//pid计算
      a_speed_r = -a_speed_l;
      
      m7_1_to_m7_0_data[0] = a_speed_l;//传速度值给0核
      m7_1_to_m7_0_data[1] = a_speed_r;
      SCB_CleanInvalidateDCache_by_Addr(&m7_1_to_m7_0_data, sizeof(m7_1_to_m7_0_data));
    }
    else
    {
      have_yaw = actual_yaw;//获取本次角度pid控制的初始姿态角度，如果进行新的初始姿态的角度pid控制应该额外清零
    }
    
    return yaw_is_stable;
}

uint8_t Distance_Control(float left_target, float right_target)
{
    target_left_distacne = left_target;
    if(last_left_distance != 0.0f)
    {
      //此处用于判断是否稳定，可用于结束距离pid控制——连续20次目标距离和实际距离的误差小于0.1认为稳定
      if(fabsf(target_left_distacne - true_left_distance) < 0.1f && left_distance_is_stable < 20)
      {
        left_distance_is_stable++;
      }
      else if(fabsf(target_left_distacne - true_left_distance) > 0.1f && left_distance_is_stable < 20)
      {
        left_distance_is_stable = 0;
      }
      
      true_left_distance += actual_left_distance - last_left_distance;//设想的距离pid控制是以自身当前初始位置为起点，控制转过target_left_distacne度
      last_left_distance = actual_left_distance;
      
      d_speed_l = PID_Control_Angles_And_Distance(&Right_Distance_PID, target_left_distacne, true_left_distance, DISTANCE_PID);//pid计算
      
      m7_1_to_m7_0_data[0] = d_speed_l;//传速度值给0核
    }
    else
    {
      last_left_distance = actual_left_distance;
    }
    
    target_right_distacne = right_target;
    if(last_right_distance != 0.0f)
    {
      //此处用于判断是否稳定，可用于结束距离pid控制——连续20次目标距离和实际距离的误差小于0.1认为稳定
      if(fabsf(target_right_distacne - true_right_distance) < 0.1f && right_distance_is_stable < 20)
      {
        right_distance_is_stable++;
      }
      else if(fabsf(target_right_distacne - true_right_distance) > 0.1f && right_distance_is_stable < 20)
      {
        right_distance_is_stable = 0;
      }
      
      true_right_distance += actual_right_distance - last_right_distance;//设想的距离pid控制是以自身当前初始位置为起点，控制转过target_left_distacne度
      last_right_distance = actual_right_distance;
      
      d_speed_r = PID_Control_Angles_And_Distance(&Right_Distance_PID, target_right_distacne, true_right_distance, DISTANCE_PID);//pid计算
      
      m7_1_to_m7_0_data[1] = d_speed_r;//传速度值给0核
    }
    else
    {
      last_right_distance = actual_right_distance;
    }  
    SCB_CleanInvalidateDCache_by_Addr(&m7_1_to_m7_0_data, sizeof(m7_1_to_m7_0_data));
    
    return 1;
}
