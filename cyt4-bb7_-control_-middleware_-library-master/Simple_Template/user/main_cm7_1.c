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

#define DATA_LENGTH               (5)                                           // 数组数据长度

#pragma location = 0x28001000                                                   // 将下面这个数组定义到指定的RAM地址，便于其他核心直接访问(开源库默认在 0x28001000 地址保留了8kb的空间用于数据交互)
                                                                                // 此处为0x28001014的原因是前面放了一个M0的数组
float m7_1_data[DATA_LENGTH] = {0, 0.1, 0.2, 0.3, 0.4};                        // 定义 M7_1 演示数据数组 浮点数类型

//wifi——spi告诉模块网络连接部分宏定义
#define WIFI_SSID_TEST          "AutoRobot_201"
#define WIFI_PASSWORD_TEST      "AuTo201#"                  // 如果需要连接的WIFI 没有密码则需要将 这里 替换为 NULL
#define TCP_TARGET_IP           "192.168.1.118"             // 连接目标的 IP
#define TCP_TARGET_PORT         "8086"                      // 连接目标的端口
#define WIFI_LOCAL_PORT         "6666"                      // 本机的端口 0：随机  可设置范围2048-65535  默认 6666

// 图像备份数组，在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
uint8 image_copy[MT9V03X_H][MT9V03X_W];
//视觉处理部分定义
uint8_t best_threshold = 0;
uint8_t last_threshold = 0;
uint8_t is_threshold_stable = 0;
uint8_t the_flag_of_camera_init = 1;
//串口发送变量

uint8_t send_data[1] = {48};                                                            // 接收数据变量                                                        // 接收数据变量
void my_ipc_callback(uint32 receive_data)
{
    
}
int main(void)
{   
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init(); 


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
      //system_delay_ms(1000);   

      
       // M7_1核心有Dcache 当数据有变化时应该更新Dcache的内容 否则数据无法同步到RAM(其他核心访问的RAM地址也就无法读取到数据)
      
      
      if(mt9v03x_finish_flag)
      {
          mt9v03x_finish_flag = 0;

          // 在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
          memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
          //GARY_TO_BINARY((uint8_t*)image_copy, MT9V03X_W, MT9V03X_H, 66);
          //Draw_Block((uint8_t*)image_copy, MT9V03X_W, MT9V03X_H, &try_block);
          //Find_Block_Pro_Max((uint8_t*)image_copy, MT9V03X_W, MT9V03X_H, &block, 240, 190, 80, 60, 40, 40);
          //Draw_Max_Block((uint8_t*)image_copy, MT9V03X_W, MT9V03X_H, &block);
          //Draw_Merge_Block((uint8_t*)image_copy, MT9V03X_W, MT9V03X_H, &block);
          // 发送图像
          seekfree_assistant_camera_send();
          for(int i = 0; i < DATA_LENGTH; i ++)                                   // M7_1数据自增 步进值0.1
          {
            m7_1_data[i] += 0.1;
          }
          SCB_CleanInvalidateDCache_by_Addr(&m7_1_data, sizeof(m7_1_data));
      }
     
    }
}

// **************************** 代码区域 ****************************
