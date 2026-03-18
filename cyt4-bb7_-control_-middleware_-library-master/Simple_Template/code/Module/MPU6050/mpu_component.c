/**
    ******************************************************************************
    * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
    * @file    mpu_component.cpp
    * @author  Lutoo e19135693@163.com
    * @brief   Code for Templete.
    * @date    2021-01-09
    * @version 1.0
    * @par Change Log：
    * <table>
    * <tr><th>Date        <th>Version  <th>Author    		<th>Description
    * <tr><td>2021-01-09  <td> 1.0     <td>Lutoo      <td>Creator
    * </table>
    *
    ==============================================================================
                        ##### How to use this driver #####
    ==============================================================================
      @note
        -#  
		

      @warning
        -# 
				
    ******************************************************************************
    * @attention
    * 
    * if you had modified this file, please make sure your code does not have many 
    * bugs, update the version Number, write dowm your name and the date, the most
    * important is make sure the users will have clear and definite understanding 
    * through your new brief.
    *
    * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
    * All rights reserved.</center></h2>
    ******************************************************************************
    */

#define USE_SRML_MPU6050 1
#if USE_SRML_MPU6050

/* Includes ------------------------------------------------------------------*/
#include "zf_driver_soft_iic.h"
#include "zf_driver_timer.h"
#include "mpu_component.h"
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//IIC_PIN_Typedef MPU_IIC_PIN;
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
  * @brief  
  * @param  
  * @retval 
	*/
unsigned char MPU_I2C_WriteRegister(unsigned char slave_addr,
                              unsigned char reg_addr,
                              unsigned short len, 
                              unsigned char *data_ptr)
{
        soft_iic_info_struct soft_iic_t = {
          .scl_pin = MPU6050_SCL_PIN,
          .sda_pin = MPU6050_SDA_PIN,
          .addr = slave_addr,
          .delay = MPU6050_DELAY,
        };
        
        return soft_iic_write_8bit_registers(&soft_iic_t,reg_addr,data_ptr,len);
}

/**
  * @brief  
  * @param  
  * @retval 
	*/
unsigned char MPU_I2C_ReadRegister(unsigned char Address,
	                           unsigned char RegisterAddr, 
                             unsigned short RegisterLen,
			unsigned char *RegisterValue)
{
   soft_iic_info_struct soft_iic_t = {
    .scl_pin = MPU6050_SCL_PIN,
    .sda_pin = MPU6050_SDA_PIN,
    .addr = Address,
    .delay = MPU6050_DELAY,
  };
  
  return soft_iic_read_8bit_registers(&soft_iic_t,RegisterAddr,RegisterValue,RegisterLen);
}

/**
  * @brief  
  * @param  
  * @retval 
	*/
int get_tick_count(unsigned long *count)
{
  count[0] = timer_get(TC_TIME2_CH0);
  return 0;
}

#endif


/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

