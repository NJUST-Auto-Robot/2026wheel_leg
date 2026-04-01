/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-07
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-08
 * @FilePath: fast_math_functions.h
 * @Description: 
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.   
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved. 
 */
#pragma once
#include "arm_math.h"     

   #ifdef __cplusplus
   extern "C" {
   #endif // __cplusplus

  /**
     @brief  Arc tangent in radian of y/x using sign of x and y to determine right quadrant.
     @param[in]   y  y coordinate
     @param[in]   x  x coordinate
     @param[out]  result  Result
     @return  error status.
   */
  arm_status arm_atan2_f32(float32_t y,float32_t x,float32_t *result);

  #ifdef __cplusplus
   }
   #endif // __cplusplus
