#include "Simple_PID/Base_Control.h"

#pragma location = 0x28001000                                                   // 将下面这个数组定义到指定的RAM地址，便于其他核心直接访问(开源库默认在 0x28001000 地址保留了8kb的空间用于数据交互)
                                                                                // 此处为0x28001014的原因是前面放了一个M0的数组
float m7_1_to_m7_0_data[M7_1_TO_M7_0_DATA_LENGTH] = {-0.05f, 0.85f, 0.2, 0.3, 0.4}; 

#pragma location = 0x28002000                                                   // 将下面这个数组定义到指定的RAM地址，便于其他核心直接访问(开源库默认在 0x28001000 地址保留了8kb的空间用于数据交互)
                                                                                // 此处为0x28001014的原因是前面放了一个M0的数组
float m7_0_to_m7_1_data[M7_0_TO_M7_1_DATA_LENGTH] = {-0.05f, 0.85f, 0.2, 0.3, 0.4};    

//角度pid控制变量
float is_yaw_ok = 0.0f;
float target_yaw = 0.0f;
float actual_yaw = 0.0f;
float true_yaw = 0.0f;
float have_yaw = 0.0f;
uint8_t yaw_is_stable = 0;

float yaw_now = 0;

float a_speed_l = 0.0f;
float a_speed_r = 0.0f;

//距离pid控制变量
float is_distance_ok = 0.0f;

uint8_t distance_control_flag = 0;

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

float d_speed_l = 0.6f;
float d_speed_r = 0.6f;

float delta_speed_l = 0.0f;
float delta_speed_r = 0.0f;

uint8_t integral_l_num = 0;
uint8_t integral_r_num = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t turn_to_true_yaw = 0;
//结合GPS使用的路径规划所需要的数组
//[0][0]、[0][1]为起始出发点的纬度、经度
/***********10个点，点与点距离、点与原点角度只有9个值，所以distance[0]、azimuth[0]、yaw[0]无意义*********/
float stage_1_point[10][2] = {{11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}};
//点到点之间的距离
float stage_1_distance[10] = {0};
//点到原点的世界坐标系的角度
float stage_1_azimuth[10] = {0};
//[0][0]、[0][1]为起始出发点的纬度、经度
float stage_2_point[10][2] = {{11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}, {11.4, 5.14}};
//点到点之间的距离
float stage_2_distance[10] = {0};
//点到原点的世界坐标系的角度
float stage_2_azimuth[10] = {0};
//正北方向的yaw角
float yaw_offset = 114.5;
//实际使用的yaw角
float stage_1_yaw[10] = {0};
float stage_2_yaw[10] = {0};

///////////////////////////////////////////////////////////////////////////////角度距离pid///////////////////////////////////////////////////////////////////////////////////////
//该函数放回的是目标角度和实际角度的差值连续小于1度的次数，可以根据该值结束角度pid控制，但进行下一次角度pid调控时应该将变量have_yaw清零
//传入想要转过的角度、基本行驶左轮速度、基本行驶右轮速度
uint8_t Angle_Control(float target, float base_speed_l, float base_speed_r)
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
      
      m7_1_to_m7_0_data[0] = base_speed_l + a_speed_l;//传速度值给0核
      m7_1_to_m7_0_data[1] = base_speed_r + a_speed_r;
      SCB_CleanInvalidateDCache_by_Addr(&m7_1_to_m7_0_data, sizeof(m7_1_to_m7_0_data));
    }
    else
    {
      have_yaw = actual_yaw;//获取本次角度pid控制的初始姿态角度，如果进行新的初始姿态的角度pid控制应该额外清零
    }
    
    return yaw_is_stable;
}
//该函数放回的使目标角度和实际角度的差值连续小于1度的次数，可以根据该值结束角度pid控制，但进行下一次角度直接调用即可
//传入想要转到的角度、基本行驶左轮速度、基本行驶右轮速度
uint8_t Angle_Control_2(float target, float base_speed_l, float base_speed_r)
{
    target_yaw = target;
      //此处用于判断是否稳定，可用于结束角度pid控制——连续20次目标角度和实际角度的误差小于1度认为稳定
    if(fabsf(target_yaw - actual_yaw) < 1.0f && yaw_is_stable < 20)
    {
      yaw_is_stable++;
    }
    else if(fabsf(target_yaw - actual_yaw) > 1.0f && yaw_is_stable < 20)
    {
      yaw_is_stable = 0;
    }
        
    a_speed_l = PID_Control_Angles_And_Distance(&Angle_PID, target_yaw, actual_yaw, ANGLE_PID);//pid计算
    a_speed_r = -a_speed_l;
    
    m7_1_to_m7_0_data[0] = base_speed_l + a_speed_l;//传速度值给0核
    m7_1_to_m7_0_data[1] = base_speed_r + a_speed_r;
    SCB_CleanInvalidateDCache_by_Addr(&m7_1_to_m7_0_data, sizeof(m7_1_to_m7_0_data));
    
    return yaw_is_stable;
}

///////////////////////////////////////////////////////////////////////////////距离控制///////////////////////////////////////////////////////////////////////////////////////
//此函数实现行驶到目标距离后将速度线性变化为终点速度，当速度到达终点速度时会清零相关变量并返回1，可根据该值进行下一次调控，且下一次调控于角度pid控制不同，无需进行变量清零
//传入所要左轮行驶的距离、右轮行驶的距离、左轮行驶基本速度、右轮行驶基本速度、终点左轮速度、右轮速度
uint8_t Distance_Control(float left_target, float right_target, float base_speed_l, float base_speed_r, float end_speed_l, float end_speed_r)
{
    target_left_distacne = left_target;
    
    
    if(last_left_distance != 0.0f)
    {
      //此处用于判断是否接近终点，如果接近终点则进行简单线性速度变化使速度达到终点速度，没有则继续进行接近终点的行驶
      if(true_left_distance >= 0.8f * target_left_distacne)//修改0.8或者优化判断是否行驶至目标距离的判断逻辑可以进行更准确的行驶
      {
        if(delta_speed_l == 0.0f)
        {
          delta_speed_l = d_speed_l - end_speed_l;
        }
        if(integral_l_num < 100)//通过100次的线性速度变化使速度到达终点速度，可以修改100以及与之对应的0.01f使得线性累加速度变化更快完成
        {
          integral_l_num++;
          d_speed_l -= 0.01f * delta_speed_l;
        }
      }
      else
      {
        true_left_distance += actual_left_distance - last_left_distance;//设想的距离pid控制是以自身当前初始位置为起点，控制转过target_left_distacne度
        last_left_distance = actual_left_distance;
      }
    }
    else
    {
      d_speed_l = base_speed_l;
      
      last_left_distance = actual_left_distance;
    }
    
    target_right_distacne = right_target;
    
    
    if(last_right_distance != 0.0f)
    {
      //此处用于判断是否接近终点，如果接近终点则进行简单线性速度变化使速度达到终点速度，没有则继续进行接近终点的行驶
      if(true_right_distance >= 0.8f * target_right_distacne)
      {
        if(delta_speed_r == 0.0f)
        {
          delta_speed_r = d_speed_r - end_speed_r;
        }
        if(integral_r_num < 100)//通过100次的线性速度变化使速度到达终点速度，可以修改100以及与之对应的0.01f使得线性累加速度变化更快完成
        {
          integral_r_num++;
          d_speed_r -= 0.01f * delta_speed_r;
        }
      }
      else
      {
        true_right_distance += actual_right_distance - last_right_distance;//设想的距离pid控制是以自身当前初始位置为起点，控制转过target_left_distacne度
        last_right_distance = actual_right_distance;
      }
    }
    else
    {
      d_speed_r = base_speed_r;
      
      last_right_distance = actual_right_distance;
    }
    
    m7_1_to_m7_0_data[0] = d_speed_l;//传速度值给0核
    m7_1_to_m7_0_data[1] = d_speed_r;//传速度值给0核
    SCB_CleanInvalidateDCache_by_Addr(&m7_1_to_m7_0_data, sizeof(m7_1_to_m7_0_data));
    
    //进行全局变量复位和是否完成单次距离控制进行返回值，成功返回1，失败返回0，便于下一次距离控制
    if(integral_r_num == 100 && integral_l_num == 100)
    {
      integral_l_num = 0;
      integral_r_num = 0;
      
      delta_speed_l = 0;
      delta_speed_r = 0;
      
      last_left_distance = 0;
      last_right_distance = 0;
      
      true_left_distance = 0;
      true_right_distance = 0;
        
      return 1;
    }
    else
    {
      return 0;
    }
    
}
//此函数在角度和距离控制的基础上进行整合从而实现按照指定角度行驶指定距离，核心想法是行驶距离没有接近目标距离时以角度调控为主，如果接近目标距离则进行最后的线性速度变化到达目标速度，返回值和距离调控一致
//传入参数为角度调控和距离调控参数的结合
uint8_t Angle_And_Distance_Control(float target, float base_speed_l, float base_speed_r, float left_target, float right_target, float end_speed_l, float end_speed_r)
{
  uint8_t is_last_stage = 0;//用于判断传给0核的速度是角度pid的速度还是距离控制的速度
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    target_yaw = target;
    if(have_yaw != 0.0f)
    {
      //此处用于判断是否稳定，可用于结束角度pid控制——连续20次目标角度和实际角度的误差小于1度认为稳定      
      true_yaw = actual_yaw - have_yaw;//设想的角度pid控制是以自身当前初始姿态为0度，控制转过target_yaw度
      
      a_speed_l = PID_Control_Angles_And_Distance(&Angle_PID, target_yaw, true_yaw, ANGLE_PID);//pid计算
      a_speed_r = -a_speed_l;
    }
    else
    {
      have_yaw = actual_yaw;//获取本次角度pid控制的初始姿态角度，如果进行新的初始姿态的角度pid控制应该额外清零
    }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    target_left_distacne = left_target;
    if(last_left_distance != 0.0f)
    {
      //此处用于判断是否接近终点，如果接近终点则进行简单线性速度变化使速度达到终点速度，没有则继续进行接近终点的行驶
      if(true_left_distance >= 0.8f * target_left_distacne)
      {
        is_last_stage++;
        if(delta_speed_l == 0.0f)
        {
          delta_speed_l = d_speed_l - end_speed_l;
        }
        if(integral_l_num < 100)//通过100次的线性速度变化使速度到达终点速度，可以修改100以及与之对应的0.01f使得线性累加速度变化更快完成
        {
          integral_l_num++;
          d_speed_l -= 0.01f * delta_speed_l;
        }
      }
      else
      {
        true_left_distance += actual_left_distance - last_left_distance;//设想的距离pid控制是以自身当前初始位置为起点，控制转过target_left_distacne度
        last_left_distance = actual_left_distance;
      }
    }
    else
    {
      d_speed_l = base_speed_l;

      last_left_distance = actual_left_distance;
    }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    target_right_distacne = right_target;
    if(last_right_distance != 0.0f)
    {
      //此处用于判断是否接近终点，如果接近终点则进行简单线性速度变化使速度达到终点速度，没有则继续进行接近终点的行驶
      if(true_right_distance >= 0.8f * target_right_distacne)
      {
        is_last_stage++;
        if(delta_speed_r == 0.0f)
        {
          delta_speed_r = d_speed_r - end_speed_r;
        }
        if(integral_r_num < 100)//通过100次的线性速度变化使速度到达终点速度，可以修改100以及与之对应的0.01f使得线性累加速度变化更快完成
        {
          integral_r_num++;
          d_speed_r -= 0.01f * delta_speed_r;
        }
      }
      else
      {
        true_right_distance += actual_right_distance - last_right_distance;//设想的距离pid控制是以自身当前初始位置为起点，控制转过target_left_distacne度
        last_right_distance = actual_right_distance;
      }
    }
    else
    {
      d_speed_r = base_speed_r;

      last_right_distance = actual_right_distance;
    }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(is_last_stage == 2)
    {
      m7_1_to_m7_0_data[0] = d_speed_l;//传速度值给0核
      m7_1_to_m7_0_data[1] = d_speed_r;//传速度值给0核
      SCB_CleanInvalidateDCache_by_Addr(&m7_1_to_m7_0_data, sizeof(m7_1_to_m7_0_data));
    }
    else
    {
      m7_1_to_m7_0_data[0] = base_speed_l + a_speed_l;//传速度值给0核
      m7_1_to_m7_0_data[1] = base_speed_r + a_speed_r;
      SCB_CleanInvalidateDCache_by_Addr(&m7_1_to_m7_0_data, sizeof(m7_1_to_m7_0_data));
    }
    
    //进行全局变量复位和是否完成单次距离控制进行返回值，成功返回1，失败返回0，便于下一次距离控制
    if(integral_r_num == 100 && integral_l_num == 100)
    {
      integral_l_num = 0;
      integral_r_num = 0;
      
      delta_speed_l = 0;
      delta_speed_r = 0;
      
      true_left_distance = 0;
      true_right_distance = 0;
        
      last_left_distance = 0;
      last_right_distance = 0;
      
      have_yaw = 0;
      
      return 1;
    }
    else
    {
      return 0;
    }
}

//此函数在角度和距离控制的基础上进行整合从而实现按照指定角度行驶指定距离，核心想法是行驶距离没有接近目标距离时以角度调控为主，如果接近目标距离则进行最后的线性速度变化到达目标速度，返回值和距离调控一致
//传入参数为角度调控和距离调控参数的结合
uint8_t Angle_And_Distance_Control_2(float target, float base_speed_l, float base_speed_r, float left_target, float right_target, float end_speed_l, float end_speed_r)
{
  uint8_t is_last_stage = 0;//用于判断传给0核的速度是角度pid的速度还是距离控制的速度
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    target_yaw = target;
    //此处用于判断是否稳定，可用于结束角度pid控制——连续20次目标角度和实际角度的误差小于1度认为稳定      
      
    a_speed_l = PID_Control_Angles_And_Distance(&Angle_PID, target_yaw, actual_yaw, ANGLE_PID);//pid计算
    a_speed_r = -a_speed_l;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    target_left_distacne = left_target;
    if(last_left_distance != 0.0f)
    {
      //此处用于判断是否接近终点，如果接近终点则进行简单线性速度变化使速度达到终点速度，没有则继续进行接近终点的行驶
      if(true_left_distance >= 0.8f * target_left_distacne)
      {
        is_last_stage++;
        if(delta_speed_l == 0.0f)
        {
          delta_speed_l = d_speed_l - end_speed_l;
        }
        if(integral_l_num < 100)//通过100次的线性速度变化使速度到达终点速度，可以修改100以及与之对应的0.01f使得线性累加速度变化更快完成
        {
          integral_l_num++;
          d_speed_l -= 0.01f * delta_speed_l;
        }
      }
      else
      {
        true_left_distance += actual_left_distance - last_left_distance;//设想的距离pid控制是以自身当前初始位置为起点，控制转过target_left_distacne度
        last_left_distance = actual_left_distance;
      }
    }
    else
    {
      d_speed_l = base_speed_l;

      last_left_distance = actual_left_distance;
    }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    target_right_distacne = right_target;
    if(last_right_distance != 0.0f)
    {
      //此处用于判断是否接近终点，如果接近终点则进行简单线性速度变化使速度达到终点速度，没有则继续进行接近终点的行驶
      if(true_right_distance >= 0.8f * target_right_distacne)
      {
        is_last_stage++;
        if(delta_speed_r == 0.0f)
        {
          delta_speed_r = d_speed_r - end_speed_r;
        }
        if(integral_r_num < 100)//通过100次的线性速度变化使速度到达终点速度，可以修改100以及与之对应的0.01f使得线性累加速度变化更快完成
        {
          integral_r_num++;
          d_speed_r -= 0.01f * delta_speed_r;
        }
      }
      else
      {
        true_right_distance += actual_right_distance - last_right_distance;//设想的距离pid控制是以自身当前初始位置为起点，控制转过target_left_distacne度
        last_right_distance = actual_right_distance;
      }
    }
    else
    {
      d_speed_r = base_speed_r;

      last_right_distance = actual_right_distance;
    }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(is_last_stage == 2)
    {
      m7_1_to_m7_0_data[0] = d_speed_l;//传距离调控速度值给0核
      m7_1_to_m7_0_data[1] = d_speed_r;//传距离调控速度值给0核
      SCB_CleanInvalidateDCache_by_Addr(&m7_1_to_m7_0_data, sizeof(m7_1_to_m7_0_data));
    }
    else
    {
      m7_1_to_m7_0_data[0] = base_speed_l + a_speed_l;//传角度调控速度值给0核
      m7_1_to_m7_0_data[1] = base_speed_r + a_speed_r;//传角度调控速度值给0核
      SCB_CleanInvalidateDCache_by_Addr(&m7_1_to_m7_0_data, sizeof(m7_1_to_m7_0_data));
    }
    
    //进行全局变量复位和是否完成单次距离控制进行返回值，成功返回1，失败返回0，便于下一次距离控制
    if(integral_r_num == 100 && integral_l_num == 100)
    {
      integral_l_num = 0;
      integral_r_num = 0;
      
      delta_speed_l = 0;
      delta_speed_r = 0;
      
      true_left_distance = 0;
      true_right_distance = 0;

      last_left_distance = 0;
      last_right_distance = 0;
      
      return 1;
    }
    else
    {
      return 0;
    }
}
//计算点到点之间的距离、点到原点之间的世界坐标系下的角度
void how_to_go_to_point(float *point, float *distance, float *azimuth, uint8_t point_num)
{
  for(uint8_t i = 1; i < point_num; i++)
  {
    distance[i] = get_two_points_distance(point[(i - 1) * 10 + 0], point[(i - 1) * 10 + 1], point[i * 10 + 0], point[i * 10 + 1]);
    //以正北方向为起点，顺时针选择的范围是0~360度的角度
    azimuth[i] = -get_two_points_azimuth(point[(i - 1) * 10 + 0], point[(i - 1) * 10 + 1], point[i * 10 + 0], point[i * 10 + 1]);
  }
}
//将上个点得到的点到原点的角度转化成实际的yaw角
void to_get_useful_angle(float *azimuth, float *yaw, float offset, uint8_t point_num)
{
  for(uint8_t i = 1; i < point_num; i++)
  {
    if(azimuth[i] < -180.0f)
      yaw[i] = azimuth[i] + 360.0f + offset;
    else
      yaw[i] = azimuth[i] + offset;
  }
}