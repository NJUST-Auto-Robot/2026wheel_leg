#ifndef __BASE_CONTROL_H_
#define __BASE_CONTROL_H_


#include "zf_common_typedef.h"
#include "zf_common_headfile.h"
#include "Simple_PID/PID.h"

#define M7_1_TO_M7_0_DATA_LENGTH               (5)                                           // 数组数据长度
#define M7_0_TO_M7_1_DATA_LENGTH               (5)                                           // 数组数据长度

extern float m7_1_to_m7_0_data[M7_1_TO_M7_0_DATA_LENGTH];//1核数据传给0核
extern float m7_0_to_m7_1_data[M7_1_TO_M7_0_DATA_LENGTH];//0核数据传给1核
 
 //角度pid控制变量
extern float is_yaw_ok ;//检验0核是否又传来yaw角数据再读取传来的yaw角数据，1表示有数据传来
extern float target_yaw ;//目标yaw值，用于给pid调控赋目标值，在函数内有使用。也可在函数外合理使用（如获取一次当前yaw角并赋值给target_yaw，传target_yaw+n作为角度pid调控的目标参数）
extern float actual_yaw ;//实际yaw值，接收到0核传来的数据后将数据转存给此变量
extern float true_yaw;//真实yaw值，以当前yaw值为0度位置进行“旋转目标角度”的角度pid调控
extern float have_yaw;//在函数内获取一次当前yaw值，以当前yaw值为0度位置进行pid调控，不在函数内清零，需要自己额外清零
extern uint8_t yaw_is_stable;//实际yaw值核目标yaw值是否相差不大
 
extern float yaw_now;//现在的yaw值，未使用，可用于自行编写获取一次当前yaw值的逻辑然后进行以实际想要“转到的yaw值”作为目标yaw值的角度调控
 
extern float a_speed_l;//角度调控左轮速度
extern float a_speed_r;//角度调控右轮速度
 
 //距离pid控制变量
extern float is_distance_ok;//检验0核是否又传来距离数据再读取传来的距离数据，1表示有数据传来
 
extern uint8_t distance_control_flag;//距离调控阶段标志位，是本人在调试检验所写函数是否有用使用的标志位
 
extern float target_left_distacne;//目标左轮距离，m为单位
extern float actual_left_distance;//实际左轮距离，m为单位
extern float last_left_distance;//上一次实际左轮距离
extern float true_left_distance;//实际使用的左轮距离，因为本人距离调控是先把当前距离设置成为0m在进行距离调控
 
extern float target_right_distacne;//目标左轮距离，m为单位
extern float actual_right_distance;//实际左轮距离，m为单位
extern float last_right_distance;//上一次实际左轮距离
extern float true_right_distance;//实际使用的左轮距离，因为本人距离调控是先把当前距离设置成为0m在进行距离调控
 
extern uint8_t left_distance_is_stable;//左轮行驶距离与目标距离是否相差不大
extern uint8_t right_distance_is_stable;//右轮行驶距离与目标距离是否相差不大
 
extern float d_speed_l;//距离调控左轮速度
extern float d_speed_r;//距离调控右轮速度
 
extern float delta_speed_l;//左轮为了到达最终速度进行线性累加的最小累加速度
extern float delta_speed_r;//右轮为了到达最终速度进行线性累加的最小累加速度
 
extern uint8_t integral_l_num;//左轮为了到达最终速度进行线性累加积分次数
extern uint8_t integral_r_num;//右轮为了到达最终速度进行线性累加积分次数
 
extern uint8_t turn_to_true_yaw;
 /***********10个点，点与点距离、点与原点角度只有9个值，所以distance[0]、azimuth[0]、yaw[0]无意义*********/
extern float stage_1_point[10][2]; //结合GPS使用的路径规划所需要的数组，[0][0]、[0][1]为起始出发点的纬度、经度

extern float stage_1_distance[10]; //点到点之间的距离

extern float stage_1_azimuth[10]; //点到原点的世界坐标系的角度

extern float stage_2_point[10][2]; //结合GPS使用的路径规划所需要的数组，[0][0]、[0][1]为起始出发点的纬度、经度
extern float stage_2_distance[10]; //点到点之间的距离

extern float stage_2_azimuth[10]; //点到原点的世界坐标系的角度

extern float yaw_offset; //车身出发时与正北方向的偏置——正北方向yaw-出发车身方向yaw

extern float stage_1_yaw[10]; //实际使用的yaw角

extern float stage_2_yaw[10]; //实际使用的yaw角

//先把当前yaw值设置为0度，之后进行转过多少度的角度调控，此函数是方便调试检验而写
uint8_t Angle_Control(float target, float base_speed_l, float base_speed_r);
//转到目标yaw角的角度调控
uint8_t Angle_Control_2(float target, float base_speed_l, float base_speed_r);
//距离调控，无pid，有两个结束标志——是否行驶至目标距离附近以及行驶速度是否线性变化成终点速度
uint8_t Distance_Control(float left_target, float right_target, float base_speed_l, float base_speed_r, float end_speed_l, float end_speed_r);
//Angle_Control+Distance_Control的结合
uint8_t Angle_And_Distance_Control(float target, float base_speed_l, float base_speed_r, float left_target, float right_target, float end_speed_l, float end_speed_r);
//Angle_Control_2+Distance_Control的结合
uint8_t Angle_And_Distance_Control_2(float target, float base_speed_l, float base_speed_r, float left_target, float right_target, float end_speed_l, float end_speed_r);
//传入点的纬度、经度坐标和点的数量后计算顺序两点之间的距离、世界坐标系的角度
void how_to_go_to_point(float *point, float *distance, float *azimuth, uint8_t point_num);
//将世界坐标系下的顺序两点之间的角度转化成实际使用的点到点之间所需要行驶的yaw角
void to_get_useful_angle(float *azimuth, float *yaw, float offset, uint8_t point_num);


#endif