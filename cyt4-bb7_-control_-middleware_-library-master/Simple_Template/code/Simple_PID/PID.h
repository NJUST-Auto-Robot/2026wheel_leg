#ifndef __PID_H_
#define __PID_H_


#include "zf_common_typedef.h"

#define MAX_INTEGRAL_ERR 60.0f
#define MAX_OUT          30.0f
#define MAX_OUTPUT       0.2f

#define ANGLE_PID        1
#define DISTANCE_PID     2

#define ANGLE_PID_MAX_INTEGRAL_ERR      90.0f
#define ANGLE_PID_MAX_OUT               45.0f
#define ANGLE_PID_MAX_OUTPUT            0.2f

#define DISTANCE_PID_MAX_INTEGRAL_ERR   10.0f
#define DISTANCE_PID_MAX_OUT            5.0f
#define DISTANCE_PID_MAX_OUTPUT         0.4f

typedef struct
{
	float target;								//电机目标距离
	float actual;								//电机实际距离
	float err_p;
	float err_i;
	float err_d;
	float err_now;
	float err_last;
        float kp,ki,kd;       			
	float output;								
	float out;								
}PID;

extern PID center_PID_1;
extern PID center_PID_2;
extern PID center_PID_3;
extern PID center_PID_4;

extern PID Angle_PID;
extern PID Left_Distance_PID;
extern PID Right_Distance_PID;
 
void PID_Init(PID*pid);							    						//参数初始化
void PID_Set(PID *pid, float p, float i, float d);			//设置PID参数
void Positional_PID_Calculate(PID * pid);
float PID_Control(PID * pid, float target, float actual);
void Positional_PID_Calculate_Angles_And_Distance(PID * pid, uint8_t PID_Mode);
float PID_Control_Angles_And_Distance(PID * pid, float target, float actual, uint8_t PID_Mode);

#endif