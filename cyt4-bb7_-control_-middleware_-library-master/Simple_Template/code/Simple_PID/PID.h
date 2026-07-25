#ifndef __PID_H_
#define __PID_H_


#include "zf_common_typedef.h"

#define MAX_INTEGRAL_ERR 60.0f
#define MAX_OUT          30.0f
#define MAX_OUTPUT       0.2f

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

void PID_Init(PID*pid);							    						//参数初始化
void PID_Set(PID *pid, float p, float i, float d);			//设置PID参数
void Positional_PID_Calculate(PID * pid);
float PID_Control(PID * pid, float target, float actual);

#endif