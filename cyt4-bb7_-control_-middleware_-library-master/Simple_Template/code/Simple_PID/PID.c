#include "Simple_PID/PID.h"

PID center_PID_1;
PID center_PID_2;
PID center_PID_3;
PID center_PID_4;

void PID_Init(PID *pid)
{  	
	pid->target = 0;				
	pid->actual = 0;
	
	pid->kp = 0;						
	pid->ki = 0;						
	pid->kd = 0;
	
	pid->err_p = 0;
	pid->err_i = 0;
	pid->err_d = 0;
	pid->err_now = 0;
	pid->err_last = 0;
	
	
	pid->output = 0;
	pid->out = 0;
}	

void PID_Set(PID *pid, float p, float i, float d)			//设置PID参数和哪一种PID类型
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
}

//位置式PID计算
void Positional_PID_Calculate(PID * pid)
{
	//误差传递与计算
	pid->err_last = pid->err_now;
	pid->err_now = pid->target - pid->actual;
	//p、i、d三项赋值
	pid->err_p  = pid->err_now;
	pid->err_i += pid->err_now;
	pid->err_d  = pid->err_now - pid->err_last;
	//积分项限幅
        if(pid->err_i >= MAX_INTEGRAL_ERR)
                pid->err_i = MAX_INTEGRAL_ERR;
        else if(pid->err_i <= -MAX_INTEGRAL_ERR)
                pid->err_i = -MAX_INTEGRAL_ERR;
	//位置式PID计算公式
	pid->out = (pid->kp * pid->err_p) + (pid->ki * pid->err_i) + (pid->kd * pid->err_d);
	//输出限制
        pid->output = pid->out;
        if(pid->output >= MAX_OUTPUT)
                pid->output = MAX_OUTPUT;
        else if(pid->output <= -MAX_OUTPUT)
                pid->output = -MAX_OUTPUT;
	
}

float PID_Control(PID * pid, float target, float actual)
{
	pid->target = target;
	pid->actual = actual;
	
        Positional_PID_Calculate(pid);

        return pid->output;
}
