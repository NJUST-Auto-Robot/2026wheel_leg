#include "Simple_PID/PID.h"

PID center_PID_1;
PID center_PID_2;
PID center_PID_3;
PID center_PID_4;

PID Angle_PID;
PID Left_Distance_PID;
PID Right_Distance_PID;

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
        if(pid->output >= MAX_OUT)
                pid->output = MAX_OUT;
        else if(pid->output <= -MAX_OUT)
                pid->output = -MAX_OUT;
        pid->output = pid->out / MAX_OUT * MAX_OUTPUT;
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

void Positional_PID_Calculate_Angles_And_Distance(PID * pid, uint8_t PID_Mode)
{
	//误差传递与计算
	pid->err_last = pid->err_now;
	pid->err_now = pid->target - pid->actual;
        if(PID_Mode == DISTANCE_PID)
        {
          if(pid->err_now >= 180.f)
            pid->err_now -= 360.0f;
          else if(pid->err_now <= -180)
            pid->err_now += 360.0f;
        }
	//p、i、d三项赋值
	pid->err_p  = pid->err_now;
	pid->err_i += pid->err_now;
	pid->err_d  = pid->err_now - pid->err_last;
	//积分项限幅
        if(PID_Mode == ANGLE_PID)
        {
          if(pid->err_i >= ANGLE_PID_MAX_INTEGRAL_ERR)
                  pid->err_i = ANGLE_PID_MAX_INTEGRAL_ERR;
          else if(pid->err_i <= -ANGLE_PID_MAX_INTEGRAL_ERR)
                  pid->err_i = -ANGLE_PID_MAX_INTEGRAL_ERR;     
        }
        else if(PID_Mode == DISTANCE_PID)
        {
          if(pid->err_i >= DISTANCE_PID_MAX_INTEGRAL_ERR)
                  pid->err_i = DISTANCE_PID_MAX_INTEGRAL_ERR;
          else if(pid->err_i <= -DISTANCE_PID_MAX_INTEGRAL_ERR)
                  pid->err_i = -DISTANCE_PID_MAX_INTEGRAL_ERR;     
        }
	//位置式PID计算公式
	pid->out = (pid->kp * pid->err_p) + (pid->ki * pid->err_i) + (pid->kd * pid->err_d);
	//输出限制
        if(PID_Mode == ANGLE_PID)
        {
          if(pid->output >= ANGLE_PID_MAX_OUT)
                  pid->output = ANGLE_PID_MAX_OUT;
          else if(pid->output <= -ANGLE_PID_MAX_OUT)
                  pid->output = -ANGLE_PID_MAX_OUT;
          pid->output = pid->out / ANGLE_PID_MAX_OUT * ANGLE_PID_MAX_OUTPUT;
       }
        else if(PID_Mode == DISTANCE_PID)
        {
          if(pid->output >= DISTANCE_PID_MAX_OUT)
                  pid->output = DISTANCE_PID_MAX_OUT;
          else if(pid->output <= -DISTANCE_PID_MAX_OUT)
                  pid->output = -DISTANCE_PID_MAX_OUT;
          pid->output = pid->out / DISTANCE_PID_MAX_OUT * DISTANCE_PID_MAX_OUTPUT;
        }
        

        if(PID_Mode == ANGLE_PID)
        {
          if(pid->output >= ANGLE_PID_MAX_OUTPUT)
                  pid->output = ANGLE_PID_MAX_OUTPUT;
          else if(pid->output <= -ANGLE_PID_MAX_OUTPUT)
                  pid->output = -ANGLE_PID_MAX_OUTPUT;
        }
        else if(PID_Mode == DISTANCE_PID)
        {
          if(pid->output >= DISTANCE_PID_MAX_OUTPUT)
                  pid->output = DISTANCE_PID_MAX_OUTPUT;
          else if(pid->output <= -DISTANCE_PID_MAX_OUTPUT)
                  pid->output = -DISTANCE_PID_MAX_OUTPUT;
        }
        	
}

float PID_Control_Angles_And_Distance(PID * pid, float target, float actual, uint8_t PID_Mode)
{
	pid->target = target;
	pid->actual = actual;
	
        Positional_PID_Calculate_Angles_And_Distance(pid, PID_Mode);

        return pid->output;
}
