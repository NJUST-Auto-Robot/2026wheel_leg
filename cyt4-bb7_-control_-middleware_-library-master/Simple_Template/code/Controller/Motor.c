#include "Motor.h"

MotorPID_t wheel_speed_pid;

static float MotorPID_Update(MotorPID_t *pid, float setpoint, float measurement, float dt) {
  float error = setpoint - measurement;
  float output = 0.0f;

  if (fabsf(error) > pid->DeadZone) {
    float p_term = pid->Kp * error;
    pid->I_term += pid->Ki * error * dt;
    pid->I_term = fmaxf(fminf(pid->I_term, pid->I_term_Max), -pid->I_term_Max);
    float d_term = pid->Kd * (error - pid->prev_error) / dt;

    output = p_term + pid->I_term + d_term;
    output = fmaxf(fminf(output, pid->Out_Max), -pid->Out_Max);
  }

  pid->prev_error = error;
  return output;
}

void MotorPID_Init(float kp, float ki, float kd, float i_term_max, float out_max) {
  wheel_speed_pid.Kp = kp;
  wheel_speed_pid.Ki = ki;
  wheel_speed_pid.Kd = kd;
  wheel_speed_pid.I_term = 0.0f;
  wheel_speed_pid.prev_error = 0.0f;
  wheel_speed_pid.I_term_Max = i_term_max;
  wheel_speed_pid.Out_Max = out_max;
  wheel_speed_pid.DeadZone = 0.0f;
}

// target_linear_speed 单位 m/s
// measured_wheel_speed 单位 rad/s
float MotorSpeedClosedLoopTorque(float target_linear_speed, float measured_wheel_speed) {
  float measured_linear_speed = measured_wheel_speed * WHEEL_RADIUS;
  float dt = 0.01f; // 10 ms 期望 ControlTask 调用周期

  float torque_out = MotorPID_Update(&wheel_speed_pid, target_linear_speed, measured_linear_speed, dt);
  return torque_out;
}

float MotorComputeTotalTorque(float target_linear_speed, float measured_wheel_speed, float u_balance) {
  float u_speed = MotorSpeedClosedLoopTorque(target_linear_speed, measured_wheel_speed);
  float u_total = u_balance + u_speed;

  // 如需要可在此做输出限幅
  // float u_max = 100.0f;
  // u_total = fmaxf(fminf(u_total, u_max), -u_max);

  return u_total;
}

