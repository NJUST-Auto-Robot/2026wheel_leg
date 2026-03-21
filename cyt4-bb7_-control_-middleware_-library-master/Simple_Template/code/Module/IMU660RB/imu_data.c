#include "imu_data.h"
#include "zf_common_headfile.h"
#include "algorithm/mahony/mahony.h"

imu_state_t imu660rb; // 全局融合状态实例

void imu_init(imu_state_t *state)
{   
    imu660rb_init();   // 初始化IMU660RB
    imu_mahony_init(); // 初始化Mahony滤波算法
    pit_us_init(PIT_CH1, 10000); //设置定时器中断为10ms，对应100Hz的采样率
    state->dt = 1.0f / IMU_FUSION_SAMPLE_RATE;
    state->imu_data_ready = false;  // 数据就绪标志初始为false
    state->imu_data_true  = false;  // 设置数据可信度为false，等待陀螺仪稳定
    for(int i=0; i<3; i++) {
        state->raw_data.acc[i] = 0.0f;  // 初始化加速度计数据
        state->raw_data.gyro[i] = 0.0f; // 初始化陀螺仪数据
    }
    state->yaw_offset_sum = 0.0f;       // 初始化偏航角数值之和
    state->yaw_offset_average = 0.0f;   // 初始化偏航角补偿值平均值
    state->angles.last_roll = 0.0f;     // 初始化上次横滚角
    state->angles.last_pitch = 0.0f;    // 初始化上次俯仰角
    state->angles.last_yaw = 0.0f;      // 初始化上次偏航角
    state->angles.roll = 0.0f;          // 初始化横滚角
    state->angles.pitch = 0.0f;         // 初始化俯仰角
    state->angles.yaw = 0.0f;           // 初始化偏航角
    state->angles.roll_in_cordinate = 0.0f;  // 初始化坐标系转换后的横滚角
    state->angles.pitch_in_cordinate = 0.0f; // 初始化坐标系转换后的俯仰角
    state->angles.yaw_in_cordinate = 0.0f;   // 初始化坐标系转换后的偏航角
}

void imu_read_data(imu_state_t *state)
{   
    if(state == NULL) {
        return; // 防止空指针访问
    }
    if(state->imu_data_ready==true) {
        return; // 如果数据已经就绪，避免重复读取
    }
    else if(!state->imu_data_ready) {
        imu660rb_get_acc();               // 获取 imu660rb 的加速度测量数值
        imu660rb_get_gyro();              // 获取 imu660rb 的角速度测量数值

        state->raw_data.acc[x]  =  imu660rb_acc_transition(imu660rb_acc_x); // 将原始加速度计数据转换为物理单位
        state->raw_data.acc[y]  =  imu660rb_acc_transition(imu660rb_acc_y); // 将原始加速度计数据转换为物理单位
        state->raw_data.acc[z]  =  imu660rb_acc_transition(imu660rb_acc_z); // 将原始加速度计数据转换为物理单位

        state->raw_data.gyro[x] = imu660rb_gyro_transition(imu660rb_gyro_x); // 将原始陀螺仪数据转换为物理单位
        state->raw_data.gyro[y] = imu660rb_gyro_transition(imu660rb_gyro_y); // 将原始陀螺仪数据转换为物理单位
        state->raw_data.gyro[z] = imu660rb_gyro_transition(imu660rb_gyro_z); // 将原始陀螺仪数据转换为物理单位
        
        state->imu_data_ready = true; // 设置数据就绪标志
    }
}

void imu_data_check(imu_state_t *state)
{   
    if(    fabs(state->angles.last_pitch - state->angles.pitch)  < 0.5f
        && fabs(state->angles.last_roll  - state->angles.roll)   < 0.5f
        && fabs(state->angles.last_yaw   - state->angles.yaw)    < 0.5f) {
        // 如果当前欧拉角与上次欧拉角的差值都小于0.5度，认为数据稳定
        state->imu_data_true ++; // 数据稳定，增加数据可信度计数
    } else {
        // 否则，更新上次欧拉角为当前欧拉角，继续等待数据稳定
        state->angles.last_pitch = state->angles.pitch;
        state->angles.last_roll  = state->angles.roll;
        state->angles.last_yaw   = state->angles.yaw;
        state->imu_data_true = 0; // 数据不稳定，重置数据可信度
    }
    if(state->imu_data_true > 50) {
        state->imu_data_true = -1; // 数据稳定且可信，可以开始使用数据，设置为-1表示数据已经稳定并且可信
    }
}

void imu_tx_data(imu_state_t *state)
{
    printf("\r\nimu660rb angle data:  roll=%d°, pitch=%d°, yaw=%d°\r\n", 
            (int)state->angles.roll_in_cordinate, (int)state->angles.pitch_in_cordinate, (int)state->angles.yaw_in_cordinate);
}

void imu_zero_calibration(imu_state_t *state)
{
    // 这里可以实现陀螺仪零偏校准的逻辑，例如在静止状态下采集多次陀螺仪数据，计算平均值作为零偏补偿
    static int yaw_offset_times = 0; // 用于记录偏航角补偿的采集次数
    if(yaw_offset_times < 500) {
        // 在陀螺仪稳定后，采集500次偏航角数据进行补偿
        state->yaw_offset_sum += state->angles.yaw - state->angles.last_yaw; // 计算当前偏航角与上次偏航角的差值
        yaw_offset_times++;
    }
    else if(yaw_offset_times > 500)
    {
        state->yaw_offset_average = state->yaw_offset_sum / 500.0f; // 计算平均偏航角补偿值 
        yaw_offset_times = -1; // 重置补偿次数
    }
    else if(yaw_offset_times == -1) {
        // 应用偏航角补偿值进行校准
        state->angles.yaw -= state->yaw_offset_average; // 从当前偏航角中减去平均补偿值，得到校准后的偏航角
    }
}

void imu_cordinate_convert(imu_state_t *state)
{   
    static bool  convert_flag = false;      // 坐标系转换标志
    static float roll_ref  = 0.0f;          // 横滚角参考值
    static float pitch_ref = 0.0f;          // 俯仰角参考值
    static float yaw_ref   = 0.0f;          // 偏航角参考值
    if(convert_flag == false ) {
        roll_ref  = state->angles.roll;
        pitch_ref = state->angles.pitch;
        yaw_ref   = state->angles.yaw;
        convert_flag = true; // 设置坐标系转换标志，后续不再更新参考值
    }
    if(convert_flag == true) {
        state->angles.roll_in_cordinate  =  state->angles.roll  - roll_ref;   // 坐标系转换后的横滚角
        state->angles.pitch_in_cordinate =  state->angles.pitch - pitch_ref;  // 坐标系转换后的俯仰角
        state->angles.yaw_in_cordinate   =  state->angles.yaw   - yaw_ref;    // 坐标系转换后的偏航角
    }
}

void imu_acceleration_compute(imu_state_t *state)
{
    // 计算欧拉角加速度
    state->angles.roll_acceleration = (state->angles.roll - state->angles.last_roll) / state->dt;   // 横滚角加速度
    state->angles.pitch_acceleration = (state->angles.pitch - state->angles.last_pitch) / state->dt; // 俯仰角加速度
    state->angles.yaw_acceleration = (state->angles.yaw - state->angles.last_yaw) / state->dt;     // 偏航角加速度
}