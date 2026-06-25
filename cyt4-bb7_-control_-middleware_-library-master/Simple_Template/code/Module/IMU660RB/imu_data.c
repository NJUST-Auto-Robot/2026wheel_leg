#include "imu_data.h"
#include "zf_common_headfile.h"
#include "Algorithm/Fusion/Fusion.h"

#define OFFSET_CAL_TIME   (50)
imu_state_t imu660rb; // 全局融合状态实例
FusionAhrs ahrs;
FusionEuler euler;
static FusionOffset offset;
static FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
static FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
static FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};

void imu_init(imu_state_t *state)
{   
    uint8_t offset_cnt;
    state->dt = 1.0f / IMU_FUSION_SAMPLE_RATE;
    state->imu_data_ready = false;  // 数据就绪标志初始为false
    state->imu_data_true  = false;  // 设置数据可信度为false，等待陀螺仪稳定
    for(int i=0; i<3; i++) {
        state->raw_data.acc[i] = 0.0f;  // 初始化加速度计数据
        state->raw_data.gyro[i] = 0.0f; // 初始化陀螺仪数据
    }
    state->angles.last_roll = 0.0f;     // 初始化上次横滚角
    state->angles.last_pitch = 0.0f;    // 初始化上次俯仰角
    state->angles.last_yaw = 0.0f;      // 初始化上次偏航角
    state->angles.roll = 0.0f;          // 初始化横滚角
    state->angles.pitch = 0.0f;         // 初始化俯仰角
    state->angles.yaw = 0.0f;           // 初始化偏航角
    state->angles.roll_acc = 0.0f;      // 初始化横滚角
    state->angles.pitch_acc = 0.0f;// 初始化俯仰角
    state->angles.yaw_acc = 0.0f;  // 初始化偏航角

    pit_us_init(PIT_CH1, 19200); //设置定时器中断为19.2ms，对应52Hz的采样率
    imu660rb_init();   // 初始化IMU660RB

    FusionAhrsInitialise(&ahrs);
    FusionOffsetInitialise(&offset, 52); // 以52Hz的采样率初始化FusionOffset
    offset_cnt = OFFSET_CAL_TIME;
    
    while(offset_cnt)
    {   
        if(imu660rb.imu_data_ready) {
            imu660rb_get_gyro();               // 读取IMU数据
            imu660rb.imu_data_ready = false;
        

        state->raw_data.gyro[x] = imu660rb_gyro_transition(imu660rb_gyro_x); // 将原始陀螺仪数据转换为物理单位
        state->raw_data.gyro[y] = imu660rb_gyro_transition(imu660rb_gyro_y); 
        state->raw_data.gyro[z] = imu660rb_gyro_transition(imu660rb_gyro_z); 

        gyroscopeOffset.array[0] += state->raw_data.gyro[x];
        gyroscopeOffset.array[1] += state->raw_data.gyro[y];
        gyroscopeOffset.array[2] += state->raw_data.gyro[z];
        offset_cnt--;
        }
    }
    gyroscopeOffset.array[0] /= OFFSET_CAL_TIME;
    gyroscopeOffset.array[1] /= OFFSET_CAL_TIME;
    gyroscopeOffset.array[2] /= OFFSET_CAL_TIME;

}

void imu_read_data(imu_state_t *state)
{   
    if(state == NULL) {
        return; // 防止空指针访问
    }

        imu660rb_get_acc();               // 获取 imu660rb 的加速度测量数值
        imu660rb_get_gyro();              // 获取 imu660rb 的角速度测量数值

        state->raw_data.acc[x]  =  imu660rb_acc_transition(imu660rb_acc_x); // 将原始加速度计数据转换为物理单位
        state->raw_data.acc[y]  =  imu660rb_acc_transition(imu660rb_acc_y); // 将原始加速度计数据转换为物理单位
        state->raw_data.acc[z]  =  imu660rb_acc_transition(imu660rb_acc_z); // 将原始加速度计数据转换为物理单位

        state->raw_data.gyro[x] = imu660rb_gyro_transition(imu660rb_gyro_x); // 将原始陀螺仪数据转换为物理单位
        state->raw_data.gyro[y] = imu660rb_gyro_transition(imu660rb_gyro_y); // 将原始陀螺仪数据转换为物理单位
        state->raw_data.gyro[z] = imu660rb_gyro_transition(imu660rb_gyro_z); // 将原始陀螺仪数据转换为物理单位

        FusionVector accelerometer = {state->raw_data.acc[x], state->raw_data.acc[y], state->raw_data.acc[z]};
        FusionVector gyroscope = {state->raw_data.gyro[x], state->raw_data.gyro[y], state->raw_data.gyro[z]};
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 1.0/52.0f);
        euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        
        state->angles.roll = euler.angle.roll;
        state->angles.pitch = euler.angle.pitch;
        state->angles.yaw = euler.angle.yaw;

        if(state->imu_data_true < 0)
        {
            state->angles.roll_acc = (state->angles.roll - state->angles.last_roll) / state->dt;
            state->angles.pitch_acc = (state->angles.pitch - state->angles.last_pitch) / state->dt;
            state->angles.yaw_acc = (state->angles.yaw - state->angles.last_yaw) / state->dt;
            state->angles.last_roll = state->angles.roll;
            state->angles.last_pitch = state->angles.pitch;
            state->angles.last_yaw = state->angles.yaw;
        }
        
}

void imu_data_check(imu_state_t *state)
{   
        state->imu_data_true = -1; // 设置为-1表示数据已经稳定并且可信
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
        state->angles.roll  =  state->angles.roll  - roll_ref;   // 坐标系转换后的横滚角
        state->angles.pitch =  state->angles.pitch - pitch_ref;  // 坐标系转换后的俯仰角
        state->angles.yaw   =  state->angles.yaw   - yaw_ref;    // 坐标系转换后的偏航角
    }
}

void imu_tx_data(imu_state_t *state)
{
    // printf("\r\nimu660rb acc data:  x=%d, y=%d, z=%d\r\n", 
             //(int)(state->angles.roll_acc*100),  (int)(state->angles.pitch_acc*100),  (int)(state->angles.yaw_acc*100));
    // printf("\r\nimu660rb gyro data: x=%f, y=%f, z=%f\r\n", 
    //         state->raw_data.gyro[x], state->raw_data.gyro[y], state->raw_data.gyro[z]);
    //printf("\r\nimu660rb angle data:  roll=%d°, pitch=%d°, yaw=%d°\r\n", 
            //(int)state->angles.roll, (int)state->angles.pitch, (int)state->angles.yaw);
}