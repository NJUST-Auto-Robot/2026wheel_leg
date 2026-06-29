#ifndef _IMU_DATA_H_
#define _IMU_DATA_H_ 

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

// �궨��
#define IMU_FUSION_SAMPLE_RATE 208.0f           // ����Ƶ�� (Hz)
#define IMU_FUSION_COMPLEMENTARY_ALPHA 0.98f   // �����˲�alphaֵ
#define IMU_FUSION_GYRO_SCALE 500.0f           // ���������� (��/s)
#define IMU_FUSION_ACC_SCALE 2.0f              // ���ٶȼ����� (g)

typedef enum {
    x = 0,
    y = 1,
    z = 2
} Axis_t;

// �ṹ�嶨��
typedef struct {
    float acc[3];   // ���ٶȼ����� (g) x,y,z
    float gyro[3];  // ���������� (��/s) x,y,z
} imu_data_t;

typedef struct {
    float roll;   // ����� (��)
    float pitch;  // ������ (��)
    float yaw;    // ƫ���� (��)
    float roll_acc;  
    float pitch_acc;  
    float yaw_acc;   
    float last_roll;  
    float last_pitch;  
    float last_yaw;
} euler_angles_t;

typedef struct {
    imu_data_t raw_data;        // ��ǰIMU����
    euler_angles_t angles;      // ��ǰŷ����
    float dt;                   // ʱ����
    bool  imu_data_ready;       // ���ݾ�����־
    int   imu_data_true;        // ��ʼ��״̬
} imu_state_t;

extern imu_state_t imu660rb; // ȫ���ں�״̬ʵ��

// ��������
void imu_init(imu_state_t *state);
void imu_read_data(imu_state_t *state);
void imu_tx_data(imu_state_t *state);
void imu_data_check(imu_state_t *state);
void imu_cordinate_convert(imu_state_t *state);

#ifdef __cplusplus
}
#endif

#endif



