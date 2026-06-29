#ifndef IMU_MAHONY_H
#define IMU_MAHONY_H

#include "zf_common_typedef.h"
#include "Module/IMU660RB/imu_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define M_PI 3.14159265358979323846f

// º¯ÊýÉùÃ÷
void imu_mahony_init(void);
void imu_mahony_update(imu_data_t *data, float dt, euler_angles_t *euler);

#ifdef __cplusplus
}
#endif

#endif // IMU_MAHONY_H