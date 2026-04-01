#ifndef UBX_DECODER_H_
#define UBX_DECODER_H_
#include "FreeRTOS.h"
#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "task.h"
#include "zf_driver_timer.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct _UBX_NAV_PVT_ValidFlagsStructdef {
  uint8_t reserved : 4;
  bool validMag : 1;
  bool timeOfDayFullyResolved : 1;
  bool validTimeOfDay : 1;
  bool validDate : 1;
} UBX_NAV_PVT_ValidFlagsStructdef;
typedef struct _UBX_NAV_PVT_FixStatusFlagsStructdef {
  uint8_t carrierPhaseState : 2;
  bool headOfVehicleValid : 1; // 模块不带IMU融合，没用
  uint8_t powerSafeMode : 3;
  bool differentialCorrectionOk : 1; // 没RTK，没用
  bool gnssFixOK : 1;
} UBX_NAV_PVT_FixStatusFlagsStructdef;

typedef struct _UBX_NAV_PVT_MessageStructdef {
  uint8_t header1;   // 0xb5
  uint8_t header2;   // 0x62
  uint8_t class_num; // 0x01
  uint8_t cmd_id;    // 0x07
  uint16_t length;   // 92
  // Payload
  uint32_t iTOW; //  GPS time of week of the navigation epoch
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour; // UTC时间
  uint8_t min;
  uint8_t second;
  UBX_NAV_PVT_ValidFlagsStructdef time_valid_flags;
  uint32_t timeAccuracy; // Time accuracy estimate (UTC)
  int32_t nano;          // Fraction of second, range -1e9 .. 1e9 (UTC)
  //     GNSS定位状态:
  //  0: no fix
  //  1: dead reckoning only
  //  2: 2D-fix
  //  3: 3D-fix
  //  4: GNSS + dead reckoning combined
  //  5: time only fix
  uint8_t fix_type;
  UBX_NAV_PVT_FixStatusFlagsStructdef fix_status_flags; // 没啥大用
  uint8_t additional_flags;
  uint8_t numOfSatellites;     // Number of satellites used in Nav Solution
  int32_t longitude;           // 乘1e-7
  int32_t latitude;            // 乘1e-7
  int32_t height;              // Height above ellipsoid 单位毫米
  int32_t hMSL;                // Height above mean sea level 单位毫米
  uint32_t horizontalAccuracy; // Horizontal accuracy estimate 单位毫米
  uint32_t verticalAccuracy;   // Vertical accuracy estimate 单位毫米
  int32_t velN;                // NED坐标系速度
  int32_t velE;                // NED坐标系速度
  int32_t velD;                // NED坐标系速度
  int32_t ground_speed;        // 地速 mm/s
  int32_t head_of_motion;      // 航向角乘1e-5
  uint32_t speedAccuracy;      // Speed accuracy estimate
  uint32_t headingAccuracy;    // Heading accuracy estimate 乘1e-5
  uint16_t pDOP;               // 乘0.01
  uint16_t flags3;
  uint8_t reserved[4];
  int32_t headVeh; // 没用
  int16_t magDec;
  int16_t magAcc;
  uint8_t cheksumA;
  uint8_t cheksumB;
} __attribute__((packed)) UBX_NAV_PVT_MessageStructdef;
typedef struct _UBXPositionHeadingDataStructdef {
  double latitude;  // 纬
  double longitude; // 经
  double headingOfVehicle;
  float positionDOP;
  float groundSpeed;   // 地面速度 m/s
  float positionError; // 定位误差，由DOP和精度估计计算得到,+-()米
} UBXPositionDataStructdef;

uint32_t decodeUbxPVT(uint8_t *addr, uint32_t len);
extern UBX_NAV_PVT_MessageStructdef UBXMsg;
extern UBXPositionDataStructdef UBXPositionData;
extern uint32_t UBXDecodedPacketCnt;
extern TickType_t lastUBXReceivedTime; // 最后一次收到有效数据包的时间
#ifdef __cplusplus
}
#endif
#endif
