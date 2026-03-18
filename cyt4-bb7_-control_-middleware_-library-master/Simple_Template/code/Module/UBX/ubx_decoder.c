/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-06
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-06
 * @FilePath: ubx_decoder.c
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#include "ubx_decoder.h"
UBX_NAV_PVT_MessageStructdef UBXMsg;
UBXPositionDataStructdef UBXPositionData;
uint32_t UBXDecodedPacketCnt;
uint32_t lastUBXReceivedTime; // 最后一次收到有效数据包的时间
uint32_t decodeUbxPVT(uint8_t *addr, uint32_t len) {
  UBX_NAV_PVT_MessageStructdef *ptr;
  uint8_t checksum_a = 0;
  uint8_t checksum_b = 0;
  uint8_t i = 0;
  bool valid_flag = false;
  if (len != sizeof(UBX_NAV_PVT_MessageStructdef))
    return 1;
  ptr = (UBX_NAV_PVT_MessageStructdef *)(addr);
  if (ptr->header1 != 0xb5 || ptr->header2 != 0x62)
    return 1;
  if (ptr->class_num != 0x01 || ptr->cmd_id != 0x07)
    return 1;
  if (ptr->length != 92)
    return 1;
  for (i = 2; i <= 97; i++) {
    checksum_a += addr[i];
    checksum_b += checksum_a;
  }
  if (checksum_a ^ ptr->cheksumA || checksum_b ^ ptr->cheksumB)
    return 1;
  valid_flag = true;
  if (valid_flag == true) {
    memcpy(&UBXMsg, ptr, sizeof(UBXMsg));
    UBXPositionData.latitude = UBXMsg.latitude * 1e-7;
    UBXPositionData.longitude = UBXMsg.longitude * 1e-7;
    UBXPositionData.headingOfVehicle = UBXMsg.head_of_motion * 1e-5;
    UBXPositionData.positionDOP = UBXMsg.pDOP * 0.01;
    UBXPositionData.groundSpeed = UBXMsg.ground_speed * 1e-3;
    UBXPositionData.positionError =
        UBXPositionData.positionDOP * UBXMsg.horizontalAccuracy * 1e-3;
    UBXDecodedPacketCnt++;
    lastUBXReceivedTime = GetSystemTimer();
    return 0;
  } else
    return 1;
}
