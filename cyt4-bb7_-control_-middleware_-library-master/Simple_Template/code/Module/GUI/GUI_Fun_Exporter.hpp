/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-07
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: GUI_Fun_Exporter.hpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#ifndef GUI_FUN_EXPORTER_HPP
#define GUI_FUN_EXPORTER_HPP
#ifdef __cplusplus
extern "C" {
#endif
void SpdClosedOffCB();
void SpdClosedOnCB();
void ReverseOffCB();
void ReverseOnCB();
void OutdoorModeCB();
void IndoorModeCB();

void GNSSDispCB();
void WirelessDispCB();
void CarStatCB();

void CollectPointsCB();
void ClearPointsCB();

void GNSSPCB();
void GNSSICB();
void GNSSDCB();
void GNSSLCB();
void GNSSSCB();
void GNSSDISCB();

void SoundPCB();
void SoundICB();
void SoundDCB();
void SoundLCB();

void SoundSCB();
void SoundRCB();
void SoundDISCB();
void ClosedSpdPCB();
void ClosedSpdICB();
void ClosedSpdDCB();
void ClosedSpdLCB();
void ClosedSpdTCB();
#ifdef __cplusplus
}
#endif
#endif // GUI_FUN_EXPORTER_HPP