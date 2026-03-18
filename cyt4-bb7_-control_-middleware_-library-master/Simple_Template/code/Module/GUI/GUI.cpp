/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-07
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: GUI.cpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#include "GUI.hpp"
#include "AAARobot/AAARobot.hpp"
#include "GUI_Fun_Exporter.hpp"
#include "Module/FS_I6X/FS_I6X.hpp"
#include "Module/RefereSystem/RefereSystem.hpp"
#include "Module/UBX/ubx_decoder.h"
#include "stdio.h"
const uint8_t app_period = 10;
void GNSS_App();
void Wireless_App();
void CarStat_App();
void CollectPoints_App();
void ClearPoints_App();
void GNSSP_App();
void GNSSI_App();
void GNSSD_App();
void GNSSL_App();
void GNSSS_App();
void GNSSDIS_App();
void SoundP_App();
void SoundI_App();
void SoundD_App();
void SoundL_App();
void SoundS_App();
void SoundR_App();
void SoundDIS_App();
void ClosedSpdP_App();
void ClosedSpdI_App();
void ClosedSpdD_App();
void ClosedSpdL_App();
void ClosedSpdT_App();
static auto While_USELESS_keep_f = [](auto func, uint16_t delay_time) {
  last_key_state = key_state;
  while (!(key_state != USELESS && last_key_state == USELESS)) {
    func();
    last_key_state = key_state;
    vTaskDelay(delay_time);
    Switch_Update();
  }
};

// 是MID的时候才会跳出循环
static auto While_MID_exit_f = [](auto func, uint16_t delay_time) {
  last_key_state = key_state;
  while (!(key_state == MID && last_key_state != key_state)) {
    func();
    last_key_state = key_state;
    vTaskDelay(delay_time);
    Switch_Update();
  }
};
/**
 * @brief 改参
 *
 * @param param 要改的参数引用
 * @param big_step 左右键大范围更改的步长
 * @param small_step 上下键小范围更改的步长
 * @return true 参数改变了
 * @return false 参数没变
 */
bool ChangeParam(float &param, float big_step, float small_step) {
  if (key_state == UP) {
    SwitchUse();
    param += small_step;
    return true;
  }
  if (key_state == DOWN) {
    SwitchUse();
    param -= small_step;
    return true;
  }
  if (key_state == LEFT) {
    SwitchUse();
    param -= big_step;
    return true;
  }
  if (key_state == RIGHT) {
    SwitchUse();
    param += big_step;
    return true;
  }
  return false;
}
void IndoorModeCB() {
  u8g2_ClearBuffer(&u8g2);
  u8g2_SendBuffer(&u8g2);
  car.com_env = indoor_com;
  vTaskDelay(100);
}
void OutdoorModeCB() {
  u8g2_ClearBuffer(&u8g2);
  u8g2_SendBuffer(&u8g2);
  car.com_env = outdoor_com;
  vTaskDelay(100);
}
void ReverseOnCB() {
  u8g2_ClearBuffer(&u8g2);
  u8g2_SendBuffer(&u8g2);
  car.is_reverse_enable = true;
  vTaskDelay(100);
}
void ReverseOffCB() {
  u8g2_ClearBuffer(&u8g2);
  u8g2_SendBuffer(&u8g2);
  car.is_reverse_enable = false;
  vTaskDelay(100);
}
void SpdClosedOnCB() {
  u8g2_ClearBuffer(&u8g2);
  u8g2_SendBuffer(&u8g2);
  car.is_spd_closed = true;
  vTaskDelay(100);
}
void SpdClosedOffCB() {
  u8g2_ClearBuffer(&u8g2);
  u8g2_SendBuffer(&u8g2);
  car.is_spd_closed = false;
  vTaskDelay(100);
}

void GNSSDispCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_USELESS_keep_f(GNSS_App, app_period);
}
void WirelessDispCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_USELESS_keep_f(Wireless_App, app_period);
}
void CarStatCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_USELESS_keep_f(CarStat_App, app_period);
}

void CollectPointsCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(CollectPoints_App, app_period);
}
void ClearPointsCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(ClearPoints_App, app_period);
}

void GNSSPCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(GNSSP_App, app_period);
}
void GNSSICB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(GNSSI_App, app_period);
}
void GNSSDCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(GNSSD_App, app_period);
}
void GNSSLCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(GNSSL_App, app_period);
}
void GNSSSCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(GNSSS_App, app_period);
}
void GNSSDISCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(GNSSDIS_App, app_period);
}

void SoundPCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(SoundP_App, app_period);
}
void SoundICB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(SoundI_App, app_period);
}
void SoundDCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(SoundD_App, app_period);
}
void SoundLCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(SoundL_App, app_period);
}
void SoundSCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(SoundS_App, app_period);
}
void SoundRCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(SoundR_App, app_period);
}
void SoundDISCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(SoundDIS_App, app_period);
}

void ClosedSpdPCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(ClosedSpdP_App, app_period);
}
void ClosedSpdICB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(ClosedSpdI_App, app_period);
}
void ClosedSpdDCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(ClosedSpdD_App, app_period);
}
void ClosedSpdLCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(ClosedSpdL_App, app_period);
}
void ClosedSpdTCB() {
  u8g2_ClearBuffer(&u8g2);
  DialogScale_Show(&u8g2, 1, 2, 120, 62);
  While_MID_exit_f(ClosedSpdT_App, app_period);
}

void GNSS_App() {
  u8g2_ClearBuffer(&u8g2);
  char buf[50];
  sprintf(buf, "status:%d", car.gnss_data.state);
  u8g2_DrawStr(&u8g2, 8, 10, buf);
  sprintf(buf, "lat:%.6f", UBXPositionData.latitude);
  u8g2_DrawStr(&u8g2, 8, 20, buf);
  sprintf(buf, "lon:%.6f", UBXPositionData.longitude);
  u8g2_DrawStr(&u8g2, 8, 30, buf);
  sprintf(buf, "numOfS:%d", UBXMsg.numOfSatellites);
  u8g2_DrawStr(&u8g2, 8, 40, buf);
  sprintf(buf, "spd: %.2f m/s", UBXPositionData.groundSpeed);
  u8g2_DrawStr(&u8g2, 8, 50, buf);
  sprintf(buf, "acc: %.2f m", UBXPositionData.positionError);
  u8g2_DrawStr(&u8g2, 8, 60, buf);
  u8g2_SendBuffer(&u8g2);
}
void Wireless_App() {
  u8g2_ClearBuffer(&u8g2);
  char buf[50];
  sprintf(buf, "FS:%d", remote.GetStatus());
  u8g2_DrawStr(&u8g2, 8, 15, buf);
  sprintf(buf, "Ref:%d", referee_inf.link_status);
  u8g2_DrawStr(&u8g2, 8, 25, buf);
  u8g2_SendBuffer(&u8g2);
}
void CarStat_App() {
  u8g2_ClearBuffer(&u8g2);
  char buf[50];
  sprintf(buf, "move:%d", car.move_mode);
  u8g2_DrawStr(&u8g2, 8, 15, buf);
  sprintf(buf, "indoor:%d", car.com_env);
  u8g2_DrawStr(&u8g2, 55, 15, buf);
  sprintf(buf, "goal:%d", car.where_goal_point);
  u8g2_DrawStr(&u8g2, 8, 25, buf);
  sprintf(buf, "reverse:%d", car.is_reverse_enable);
  u8g2_DrawStr(&u8g2, 55, 25, buf);
  if (car.navi_mode == gps_navi) {
    u8g2_DrawStr(&u8g2, 8, 35, "GPS");
  } else {
    u8g2_DrawStr(&u8g2, 8, 35, "Sound");
  }
  sprintf(buf, "dist:%.2f m", car.how_far_goal);
  u8g2_DrawStr(&u8g2, 8, 45, buf);
  sprintf(buf, "sound:%d", *car.pSound_settlement);
  u8g2_DrawStr(&u8g2, 8, 55, buf);
  u8g2_SendBuffer(&u8g2);
}
void CollectPoints_App() {
  u8g2_ClearBuffer(&u8g2);
  char buf[50];
  sprintf(buf, "lat:%.6f", UBXPositionData.latitude);
  u8g2_DrawStr(&u8g2, 8, 35, buf);
  sprintf(buf, "lon:%.6f", UBXPositionData.longitude);
  u8g2_DrawStr(&u8g2, 8, 45, buf);
  sprintf(buf, "goal:%d", car.where_goal_point);
  u8g2_DrawStr(&u8g2, 8, 15, buf);
  sprintf(buf, "max:%d", car.where_goal_max);
  u8g2_DrawStr(&u8g2, 8, 25, buf);
  sprintf(buf, "err:%.3f", UBXPositionData.positionError);
  u8g2_DrawStr(&u8g2, 8, 55, buf);
  if (key_state == LEFT) {
    SwitchUse();
    if (car.where_goal_point > 0) {
      car.where_goal_point--;
    }
    // 切换到上一个点
  }
  if (key_state == RIGHT) {
    SwitchUse();
    if (car.where_goal_point + 1 <
            sizeof(car.goal_point_s) / sizeof(car.goal_point_s[0]) &&
        car.where_goal_point < car.where_goal_max) {
      car.where_goal_point++;
    }
    // 切换到下一个点
  }
  if (key_state == DOWN) {
    SwitchUse();

    // 只有在手动模式下，才可以采点
    if (car.getMoveMode() == manual_ctrl)
      car.setMoveMode(collect_points);

    // 打点
  }
  u8g2_SendBuffer(&u8g2);
}
void ClearPoints_App() {
  u8g2_ClearBuffer(&u8g2);

  if (key_state == UP) {
    // 清除所有点
    SwitchUse();

    car.clearTargetPoints();
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);
    vTaskDelay(200);

    u8g2_DrawStr(&u8g2, 8, 15, "Clear!");
    u8g2_SendBuffer(&u8g2);
    vTaskDelay(200);

  } else {
    u8g2_DrawStr(&u8g2, 8, 15, "press up to clear all points");
  }

  u8g2_SendBuffer(&u8g2);
}
void GNSSP_App() {
  u8g2_ClearBuffer(&u8g2);
  char kp[40];
  sprintf(kp, "Kp:%.4f", car.steerPID_kp[gps_navi]);
  u8g2_DrawStr(&u8g2, 8, 15, "GNSS Kp");
  u8g2_DrawStr(&u8g2, 8, 25, kp);
  if (ChangeParam(car.steerPID_kp[gps_navi], 0.01, 0.001))
    car.updatePIDParam();

  u8g2_SendBuffer(&u8g2);
}
void GNSSI_App() {
  u8g2_ClearBuffer(&u8g2);
  char ki[40];
  sprintf(ki, "Ki:%.4f", car.steerPID_ki[gps_navi]);
  u8g2_DrawStr(&u8g2, 8, 15, "GNSS Ki");
  u8g2_DrawStr(&u8g2, 8, 25, ki);
  if (ChangeParam(car.steerPID_ki[gps_navi], 0.01, 0.001))
    car.updatePIDParam();
  u8g2_SendBuffer(&u8g2);
}
void GNSSD_App() {
  u8g2_ClearBuffer(&u8g2);
  char kd[40];
  sprintf(kd, "Kd:%.4f", car.steerPID_kd[gps_navi]);
  u8g2_DrawStr(&u8g2, 8, 15, "GNSS Kd");
  u8g2_DrawStr(&u8g2, 8, 25, kd);
  if (ChangeParam(car.steerPID_kd[gps_navi], 0.01, 0.001))
    car.updatePIDParam();
  u8g2_SendBuffer(&u8g2);
}
void GNSSL_App() {
  u8g2_ClearBuffer(&u8g2);

  char limit[40];
  sprintf(limit, "limit:%.2f", car.steerPID_max_out[gps_navi]);
  u8g2_DrawStr(&u8g2, 8, 15, "GNSS lim");
  u8g2_DrawStr(&u8g2, 8, 25, limit);
  if (ChangeParam(car.steerPID_max_out[gps_navi], 0.01, 0.001))
    car.updatePIDParam();
  u8g2_SendBuffer(&u8g2);
}
void GNSSS_App() {
  u8g2_ClearBuffer(&u8g2);

  char limit[40];
  sprintf(limit, "spd:%.2f", car.drv_gps_norm);
  u8g2_DrawStr(&u8g2, 8, 15, "GNSS drv");
  u8g2_DrawStr(&u8g2, 8, 25, limit);
  if (ChangeParam(car.drv_gps_norm, 0.1, 0.01))
    car.updatePIDParam();
  u8g2_SendBuffer(&u8g2);
}
void GNSSDIS_App() {
  u8g2_ClearBuffer(&u8g2);

  char limit[40];
  sprintf(limit, "to snd dis:%.2f", car.gps_to_snd_dis);
  u8g2_DrawStr(&u8g2, 8, 15, "GNSS to snd dis");
  u8g2_DrawStr(&u8g2, 8, 25, limit);
  if (ChangeParam(car.gps_to_snd_dis, 1, 0.1))
    car.updatePIDParam();
  u8g2_SendBuffer(&u8g2);
}

void SoundP_App() {
  u8g2_ClearBuffer(&u8g2);
  char kp[40];
  sprintf(kp, "Kp:%.4f", car.steerPID_kp[sound_navi]);
  u8g2_DrawStr(&u8g2, 8, 15, "Sound Kp");
  u8g2_DrawStr(&u8g2, 8, 25, kp);
  if (ChangeParam(car.steerPID_kp[sound_navi], 0.01, 0.001))
    car.updatePIDParam();

  u8g2_SendBuffer(&u8g2);
}
void SoundI_App() {
  u8g2_ClearBuffer(&u8g2);
  char ki[40];
  sprintf(ki, "Ki:%.4f", car.steerPID_ki[sound_navi]);
  u8g2_DrawStr(&u8g2, 8, 15, "Sound Ki");
  u8g2_DrawStr(&u8g2, 8, 25, ki);
  if (ChangeParam(car.steerPID_ki[sound_navi], 0.01, 0.001))
    car.updatePIDParam();
  u8g2_SendBuffer(&u8g2);
}
void SoundD_App() {
  u8g2_ClearBuffer(&u8g2);
  char kd[40];
  sprintf(kd, "Kd:%.4f", car.steerPID_kd[sound_navi]);
  u8g2_DrawStr(&u8g2, 8, 15, "Sound Kd");
  u8g2_DrawStr(&u8g2, 8, 25, kd);
  if (ChangeParam(car.steerPID_kd[sound_navi], 0.01, 0.001))
    car.updatePIDParam();
  u8g2_SendBuffer(&u8g2);
}
void SoundL_App() {
  u8g2_ClearBuffer(&u8g2);

  char limit[40];
  sprintf(limit, "limit:%.2f", car.steerPID_max_out[sound_navi]);
  u8g2_DrawStr(&u8g2, 8, 15, "Sound lim");
  u8g2_DrawStr(&u8g2, 8, 25, limit);
  if (ChangeParam(car.steerPID_max_out[sound_navi], 0.01, 0.001))
    car.updatePIDParam();
  u8g2_SendBuffer(&u8g2);
}
void SoundS_App() {
  u8g2_ClearBuffer(&u8g2);

  char limit[40];
  sprintf(limit, "spd:%.2f", car.drv_snd_norm);
  u8g2_DrawStr(&u8g2, 8, 15, "Sound spd");
  u8g2_DrawStr(&u8g2, 8, 25, limit);
  if (ChangeParam(car.drv_snd_norm, 0.1, 0.01))
    car.updatePIDParam();
  u8g2_SendBuffer(&u8g2);
}
void SoundR_App() {
  u8g2_ClearBuffer(&u8g2);

  char limit[40];
  sprintf(limit, "rvs:%.2f", car.drv_rvs_norm);
  u8g2_DrawStr(&u8g2, 8, 15, "Sound rvs");
  u8g2_DrawStr(&u8g2, 8, 25, limit);
  if (ChangeParam(car.drv_rvs_norm, 0.1, 0.01))
    car.updatePIDParam();
  u8g2_SendBuffer(&u8g2);
}
void SoundDIS_App() {
  u8g2_ClearBuffer(&u8g2);

  char limit[40];
  sprintf(limit, "to gps dis:%.2f", car.snd_to_gps_dis);
  u8g2_DrawStr(&u8g2, 8, 15, "Sound to gps dis");
  u8g2_DrawStr(&u8g2, 8, 25, limit);
  if (ChangeParam(car.snd_to_gps_dis, 1, 0.1))
    car.updatePIDParam();
  u8g2_SendBuffer(&u8g2);
}

void ClosedSpdP_App() {
  u8g2_ClearBuffer(&u8g2);
  char kp[40];
  sprintf(kp, "Kp:%.4f", car.closedSpdPID_kp);
  u8g2_DrawStr(&u8g2, 8, 15, "ClosedSpd Kp");
  u8g2_DrawStr(&u8g2, 8, 25, kp);
  if (ChangeParam(car.closedSpdPID_kp, 0.01, 0.001))
    car.updatePIDParam();

  u8g2_SendBuffer(&u8g2);
}
void ClosedSpdI_App() {
  u8g2_ClearBuffer(&u8g2);
  char ki[40];
  sprintf(ki, "Ki:%.4f", car.closedSpdPID_ki);
  u8g2_DrawStr(&u8g2, 8, 15, "ClosedSpd Ki");
  u8g2_DrawStr(&u8g2, 8, 25, ki);
  if (ChangeParam(car.closedSpdPID_ki, 0.01, 0.001))
    car.updatePIDParam();
  u8g2_SendBuffer(&u8g2);
}
void ClosedSpdD_App() {
  u8g2_ClearBuffer(&u8g2);
  char kd[40];
  sprintf(kd, "Kd:%.4f", car.closedSpdPID_kd);
  u8g2_DrawStr(&u8g2, 8, 15, "ClosedSpd Kd");
  u8g2_DrawStr(&u8g2, 8, 25, kd);
  if (ChangeParam(car.closedSpdPID_kd, 0.01, 0.001))
    car.updatePIDParam();
  u8g2_SendBuffer(&u8g2);
}
void ClosedSpdL_App() {
  u8g2_ClearBuffer(&u8g2);
  char kd[40];
  sprintf(kd, "Lim:%.4f", car.closedSpdPID_max_out);
  u8g2_DrawStr(&u8g2, 8, 15, "gnssSpdPID Lim");
  u8g2_DrawStr(&u8g2, 8, 25, kd);
  if (ChangeParam(car.closedSpdPID_max_out, 0.01, 0.001))
    car.updatePIDParam();
  u8g2_SendBuffer(&u8g2);
}
void ClosedSpdT_App() {
  u8g2_ClearBuffer(&u8g2);
  char kd[40];
  sprintf(kd, "Target:%.4f", car.closed_gps_spd_tgt);
  u8g2_DrawStr(&u8g2, 8, 15, "ClosedGPSSpd Target");
  u8g2_DrawStr(&u8g2, 8, 25, kd);
  if (ChangeParam(car.closed_gps_spd_tgt, 1, 0.1))
    car.updatePIDParam();
  u8g2_SendBuffer(&u8g2);
}
