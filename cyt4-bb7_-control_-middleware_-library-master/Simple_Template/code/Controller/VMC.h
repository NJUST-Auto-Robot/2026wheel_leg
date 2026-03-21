#ifndef VMC_H
#define VMC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 常量，可按机械结构修改
#ifndef VMC_L1
#define VMC_L1 0.06
#endif
#ifndef VMC_L2
#define VMC_L2 0.09
#endif
#ifndef VMC_L3
#define VMC_L3 0.09
#endif
#ifndef VMC_L4
#define VMC_L4 0.06
#endif
#ifndef VMC_L5
#define VMC_L5 0.136
#endif

typedef struct {
    double l1;
    double l2;
    double l3;
    double l4;
    double l5;
} VMC_Params_t;

// 单腿实时状态
typedef struct {
    double alpha;   // 髋关节前后角（rad）
    double beta;    // 髋关节左右角（rad）
    double theta1;  // 后髋闭链角（rad）
    double theta2;  // 前髋闭链角（rad）
} VMC_LegState_t;

// 前向运动学结果（基于 alpha/beta 计算theta1/2及足端）
typedef struct {
    double xa;
    double ya;
    double xc;
    double yc;

    double theta1_1;
    double theta1_2;
    double theta1;

    double x;
    double y;

    double x1;
    double y1;
    double x2;
    double y2;
} VMC_FKResult_t;

// 初始化参数 (默认或自定义)
static inline VMC_Params_t VMC_DefaultParams(void)
{
    VMC_Params_t p;
    p.l1 = VMC_L1;
    p.l2 = VMC_L2;
    p.l3 = VMC_L3;
    p.l4 = VMC_L4;
    p.l5 = VMC_L5;
    return p;
}

// 计算雅可比矩阵 J (2x2)，输入角度 alpha/beta 和闭链内角 theta1/theta2
// 返回 0 表示成功，非 0 表示奇异/无效
int VMC_ComputeJacobian(const VMC_Params_t *params,
                        double alpha, double beta,
                        double theta1, double theta2,
                        double J[2][2]);

// 根据足端力 Fx,Fy 计算对应关节扭矩 tau_alpha,tau_beta
// 也可以用已知的雅可比矩阵直接调用 VMC_ForceToTorque
int VMC_ComputeJointTorqueFromFootForce(const VMC_Params_t *params,
                                         double alpha, double beta,
                                         double theta1, double theta2,
                                         double Fx, double Fy,
                                         double *tau_alpha, double *tau_beta);

// 前向运动学：从 hip alpha,beta 计算足端坐标与两种 theta1 可能解
int VMC_ForwardKinematics(const VMC_Params_t *params,
                           double alpha, double beta,
                           VMC_FKResult_t *result);

// 虚拟弹簧-阻尼补偿参数
typedef struct {
    double k_height;    // 高度环刚度
    double d_height;    // 高度环阻尼
    double k_roll;      // 侧倾环刚度
    double d_roll;      // 侧倾环阻尼
    double k_vel;       // 纵向速度补偿增益
} VMC_VirtualForceParams_t;

// 计算虚拟力：fz_left = fhight + froll, fz_right = fhight - froll
// 以及速度补偿力 fx。
int VMC_ComputeVirtualForces(const VMC_VirtualForceParams_t *vf,
                             double height_error, double height_error_dot,
                             double roll_error, double roll_error_dot,
                             double velocity_error,
                             double *fz_left, double *fz_right, double *fx);

// 机械角度平台：raw 来自串口，计算后增加偏置 (alpha_raw+90, beta_raw+90)
void VMC_SetHipRawAngles(double alpha_raw_deg, double beta_raw_deg); // 兼容旧接口（左右同值）
void VMC_SetHipRawAnglesLeft(double alpha_raw_deg, double beta_raw_deg);
void VMC_SetHipRawAnglesRight(double alpha_raw_deg, double beta_raw_deg);

void VMC_GetHipRawAnglesDeg(double *alpha_raw_deg, double *beta_raw_deg); // 返回左腿
void VMC_GetHipRawAnglesDegLeft(double *alpha_raw_deg, double *beta_raw_deg);
void VMC_GetHipRawAnglesDegRight(double *alpha_raw_deg, double *beta_raw_deg);

void VMC_GetHipAnglesRad(double *alpha_rad, double *beta_rad); // 返回左腿
void VMC_GetHipAnglesRadLeft(double *alpha_rad, double *beta_rad);
void VMC_GetHipAnglesRadRight(double *alpha_rad, double *beta_rad);

int VMC_ComputeVirtualForces(const VMC_VirtualForceParams_t *vf,
                              double height_error, double height_error_dot,
                              double roll_error, double roll_error_dot,
                              double velocity_error,
                              double *fz_left, double *fz_right, double *fx);
int VMC_ComputeJointTorqueFromFootForce(const VMC_Params_t *params,
                                        double alpha, double beta,
                                        double theta1, double theta2,
                                        double Fx, double Fy,
                                        double *tau_alpha, double *tau_beta);

int VMC_ComputeLegTheta(const VMC_Params_t *params, VMC_LegState_t *leg, double x_fk, double y_fk);                              

extern double fz_left_cmd;
extern double fz_right_cmd;
extern double fx_cmd;
extern double tau_alpha_cmd;
extern double tau_beta_cmd;
extern double J[2][2];
extern VMC_Params_t VMC_Left;
extern VMC_Params_t VMC_Right;
extern VMC_LegState_t VMC_LeftState;
extern VMC_LegState_t VMC_RightState;
extern VMC_VirtualForceParams_t VMC_VirtualForceParams;

#ifdef __cplusplus
}
#endif

#endif // VMC_H
