#include "VMC.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define VMC_EPSILON (1e-9)

VMC_Params_t VMC_Left={
    .l1 = VMC_L1,
    .l2 = VMC_L2,
    .l3 = VMC_L3,
    .l4 = VMC_L4,
    .l5 = VMC_L5,
};
VMC_Params_t VMC_Right={
    .l1 = VMC_L1,
    .l2 = VMC_L2,
    .l3 = VMC_L3,
    .l4 = VMC_L4,
    .l5 = VMC_L5
};
VMC_VirtualForceParams_t VMC_VirtualForceParams = {
    .k_height = 50.0,
    .d_height = 20.0,
    .k_roll   = 30.0,
    .d_roll   = 15.0,
    .k_vel    = 10.0
};

VMC_LegState_t VMC_LeftState = {0};
VMC_LegState_t VMC_RightState = {0};

double fz_left_cmd = 0.0;
double fz_right_cmd = 0.0;
double fx_cmd = 0.0;
double tau_alpha_cmd = 0.0;
double tau_beta_cmd = 0.0;
double J[2][2];

static int VMC_SolveThetaDot(const VMC_Params_t *p,
                             double alpha, double beta,
                             double theta1, double theta2,
                             double alpha_dot, double beta_dot,
                             double *theta1_dot, double *theta2_dot)
{
    if (!p || !theta1_dot || !theta2_dot) return -1;

    double A = p->l1 * sin(alpha) * alpha_dot;
    double B = p->l4 * sin(beta)  * beta_dot;
    double C = p->l1 * cos(alpha) * alpha_dot;
    double D = p->l4 * cos(beta)  * beta_dot;

    double q1 = -B + A;
    double q2 = D - C;

    double det = p->l2 * p->l3 * sin(theta1 - theta2);
    if (fabs(det) < VMC_EPSILON) {
        return -2; // 奇异点：l2*l3*sin(theta1-theta2) 近 0
    }

    *theta1_dot = (-p->l3 * cos(theta2) * q1 - p->l3 * sin(theta2) * q2) / det;
    *theta2_dot = (-p->l2 * cos(theta1) * q1 - p->l2 * sin(theta1) * q2) / det;

    return 0;
}


static void VMC_FootVelocityFromRate(const VMC_Params_t *p,
                                     double alpha, double theta1,
                                     double alpha_dot, double theta1_dot,
                                     double *x_dot, double *y_dot)
{
    if (!p || !x_dot || !y_dot) return;

    *x_dot = -p->l1 * sin(alpha) * alpha_dot - p->l2 * sin(theta1) * theta1_dot;
    *y_dot =  p->l1 * cos(alpha) * alpha_dot + p->l2 * cos(theta1) * theta1_dot;
}


int VMC_ComputeJacobian(const VMC_Params_t *params,
                        double alpha, double beta,
                        double theta1, double theta2,
                        double J[2][2])
{
    if (!params || !J) return -1;

    double t1dot, t2dot;

    // 列 0: alpha_dot = 1, beta_dot = 0
    int ret = VMC_SolveThetaDot(params, alpha, beta, theta1, theta2, 1.0, 0.0, &t1dot, &t2dot);
    if (ret != 0) return ret;
    VMC_FootVelocityFromRate(params, alpha, theta1, 1.0, t1dot, &J[0][0], &J[1][0]);

    // 列 1: alpha_dot = 0, beta_dot = 1
    ret = VMC_SolveThetaDot(params, alpha, beta, theta1, theta2, 0.0, 1.0, &t1dot, &t2dot);
    if (ret != 0) return ret;
    VMC_FootVelocityFromRate(params, alpha, theta1, 0.0, t1dot, &J[0][1], &J[1][1]);

    return 0;
}

int VMC_ComputeLegTheta(const VMC_Params_t *params, VMC_LegState_t *leg, double x_fk, double y_fk)
{
    if (!params || !leg) return -1;

    double cos_beta = cos(leg->beta);
    double sin_beta = sin(leg->beta);

    double a = (x_fk - (params->l5 + params->l4 * cos_beta)) / params->l3;
    double b = (y_fk - (params->l4 * sin_beta)) / params->l3;

    if (a < -1.0) a = -1.0;
    if (a >  1.0) a =  1.0;
    if (b < -1.0) b = -1.0;
    if (b >  1.0) b =  1.0;

    leg->theta2 = acos(a);
    leg->theta1 = asin(b);

    return 0;
}

static double VMC_NormalizeAngle(double angle)
{
    while (angle < 0.0) angle += 2.0 * M_PI;
    while (angle >= 2.0 * M_PI) angle -= 2.0 * M_PI;
    return angle;
}

int VMC_ForwardKinematics(const VMC_Params_t *params,
                           double alpha, double beta,
                           VMC_FKResult_t *result)
{
    if (!params || !result) return -1;

    double xa = params->l1 * cos(alpha);
    double ya = params->l1 * sin(alpha);
    double xc = params->l5 + params->l4 * cos(beta);
    double yc = params->l4 * sin(beta);

    double a = 2.0 * (xa - xc) * params->l2;
    double b = 2.0 * (ya - yc) * params->l2;
    double dx = xa - xc;
    double dy = ya - yc;
    double LAC = sqrt(dx * dx + dy * dy);
    double c = params->l3 * params->l3 - params->l2 * params->l2 - LAC * LAC;

    double sq_arg = a * a + b * b - c * c;
    if (sq_arg < 0.0) {
        if (sq_arg > -VMC_EPSILON) sq_arg = 0.0;
        else return -2; // 无解（超出工作空间）
    }

    double sqrt_val = sqrt(sq_arg);
    double denom = a + c;

    double theta1_1 = 2.0 * atan2(b + sqrt_val, denom);
    double theta1_2 = 2.0 * atan2(b - sqrt_val, denom);

    // 归一到 [0, 2*pi)
    theta1_1 = VMC_NormalizeAngle(theta1_1);
    theta1_2 = VMC_NormalizeAngle(theta1_2);

    double theta1 = theta1_1 < (M_PI / 2.0) ? theta1_1 : theta1_2;

    double x = params->l1 * cos(alpha) + params->l2 * cos(theta1);
    double y = params->l1 * sin(alpha) + params->l2 * sin(theta1);

    double x1 = params->l1 * cos(alpha) + params->l2 * cos(theta1_1);
    double y1 = params->l1 * sin(alpha) + params->l2 * sin(theta1_1);

    double x2 = params->l1 * cos(alpha) + params->l2 * cos(theta1_2);
    double y2 = params->l1 * sin(alpha) + params->l2 * sin(theta1_2);

    result->xa = xa;
    result->ya = ya;
    result->xc = xc;
    result->yc = yc;

    result->theta1_1 = theta1_1;
    result->theta1_2 = theta1_2;
    result->theta1 = theta1;

    result->x = x;
    result->y = y;

    result->x1 = x1;
    result->y1 = y1;
    result->x2 = x2;
    result->y2 = y2;

    return 0;
}

static double hip_alpha_left_raw_deg = 0.0;
static double hip_beta_left_raw_deg  = 0.0;
static double hip_alpha_right_raw_deg = 0.0;
static double hip_beta_right_raw_deg  = 0.0;
static double hip_alpha_left_rad = M_PI / 2.0; // 默认 90 度
static double hip_beta_left_rad  = M_PI / 2.0;
static double hip_alpha_right_rad = M_PI / 2.0;
static double hip_beta_right_rad  = M_PI / 2.0;

static void update_rad_from_raw(double raw_alpha, double raw_beta, double *alpha_rad, double *beta_rad) {
    double alpha_deg = raw_alpha + 90.0;
    double beta_deg = raw_beta + 90.0;
    *alpha_rad = alpha_deg * M_PI / 180.0;
    *beta_rad  = beta_deg  * M_PI / 180.0;
}

void VMC_SetHipRawAngles(double alpha_raw_deg, double beta_raw_deg)
{
    // 兼容旧接口：全局左右相同
    VMC_SetHipRawAnglesLeft(alpha_raw_deg, beta_raw_deg);
    VMC_SetHipRawAnglesRight(alpha_raw_deg, beta_raw_deg);
}

void VMC_SetHipRawAnglesLeft(double alpha_raw_deg, double beta_raw_deg)
{
    hip_alpha_left_raw_deg = alpha_raw_deg;
    hip_beta_left_raw_deg  = beta_raw_deg;
    update_rad_from_raw(alpha_raw_deg, beta_raw_deg,
                        &hip_alpha_left_rad, &hip_beta_left_rad);
    VMC_LeftState.alpha = hip_alpha_left_rad;
    VMC_LeftState.beta  = hip_beta_left_rad;
    // 在计算前填充，会在任务中计算 theta1/theta2
}

void VMC_SetHipRawAnglesRight(double alpha_raw_deg, double beta_raw_deg)
{
    hip_alpha_right_raw_deg = alpha_raw_deg;
    hip_beta_right_raw_deg  = beta_raw_deg;
    update_rad_from_raw(alpha_raw_deg, beta_raw_deg,
                        &hip_alpha_right_rad, &hip_beta_right_rad);
    VMC_RightState.alpha = hip_alpha_right_rad;
    VMC_RightState.beta  = hip_beta_right_rad;
}

void VMC_GetHipRawAnglesDeg(double *alpha_raw_deg, double *beta_raw_deg)
{
    // 兼容旧接口：返回左腿
    if (alpha_raw_deg) *alpha_raw_deg = hip_alpha_left_raw_deg;
    if (beta_raw_deg)  *beta_raw_deg  = hip_beta_left_raw_deg;
}

void VMC_GetHipRawAnglesDegLeft(double *alpha_raw_deg, double *beta_raw_deg)
{
    if (alpha_raw_deg) *alpha_raw_deg = hip_alpha_left_raw_deg;
    if (beta_raw_deg)  *beta_raw_deg  = hip_beta_left_raw_deg;
}

void VMC_GetHipRawAnglesDegRight(double *alpha_raw_deg, double *beta_raw_deg)
{
    if (alpha_raw_deg) *alpha_raw_deg = hip_alpha_right_raw_deg;
    if (beta_raw_deg)  *beta_raw_deg  = hip_beta_right_raw_deg;
}

void VMC_GetHipAnglesRad(double *alpha_rad, double *beta_rad)
{
    // 兼容旧接口：返回左腿
    if (alpha_rad) *alpha_rad = hip_alpha_left_rad;
    if (beta_rad)  *beta_rad  = hip_beta_left_rad;
}

void VMC_GetHipAnglesRadLeft(double *alpha_rad, double *beta_rad)
{
    if (alpha_rad) *alpha_rad = hip_alpha_left_rad;
    if (beta_rad)  *beta_rad  = hip_beta_left_rad;
}

void VMC_GetHipAnglesRadRight(double *alpha_rad, double *beta_rad)
{
    if (alpha_rad) *alpha_rad = hip_alpha_right_rad;
    if (beta_rad)  *beta_rad  = hip_beta_right_rad;
}

int VMC_ComputeVirtualForces(const VMC_VirtualForceParams_t *vf,
                              double height_error, double height_error_dot,
                              double roll_error, double roll_error_dot,
                              double velocity_error,
                              double *fz_left, double *fz_right, double *fx)
{
    if (!vf || !fz_left || !fz_right || !fx) return -1;

    double fhight = vf->k_height * height_error + vf->d_height * height_error_dot;
    double froll  = vf->k_roll * roll_error + vf->d_roll * roll_error_dot;

    *fz_left  = fhight + froll;
    *fz_right = fhight - froll;
    *fx = vf->k_vel * velocity_error;

    return 0;
}


int VMC_ComputeJointTorqueFromFootForce(const VMC_Params_t *params,
                                        double alpha, double beta,
                                        double theta1, double theta2,
                                        double Fx, double Fy,
                                        double *tau_alpha, double *tau_beta)
{
    if (!params || !tau_alpha || !tau_beta) return -1;

    double J[2][2];
    int ret = VMC_ComputeJacobian(params, alpha, beta, theta1, theta2, J);
    if (ret != 0) return ret;

    *tau_alpha = J[0][0] * Fx + J[1][0] * Fy;
    *tau_beta  = J[0][1] * Fx + J[1][1] * Fy;

    return 0;
}
