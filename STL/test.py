import mujoco
import mujoco.viewer as viewer
import numpy as np
from scipy.linalg import solve_continuous_are, inv
import threading
import sys
import termios
import tty
import os
import sympy as sp

# ===================== 0. 辅助函数 =====================
def quat2euler(quat):
    """将四元数转换为欧拉角 (roll, pitch, yaw)
    输入：quat = [w, x, y, z]
    输出：euler = [roll, pitch, yaw]
    欧拉角顺序：ZYX (索引 0 为 roll/X 轴，索引 1 为 pitch/Y 轴)
    修改：x 与 y 轴互换后，pitch 对应 X 轴旋转（索引 0）
    """
    w, x, y, z = quat
    
    # Roll (Φ) - 绕 X 轴旋转（修改：原为 pitch，现作为 roll）
    sinh = 2.0 * (w * x + y * z)
    cosh = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinh, cosh)
    
    # Pitch (Θ) - 绕 Y 轴旋转，需裁剪防止数值溢出
    sin_p = 2.0 * (w * y - z * x)
    sin_p = np.clip(sin_p, -1.0, 1.0)
    pitch = np.arcsin(sin_p)
    
    # Yaw (Ψ) - 绕 Z 轴旋转
    sinh = 2.0 * (w * z + x * y)
    cosh = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(sinh, cosh)
    
    return np.array([roll, pitch, yaw])


def euler2quat(euler):
    """将欧拉角转换为四元数 (roll, pitch, yaw) -> [w, x, y, z]
    输入：euler = [roll, pitch, yaw]
    输出：quat = [w, x, y, z]
    """
    roll, pitch, yaw = euler
    
    # 计算半角
    cr = np.cos(roll / 2.0)
    sr = np.sin(roll / 2.0)
    cp = np.cos(pitch / 2.0)
    sp = np.sin(pitch / 2.0)
    cy = np.cos(yaw / 2.0)
    sy = np.sin(yaw / 2.0)
    
    # 组合四元数
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return np.array([w, x, y, z])


# ===================== 1. 核心参数配置 =====================
XML_PATH = "output_base.xml"

N_STATES = 4
N_CONTROLS = 2

X_REF = np.zeros(N_STATES)

g = 9.8

VERBOSE = True

# 新增：VMC 调试日志开关（修改：独立控制 VMC 日志输出）
VMC_VERBOSE = False

K_HIP = 500.0
D_HIP = 50.0

CTRL_LIMIT = 30.0

# 新增：驱动控制参数
TARGET_VELOCITY = 0.0  # 目标速度 (m/s)
TARGET_POSITION = None  # 目标位置 (None 表示不启用位置控制)
K_VEL = 5.0  # 速度控制增益 (修改：从 3.0 增加至 5.0)
K_POS = 2.0  # 位置控制增益

# 新增：轮子半径（用于速度转换）
WHEEL_RADIUS = 0.08  # 轮子半径 (m)

# 新增：腿部几何参数（用于雅克比矩阵计算）
L1 = 0.06  # 大腿长度 (m) - l1 
L2 = 0.09  # 小腿长度 (m) - l2
L3 = 0.09  # 小腿长度 (m) - l3
L4 = 0.06  # 大腿长度 (m) - l4 
L5 = 0.136  # 髋关节距离 (m) - l5

# 新增：腿部关节名称映射（需与 XML 模型一致）
HIP_JOINTS = ["left_behind_hip_joint", "right_behind_hip_joint", "left_front_hip_joint", "right_front_hip_joint"]
# 新增：膝关节名称映射（4 个膝关节）
KNEE_JOINTS = ["left_behind_knee_joint", "right_behind_knee_joint", "left_front_knee_joint", "right_front_knee_joint"]
LEG_ACTUATORS = [
    "left_behind_hip_motor", "right_behind_hip_motor",
    "left_front_hip_motor", "right_front_hip_motor",
]
# 新增：膝关节执行器名称映射
KNEE_ACTUATORS = [
    "left_behind_knee_motor", "right_behind_knee_motor",
    "left_front_knee_motor", "right_front_knee_motor",
]

# 新增：髋关节减速箱参数
HIP_GEAR_RATIO = 5.18  # 髋关节减速比 (修改：电机扭矩×减速比=关节扭矩)
# 新增：膝关节减速箱参数
KNEE_GEAR_RATIO = 5.18  # 膝关节减速比

# 新增：膝关节限位参数（根据前后腿分别设置）
KNEE_LIMIT_ANGLE_REAR_MIN = np.deg2rad(-60.0)  # 后膝关节最小限位角度 (rad)
KNEE_LIMIT_ANGLE_REAR_MAX = np.deg2rad(30.0)   # 后膝关节最大限位角度 (rad)
KNEE_LIMIT_ANGLE_FRONT_MIN = np.deg2rad(-30.0) # 前膝关节最小限位角度 (rad)
KNEE_LIMIT_ANGLE_FRONT_MAX = np.deg2rad(60.0)  # 前膝关节最大限位角度 (rad)
# 修改：恢复初始膝关节限位增益
K_KNEE_LIMIT = 200.0  # 膝关节限位比例增益 (修改：从 250.0 调回 200.0，恢复初始刚性)
D_KNEE_LIMIT = 40.0  # 膝关节限位微分增益 (修改：从 50.0 调回 40.0，恢复初始阻尼)
# 修改：膝关节限位激活阈值保持 15 度（与初始值一致）
KNEE_LIMIT_THRESHOLD = np.deg2rad(15.0)  # 距离限位 15 度开始激活限位 (修改：保持 15 度)

# 新增：髋关节扭矩限幅参数（修复未定义变量）
HIP_TORQUE_LIMIT = 30.0  # 髋关节电机扭矩限幅 (N·m)

# 新增：VMC 控制参数
TARGET_HEIGHT = 0.35  # 目标车身高度 (m)
K_HEIGHT = 50.0  # 高度控制弹簧刚度 (修改：从 30.0 增加至 50.0，加快高度收敛)
D_HEIGHT = 20.0  # 高度控制阻尼系数 (修改：从 15.0 增加至 20.0，增强阻尼抑制振荡)

# 修改：增加平衡控制增益，应对大角度倾斜
K_BALANCE = 15.0  # 平衡控制增益 (修改：从 5.0 增加至 15.0，增强俯仰响应强度)
D_BALANCE = 20.0  # 平衡控制阻尼 (修改：从 8.0 增加至 20.0，增强俯仰阻尼)

K_VEL_COMP_X = 10.0  # X 方向速度补偿增益 (修改：分离 X/Y 方向控制)
D_VEL_COMP_X = 8.0  # X 方向速度阻尼 (修改：增加阻尼抑制漂移)
K_VEL_COMP_Y = 15.0  # Y 方向速度补偿增益 (修改：增强侧向稳定性)
D_VEL_COMP_Y = 10.0  # Y 方向速度阻尼 (修改：增强侧向阻尼)

# 新增：重力补偿参数
MASS_COMP = 5.0  # 车身质量估计值 (kg)，用于重力补偿 (修改：从 3.0 增加到 5.0)
G_COMP = 9.8  # 重力加速度

# 新增：运行模式配置
HEADLESS_MODE = os.environ.get("HEADLESS", "false").lower() == "true"  # 无头模式


# ===================== 2. 模型加载与基础函数 =====================
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)


def compute_theoretical_AB(data, model, r=0.08):
    """从模型参数计算四维轮子驱动倒立摆线性化 A、B 矩阵。
    状态：[wheel_pos, wheel_vel, body_pitch, body_pitch_vel]
    控制：轮子力矩 u
    
    参数：
        data: MuJoCo 数据对象
        model: MuJoCo 模型对象
        r: 轮子半径 (默认 0.0935m)
    
    返回：
        A: 4x4 状态矩阵
        B: 4x1 输入矩阵
    """
    right_wheel_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_wheel_body")
    if right_wheel_id == -1:
        raise ValueError("未找到名为 'right_wheel' 的刚体，请检查模型定义。")
    left_wheel_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_wheel_body")
    if left_wheel_id == -1:
        raise ValueError("未找到名为 'left_wheel' 的刚体，请检查模型定义。")
    total_mass = float(np.sum(model.body_mass))
    m = total_mass - model.body_mass[right_wheel_id]-model.body_mass[left_wheel_id]  # 躯干质量
    M = model.body_mass[right_wheel_id] + model.body_mass[left_wheel_id]  # 轮子等其他质量
    
    # 获取重力加速度，优先从模型配置获取
    g = abs(model.opt.gravity[2]) if model.opt.gravity[2] != 0 else 9.81
    
    # 使用躯干高度作为摆长近似 (qpos[2] 为自由根节点的 z 位置)
    h = 0.1668  # 直接使用已知的摆长值，避免从模型中提取可能不准确的高度
    
    # 检查除零风险
    if h == 0:
        raise ValueError("摆长 h 为 0，无法计算线性化矩阵。")
    if r == 0:
        raise ValueError("轮子半径 r 为 0，无法计算线性化矩阵。")
    if total_mass == 0:
        raise ValueError("总质量为 0，无法计算线性化矩阵。")
    
    I_body = m * h * h / 3.0
    
    # 系统总惯量 (轮子平动 + 躯干转动)
    I_total = (M + m) * I_body + (M + m) * m * h * h
    
    # 线性化 A 矩阵 (4x4)
    # 基于拉格朗日方程推导的标准轮式倒立摆模型
    A = np.array([
        [0, 1, 0, 0],                                    # d(wheel_pos)/dt = wheel_vel
        [0, 0, -(m * m * g * h * h) / I_total, 0],       # d(wheel_vel)/dt 耦合项
        [0, 0, 0, 1],                                    # d(body_pitch)/dt = body_pitch_vel
        [0, 0, ((M + m) * m * g * h) / I_total, 0]       # d(body_pitch_vel)/dt 重力项
    ])
    
    # 线性化 B 矩阵 (4x1)
    # 控制输入为轮子力矩 τ，通过 F = τ/r 转化为水平力
    B = np.array([
        [0],
        [(I_body + m * h * h) / (I_total * r)],          # 轮子加速度项
        [0],
        [-(M + m) * h / (I_total * r)]                   # 躯干角加速度项
    ])
    return A, B


# 关节/执行器名称映射（需与你的 XML 模型一致）
WHEEL_JOINTS = ["left_wheel_joint", "right_wheel_joint"]
ACTUATORS = [
    "left_wheel_motor", "right_wheel_motor",
]


def get_state(data):
    """采集四维状态向量：[wheel_pos, wheel_vel, body_pitch, body_pitch_vel]
    
    状态说明：
        - wheel_pos: 左轮关节位置 (假设左右对称)
        - wheel_vel: 左轮关节速度
        - body_pitch: 躯干俯仰角 (从四元数提取)
        - body_pitch_vel: 躯干俯仰角速度 (角速度 x 分量，修改：原为 y 分量)
    
    MuJoCo 自由根节点数据结构：
        - qpos[0:3]: 位置 (x, y, z)
        - qpos[3:7]: 四元数 (w, x, y, z)
        - qvel[0:3]: 线速度 (vx, vy, vz)
        - qvel[3:6]: 角速度 (wx, wy, wz)
    
    返回：
        np.array: 4 维状态向量
    """
    # 安全性检查：确保数据向量长度足够，防止索引越界
    assert len(data.qpos) >= 7, f"data.qpos 长度不足 (当前{len(data.qpos)}), 无法提取基座四元数"
    assert len(data.qvel) >= 5, f"data.qvel 长度不足 (当前{len(data.qvel)}), 无法提取基座角速度"
    
    # 轮子位移：使用左轮关节位置（假设对称）
    # 注意：若关节不存在，此处会抛出异常，建议确保模型加载正确
    wheel_pos = data.joint("left_wheel_joint").qpos[0]
    
    # 轮子速度
    wheel_vel = data.joint("left_wheel_joint").qvel[0]
    
    # 躯干俯仰角：从四元数提取 pitch
    # qpos[3:7] 对应四元数 [w, x, y, z]
    base_quat = data.qpos[3:7]
    base_euler = quat2euler(base_quat)
    body_pitch = base_euler[0]  # 修改：pitch 角 (索引 0 对应 X 轴旋转，原为索引 1)
    
    # 躯干俯仰角速度：角速度的 x 分量
    # qvel[3:6] 对应角速度 [wx, wy, wz]，索引 3 为 wx（修改：原为索引 4 的 wy）
    body_pitch_vel = data.qvel[3]
    
    return np.array([wheel_pos, wheel_vel, body_pitch, body_pitch_vel])


def set_actuator_ctrl(data, u, fix_hips=True, k_hip=K_HIP, d_hip=D_HIP):
    """将控制输入映射到 MuJoCo 执行器
    
    参数：
        data: MuJoCo 数据对象
        u: 2 维控制输入，只有轮子电机
        fix_hips: 是否固定髋关节 (已锁定，无需控制)
        k_hip: 髋关节比例增益
        d_hip: 髋关节微分增益
    """
    if VERBOSE:
        print(f"控制输入 u: {u}")
    
    for i, act_name in enumerate(ACTUATORS):
        # 轮子电机：使用 LQR 控制
        # 注意：如果轮子转的方向与预期相反，可能需要在这里调整符号
        # 尝试：u_clamped = np.clip(-u[i], -CTRL_LIMIT, CTRL_LIMIT)  # 如果方向反了
        u_clamped = np.clip(u[i], -CTRL_LIMIT, CTRL_LIMIT)
        
        data.actuator(act_name).ctrl[0] = u_clamped
        
        if VERBOSE:
            print(f"  {act_name}: {u_clamped}")


def reset_to_equilibrium(model, data):
    """重置机器人到平衡态
    
    初始状态设置：
        - 位置：原点 (0, 0, 0.35)  # 修改：与 XML 保持一致，提高初始高度
        - 姿态：直立 (单位四元数)
        - 速度：全部为零
        - 关节：全部归零
    """
    mujoco.mj_resetData(model, data)
    
    # 设置自由根关节的位置和姿态
    # qpos[0:3] = 位置 (x, y, z)
    # qpos[3:7] = 四元数 (w, x, y, z)
    data.qpos[0] = 0.0   # x
    data.qpos[1] = 0.0   # y
    data.qpos[2] = 0.35  # z (修改：与 XML 基座位置保持一致，避免轮子穿地)
    
    # 使用单位四元数表示直立姿态 [1, 0, 0, 0]
    # 如需小角度扰动，使用正确的四元数表示
    initial_pitch = 0  # 修改：初始俯仰角
    data.qpos[3] = np.cos(initial_pitch / 2.0)   # w
    data.qpos[4] = 0.0                            # x
    data.qpos[5] = np.sin(initial_pitch / 2.0)   # y
    data.qpos[6] = 0.0                            # z
    
    # 仅设置轮子关节为 0
    for j in WHEEL_JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if joint_id != -1:
            # 修改：使用 int() 确保索引为整数类型，避免 IndexError
            qpos_adr = int(model.jnt_qposadr[joint_id])
            data.qpos[qpos_adr] = 0.0
            # 新增：同时重置关节速度为 0
            # 修改：使用 jnt_dofadr 而非 jnt_vel（MuJoCo 正确属性）
            qvel_adr = int(model.jnt_dofadr[joint_id])
            data.qvel[qvel_adr] = 0.0
    
    # 重置所有速度为 0
    data.qvel[:] = 0.0
    
    # 重置执行器控制
    for act in ACTUATORS:
        data.actuator(act).ctrl[0] = 0.0
    
    # 前向计算以更新导出的量
    mujoco.mj_forward(model, data)


def set_target_velocity(velocity):
    """设置目标速度
    
    参数：
        velocity: 目标速度 (m/s)，正数向前，负数向后
    """
    global TARGET_VELOCITY, TARGET_POSITION
    TARGET_VELOCITY = velocity
    TARGET_POSITION = None
    if VERBOSE:
        print(f"[控制接口] 目标速度设置为：{velocity} m/s")


def set_target_position(position):
    """设置目标位置
    
    参数：
        position: 目标位置 (m)
    """
    global TARGET_POSITION, TARGET_VELOCITY
    TARGET_POSITION = position
    TARGET_VELOCITY = 0.0
    if VERBOSE:
        print(f"[控制接口] 目标位置设置为：{position} m")


def compute_drive_torque(data, model, wheel_pos, wheel_vel):
    """计算驱动扭矩 u2（速度/位置控制）
    
    参数：
        data: MuJoCo 数据对象
        model: MuJoCo 模型对象
        wheel_pos: 当前轮子关节位置 (rad)
        wheel_vel: 当前轮子关节角速度 (rad/s)
    
    返回：
        u_drive: 驱动扭矩 (2 维数组，左右轮相同)
    """
    global TARGET_VELOCITY, TARGET_POSITION, K_VEL, K_POS, WHEEL_RADIUS
    
    u_drive = np.zeros(N_CONTROLS)
    
    # 将轮子角速度转换为线速度：v = ω * r
    wheel_linear_vel = wheel_vel * WHEEL_RADIUS
    
    # 将轮子关节位置转换为线位移：s = θ * r
    wheel_linear_pos = wheel_pos * WHEEL_RADIUS
    
    if TARGET_POSITION is not None:
        # 位置控制模式
        pos_error = TARGET_POSITION - wheel_linear_pos
        u_drive[0] = K_POS * pos_error - K_VEL * wheel_linear_vel
        u_drive[1] = u_drive[0]
        if VERBOSE and np.random.rand() < 0.01:
            print(f"[驱动控制] 位置模式：error={pos_error:.4f}, u_drive={u_drive[0]:.4f}")
    else:
        # 速度控制模式
        vel_error = TARGET_VELOCITY - wheel_linear_vel
        u_drive[0] = K_VEL * vel_error
        u_drive[1] = u_drive[0]
        if VERBOSE and np.random.rand() < 0.01:
            print(f"[驱动控制] 速度模式：target={TARGET_VELOCITY:.4f}, current={wheel_linear_vel:.4f}, error={vel_error:.4f}, u_drive={u_drive[0]:.4f}")
    
    return np.clip(u_drive, -CTRL_LIMIT/2, CTRL_LIMIT/2)


# ===================== 新增：雅克比矩阵计算模块 =====================
def compute_jacobian_symbolic():
    """使用 sympy 符号计算足端速度与关节速度的雅克比矩阵
    
    运动学模型：
        - l1, l4: 大腿长度
        - l2, l3: 小腿长度
        - l5: 髋关节距离
        - alpha, beta: 大腿与水平线夹角
        - theta1, theta2: 小腿与水平线夹角
    
    返回：
        J: 2x2 雅克比矩阵 (足端速度 = J * 关节速度)
        J_func: 可数值计算的 lambda 函数
    """
    # 定义符号变量
    l1, l2, l3, l4, l5 = sp.symbols('l1 l2 l3 l4 l5', real=True, positive=True)
    t = sp.symbols('t', real=True)
    alpha = sp.Function('alpha')(t)
    beta = sp.Function('beta')(t)
    theta1 = sp.Function('theta1')(t)
    theta2 = sp.Function('theta2')(t)
    alpha_dot, beta_dot = sp.symbols('alpha_dot beta_dot', real=True)
    theta1_dot, theta2_dot = sp.symbols('theta1_dot theta2_dot', real=True)
    
    # A 点坐标（左大腿末端）
    x_A = l1 * sp.cos(alpha)
    y_A = l1 * sp.sin(alpha)
    
    # B 点坐标（足端）
    x_B = x_A + l2 * sp.cos(theta1)
    y_B = y_A + l2 * sp.sin(theta1)
    
    # C 点坐标（右大腿末端）
    x_C = l5 + l4 * sp.cos(beta)
    y_C = l4 * sp.sin(beta)
    
    # 速度约束方程（闭环运动学）
    # 足端速度 = A 点速度 + 小腿相对速度 = C 点速度 + 右小腿相对速度
    eq1 = sp.diff(x_A, t) - l2 * theta1_dot * sp.sin(theta1) - (sp.diff(x_C, t) - l3 * theta2_dot * sp.sin(theta2))
    eq2 = sp.diff(y_A, t) + l2 * theta1_dot * sp.cos(theta1) - (sp.diff(y_C, t) + l3 * theta2_dot * sp.cos(theta2))
    
    # 求解 theta1_dot, theta2_dot
    S = sp.solve([eq1, eq2], [theta1_dot, theta2_dot])
    theta1_dot_sol = sp.simplify(S[theta1_dot])
    theta2_dot_sol = sp.simplify(S[theta2_dot])
    
    # 足端速度表达式
    x_B_dot = sp.diff(x_B, t)
    y_B_dot = sp.diff(y_B, t)
    
    # 代入 theta1_dot 的解
    v_x = x_B_dot.subs(sp.diff(theta1, t), theta1_dot_sol)
    v_y = y_B_dot.subs(sp.diff(theta1, t), theta1_dot_sol)
    
    # 代入 alpha_dot, beta_dot
    v_x = v_x.subs([sp.diff(alpha, t), sp.diff(beta, t)], [alpha_dot, beta_dot])
    v_y = v_y.subs([sp.diff(alpha, t), sp.diff(beta, t)], [alpha_dot, beta_dot])
    
    # 构建速度向量和关节速度向量
    v = sp.Matrix([v_x, v_y])
    q_dot = sp.Matrix([alpha_dot, beta_dot])
    
    # 计算雅克比矩阵 J = dv/dq_dot
    J = sp.simplify(sp.collect(v, q_dot).jacobian(q_dot))
    
    # 创建数值计算函数
    J_func = sp.lambdify([l1, l2, l3, l4, l5, alpha, beta, theta1, theta2], J, 'numpy')
    
    return J, J_func

def compute_jacobian_numeric(alpha, beta, theta1, theta2, l1=L1, l2=L2, l3=L3, l4=L4, l5=L5):
    """数值计算雅克比矩阵
    
    参数：
        alpha: 左大腿与水平线夹角 (rad)
        beta: 右大腿与水平线夹角 (rad)
        theta1: 左小腿与水平线夹角 (rad)
        theta2: 右小腿与水平线夹角 (rad)
        l1, l2, l3, l4, l5: 腿部几何参数
    
    返回：
        J: 2x2 雅克比矩阵
    """
    try:
        J_sym, J_func = compute_jacobian_symbolic()
        J = J_func(l1, l2, l3, l4, l5, alpha, beta, theta1, theta2)
        return np.array(J, dtype=np.float64)
    except Exception as e:
        print(f"[雅克比计算] 符号计算失败，使用简化模型：{e}")
        # 简化雅克比矩阵（当符号计算失败时使用）
        J = np.array([
            [-l1 * np.sin(alpha) - l2 * np.sin(theta1), 0],
            [l1 * np.cos(alpha) + l2 * np.cos(theta1), 0]
        ])
        return J
def foot_force_to_joint_torque(foot_force, alpha, beta, theta1, theta2):
    """将足端力映射到关节扭矩
    
    使用公式：τ = J^T * F
    
    参数：
        foot_force: 足端力向量 [F_x, F_y] (N)
        alpha: 左大腿与水平线夹角 (rad)
        beta: 右大腿与水平线夹角 (rad)
        theta1: 左小腿与水平线夹角 (rad)
        theta2: 右小腿与水平线夹角 (rad)
    
    返回：
        joint_torque: 关节扭矩向量 [tau_alpha, tau_beta] (N·m)
    """
    J = compute_jacobian_numeric(alpha, beta, theta1, theta2)
    
    # τ = J^T * F
    foot_force_vec = np.array(foot_force).reshape(2, 1)
    joint_torque = J.T @ foot_force_vec
    
    return joint_torque.flatten()
# ===================== 新增：VMC 控制函数 =====================
def compute_vmc_foot_forces(data, model):
    """计算 VMC 足端力（高度环 + 平衡环 + 速度补偿 + 重力补偿）
    
    参数：
        data: MuJoCo 数据对象
        model: MuJoCo 模型对象
    
    返回：
        fz_left: 左足垂直力 (N)
        fz_right: 右足垂直力 (N)
        fy_left: 左足水平力 (N)
        fy_right: 右足水平力 (N)
    """
    global TARGET_HEIGHT, K_HEIGHT, D_HEIGHT, K_BALANCE, D_BALANCE, K_VEL_COMP_X, D_VEL_COMP_X, K_VEL_COMP_Y, D_VEL_COMP_Y, MASS_COMP, G_COMP
    
    # 获取车身高度（自由根节点 z 位置）
    body_height = data.qpos[2]
    
    # 获取车身高度速度
    body_height_vel = data.qvel[2]
    
    # 获取车身俯仰角和角速度
    base_quat = data.qpos[3:7]
    base_euler = quat2euler(base_quat)
    body_pitch = base_euler[0]
    body_pitch_vel = data.qvel[3]
    
    # 修改：坐标系 x 与 y 是反的，交换速度分量
    # 原代码：body_vel_x = data.qvel[0], body_vel_y = data.qvel[1]
    # 修改后：使用 qvel[1] 作为 x 方向速度，qvel[0] 作为 y 方向速度
    body_vel_x = data.qvel[1]  # 修改：原为 data.qvel[0]
    body_vel_y = data.qvel[0]  # 修改：原为 data.qvel[1]
    
    # 高度环控制：F_z = K_height * (h_ref - h) - D_height * h_dot + m*g (重力补偿)
    height_error = TARGET_HEIGHT - body_height
    gravity_comp = MASS_COMP * G_COMP  # 重力补偿项
    fz_total = K_HEIGHT * height_error - D_HEIGHT * body_height_vel + gravity_comp
    
    # 平衡环控制：产生俯仰力矩 M = K_balance * (-pitch) - D_balance * pitch_dot
    # 通过左右足力差产生力矩：M = (Fz_right - Fz_left) * l5 / 2
    balance_torque = -K_BALANCE * body_pitch - D_BALANCE * body_pitch_vel
    l5_half = L5 / 2.0
    if l5_half > 0:
        fz_diff = balance_torque / l5_half
    else:
        fz_diff = 0.0
    
    # 限制力差，防止单侧足端力为负（离开地面）
    # 修改：放宽限幅范围，允许更大的力差以应对大角度倾斜
    fz_diff = np.clip(fz_diff, -fz_total * 0.95, fz_total * 0.95)
    
    # 分配左右足垂直力
    fz_left = fz_total / 2.0 - fz_diff / 2.0
    fz_right = fz_total / 2.0 + fz_diff / 2.0
    
    # 确保足端力不为负（不能拉地面）
    fz_left = max(0.0, fz_left)
    fz_right = max(0.0, fz_right)
    
    # 速度补偿：分离 X 和 Y 方向控制
    # X 方向：用于前进/后退速度控制
    vel_error_x = TARGET_VELOCITY - body_vel_x
    fx_total = K_VEL_COMP_X * vel_error_x - D_VEL_COMP_X * body_vel_x
    
    # Y 方向：用于抑制侧向漂移（目标速度始终为 0）
    vel_error_y = 0.0 - body_vel_y
    fy_total_lateral = K_VEL_COMP_Y * vel_error_y - D_VEL_COMP_Y * body_vel_y
    
    # 修改：X 方向力平均分配到左右足（用于前进/后退）
    fx_left = fx_total / 2.0
    fx_right = fx_total / 2.0
    
    # 修改：Y 方向力平均分配到左右足（用于抑制侧向漂移）
    fy_left = fy_total_lateral / 2.0
    fy_right = fy_total_lateral / 2.0
    
    # ===================== 新增：详细调试日志 =====================
    # 修改：使用 VMC_VERBOSE 独立控制 VMC 日志输出
    if VMC_VERBOSE:
        print("\n" + "="*60)
        print("[VMC 调试] 足端力计算")
        print("="*60)
        print(f"[车身状态] 高度={body_height:.4f}m (目标={TARGET_HEIGHT:.4f}m), 误差={height_error:.4f}m")
        print(f"[车身状态] 高度速度={body_height_vel:.4f}m/s")
        print(f"[车身状态] 俯仰角={np.degrees(body_pitch):.2f}° (rad={body_pitch:.4f})")
        print(f"[车身状态] 俯仰角速度={body_pitch_vel:.4f}rad/s")
        print(f"[车身状态] 水平速度 vx={body_vel_x:.4f}m/s, vy={body_vel_y:.4f}m/s (目标={TARGET_VELOCITY:.4f}m/s)")
        print("-"*60)
        print(f"[高度环] K_height={K_HEIGHT}, D_height={D_HEIGHT}, 重力补偿={gravity_comp:.2f}N")
        print(f"[高度环] 高度项={K_HEIGHT * height_error:.2f}N, 阻尼项={-D_HEIGHT * body_height_vel:.2f}N")
        print(f"[高度环] 总垂直力 fz_total={fz_total:.2f}N")
        print("-"*60)
        print(f"[平衡环] K_balance={K_BALANCE}, D_balance={D_BALANCE}, l5_half={l5_half:.4f}m")
        print(f"[平衡环] 平衡力矩={balance_torque:.2f}N·m (pitch 项={-K_BALANCE * body_pitch:.2f}, damping 项={-D_BALANCE * body_pitch_vel:.2f})")
        print(f"[平衡环] 力差 fz_diff={fz_diff:.2f}N (限幅后)")
        print("-"*60)
        print(f"[速度补偿 X] K_vel_comp_x={K_VEL_COMP_X}, D_vel_comp_x={D_VEL_COMP_X}")
        print(f"[速度补偿 X] 速度误差 vx={vel_error_x:.4f}m/s, 补偿力 fx_total={fx_total:.2f}N")
        print(f"[速度补偿 Y] K_vel_comp_y={K_VEL_COMP_Y}, D_vel_comp_y={D_VEL_COMP_Y}")
        print(f"[速度补偿 Y] 速度误差 vy={vel_error_y:.4f}m/s, 补偿力 fy_total={fy_total_lateral:.2f}N")
        print("-"*60)
        print(f"[足端力输出] 左足:Fz={fz_left:.2f}N, Fx={fx_left:.2f}N, Fy={fy_left:.2f}N | 右足:Fz={fz_right:.2f}N, Fx={fx_right:.2f}N, Fy={fy_right:.2f}N")
        print(f"[力方向定义] +Fz=向上 (支撑力), +Fx=向前 (x 轴正方向), +Fy=侧向 (y 轴正方向)")
        print("="*60)
    
    # 修改：返回 Fx 和 Fy 分开，供关节扭矩计算使用
    return fz_left, fz_right, fx_left, fy_left, fx_right, fy_right


def compute_leg_joint_torques(data, model, fz_left, fz_right, fx_left, fy_left, fx_right, fy_right):
    """将足端力映射到腿部关节扭矩
    
    参数：
        data: MuJoCo 数据对象
        model: MuJoCo 模型对象
        fz_left: 左足垂直力 (N)
        fz_right: 右足垂直力 (N)
        fx_left: 左足 X 方向水平力 (N) - 前进/后退
        fy_left: 左足 Y 方向水平力 (N) - 侧向
        fx_right: 右足 X 方向水平力 (N) - 前进/后退
        fy_right: 右足 Y 方向水平力 (N) - 侧向
    
    返回：
        leg_torques: 腿部关节扭矩字典 {joint_name: torque}
    """
    leg_torques = {}
    
    # 获取腿部关节角度（简化处理，假设关节角度可从 qpos 获取）
    try:
        # 髋关节角度（4 个髋关节电机）
        joint_angles = []
        for joint_name in HIP_JOINTS:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id != -1:
                qpos_adr = model.jnt_qposadr[joint_id]
                # 修改：处理多自由度关节，只取第一个自由度
                if qpos_adr < len(data.qpos):
                    angle = data.qpos[qpos_adr]
                    # 修改：将角度规范化到 [-pi, pi] 范围，避免角度包裹问题
                    angle = np.arctan2(np.sin(angle), np.cos(angle))
                    joint_angles.append(angle)
                else:
                    joint_angles.append(0.0)
            else:
                # 修改：关节不存在时记录警告
                print(f"[警告] 未找到关节：{joint_name}")
                joint_angles.append(0.0)
        
        # 修改：分别计算每个关节的扭矩，而不是使用平均角度
        # 左后腿
        left_behind_angle = joint_angles[0]
        # 右后腿
        right_behind_angle = joint_angles[1]
        # 左前腿
        left_front_angle = joint_angles[2]
        # 右前腿
        right_front_angle = joint_angles[3]
        
        # 修改：修正扭矩计算公式，分离 X 和 Y 方向的力
        # 公式推导：
        # - Fz (向上为正): 产生 tau_z = -Fz * L * sin(theta) (支撑力使腿伸长)
        # - Fx (向前为正): 产生 tau_x = Fx * L * cos(theta) (前进力)
        # - Fy (侧向): 对髋关节扭矩影响较小，暂忽略
        
        # 修改：使用更大的力臂（L1+L2）来计算扭矩，增加扭矩输出
        effective_length = L1 + L2  # 使用大腿 + 小腿总长度作为力臂
        
        # 左后腿扭矩（修改：分离 Fx 和 Fy）
        left_behind_torque = fx_left * effective_length * np.cos(left_behind_angle) - fz_left * effective_length * np.sin(left_behind_angle)
        # 右后腿扭矩
        right_behind_torque = fx_right * effective_length * np.cos(right_behind_angle) - fz_right * effective_length * np.sin(right_behind_angle)
        # 左前腿扭矩
        left_front_torque = fx_left * effective_length * np.cos(left_front_angle) - fz_left * effective_length * np.sin(left_front_angle)
        # 右前腿扭矩
        right_front_torque = fx_right * effective_length * np.cos(right_front_angle) - fz_right * effective_length * np.sin(right_front_angle)
        
        # 分配到各髋关节
        leg_torques[HIP_JOINTS[0]] = left_behind_torque  # left_behind
        leg_torques[HIP_JOINTS[1]] = right_behind_torque  # right_behind
        leg_torques[HIP_JOINTS[2]] = left_front_torque  # left_front
        leg_torques[HIP_JOINTS[3]] = right_front_torque  # right_front
        
        # ===================== 修改：简化调试日志（仅在需要时输出）=====================
        # 修改：使用 VMC_VERBOSE 独立控制 VMC 日志输出
        if VMC_VERBOSE:
            print("\n" + "="*60)
            print("[VMC 调试] 关节扭矩计算")
            print("="*60)
            print(f"[关节角度] 左后={np.degrees(joint_angles[0]):.2f}°, 右后={np.degrees(joint_angles[1]):.2f}°, "
                  f"左前={np.degrees(joint_angles[2]):.2f}°, 右前={np.degrees(joint_angles[3]):.2f}°")
            print("-"*60)
            print(f"[腿部参数] L1={L1}m, L2={L2}m, 有效力臂={effective_length}m (大腿 + 小腿总长度)")
            print("-"*60)
            print(f"[足端力输入] 左足:Fx={fx_left:.2f}N, Fy={fy_left:.2f}N, Fz={fz_left:.2f}N | 右足:Fx={fx_right:.2f}N, Fy={fy_right:.2f}N, Fz={fz_right:.2f}N")
            print(f"[力方向定义] +Fz=向上 (支撑力), +Fx=向前 (x 轴正方向), +Fy=侧向 (y 轴正方向)")
            print("-"*60)
            print(f"[扭矩计算] 左后:tau = Fx*{effective_length}*cos({left_behind_angle:.4f}) - Fz*{effective_length}*sin({left_behind_angle:.4f})")
            print(f"[扭矩计算] 左后:tau = {fx_left:.2f}*{effective_length}*{np.cos(left_behind_angle):.4f} - {fz_left:.2f}*{effective_length}*{np.sin(left_behind_angle):.4f}")
            print(f"[扭矩计算] 左后:tau = {fx_left * effective_length * np.cos(left_behind_angle):.2f} - {fz_left * effective_length * np.sin(left_behind_angle):.2f} = {left_behind_torque:.2f}N·m")
            print(f"[扭矩计算] 右后:tau = {fx_right:.2f}*{effective_length}*{np.cos(right_behind_angle):.4f} - {fz_right:.2f}*{effective_length}*{np.sin(right_behind_angle):.4f}")
            print(f"[扭矩计算] 右后:tau = {fx_right * effective_length * np.cos(right_behind_angle):.2f} - {fz_right * effective_length * np.sin(right_behind_angle):.2f} = {right_behind_torque:.2f}N·m")
            print(f"[扭矩计算] 左前:tau = {fx_left:.2f}*{effective_length}*{np.cos(left_front_angle):.4f} - {fz_left:.2f}*{effective_length}*{np.sin(left_front_angle):.4f}")
            print(f"[扭矩计算] 左前:tau = {fx_left * effective_length * np.cos(left_front_angle):.2f} - {fz_left * effective_length * np.sin(left_front_angle):.2f} = {left_front_torque:.2f}N·m")
            print(f"[扭矩计算] 右前:tau = {fx_right:.2f}*{effective_length}*{np.cos(right_front_angle):.4f} - {fz_right:.2f}*{effective_length}*{np.sin(right_front_angle):.4f}")
            print(f"[扭矩计算] 右前:tau = {fx_right * effective_length * np.cos(right_front_angle):.2f} - {fz_right * effective_length * np.sin(right_front_angle):.2f} = {right_front_torque:.2f}N·m")
            print("-"*60)
            print(f"[关节扭矩输出] 左后={leg_torques[HIP_JOINTS[0]]:.2f}N·m, 右后={leg_torques[HIP_JOINTS[1]]:.2f}N·m")
            print(f"[关节扭矩输出] 左前={leg_torques[HIP_JOINTS[2]]:.2f}N·m, 右前={leg_torques[HIP_JOINTS[3]]:.2f}N·m")
            print(f"[扭矩方向定义] +tau=髋关节正向旋转 (根据 XML 定义)")
            print("="*60)
        
    except Exception as e:
        print(f"[VMC 关节扭矩] 计算失败：{e}，使用默认值")
        for joint in HIP_JOINTS:
            leg_torques[joint] = 0.0
    
    return leg_torques


def apply_leg_torques(data, model, leg_torques, verbose=False):
    """施加腿部关节扭矩到执行器
    
    参数：
        data: MuJoCo 数据对象
        model: MuJoCo 模型对象
        leg_torques: 腿部关节扭矩字典 {joint_name: torque}
        verbose: 是否输出详细日志（修改：新增参数，默认关闭）
    """
    # 腿部执行器名称映射（4 个髋关节电机）
    leg_actuator_map = {
        "left_behind_hip_joint": "left_behind_hip_motor",
        "right_behind_hip_joint": "right_behind_hip_motor",
        "left_front_hip_joint": "left_front_hip_motor",
        "right_front_hip_joint": "right_front_hip_motor",
    }
    # 新增：膝关节执行器名称映射
    knee_actuator_map = {
        "left_behind_knee_joint": "left_behind_knee_motor",
        "right_behind_knee_joint": "right_behind_knee_motor",
        "left_front_knee_joint": "left_front_knee_motor",
        "right_front_knee_joint": "right_front_knee_motor",
    }
    
    # ===================== 修改：条件化调试日志 =====================
    # 修改：使用 VMC_VERBOSE 独立控制 VMC 日志输出
    if verbose and VMC_VERBOSE:
        print("\n" + "="*60)
        print("[VMC 调试] 施加关节扭矩到执行器")
        print("="*60)
        print(f"[减速箱] 髋关节减速比={HIP_GEAR_RATIO}:1, 膝关节减速比={KNEE_GEAR_RATIO}:1")
    
    # 施加髋关节扭矩
    for joint_name, torque in leg_torques.items():
        actuator_name = leg_actuator_map.get(joint_name)
        if actuator_name:
            try:
                # 查找执行器 ID
                act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
                if act_id != -1:
                    # 修改：考虑减速比，电机扭矩 = 关节扭矩 / 减速比
                    motor_torque = torque / HIP_GEAR_RATIO
                    # 修改：使用更大的扭矩限幅，允许髋关节输出更大扭矩
                    torque_clamped = np.clip(motor_torque, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT)
                    data.actuator(actuator_name).ctrl[0] = torque_clamped
                    
                    # ===================== 修改：条件化日志输出 =====================
                    if verbose and VMC_VERBOSE:
                        clamped = torque_clamped != motor_torque
                        status = " [被限幅]" if clamped else ""
                        print(f"[执行器] {actuator_name}: 关节扭矩={torque:.2f}N·m -> 电机扭矩={motor_torque:.2f}N·m -> 限幅后={torque_clamped:.2f}N·m{status}")
            except Exception as e:
                if verbose and VMC_VERBOSE:
                    print(f"[VMC 施加扭矩] 执行器 {actuator_name} 失败：{e}")
    
    # 修改：施加膝关节限位扭矩（根据前后腿分别设置限位角度，更柔和的控制）
    if verbose and VMC_VERBOSE:
        print("-"*60)
        print("[膝关节限位控制] 后膝:-60°~30°, 前膝:-30°~60°, 柔和限位模式（15°阈值）")
    
    for i, joint_name in enumerate(KNEE_JOINTS):
        try:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id != -1:
                # 获取当前膝关节角度
                qpos_adr = model.jnt_qposadr[joint_id]
                qvel_adr = model.jnt_dofadr[joint_id]
                if qpos_adr < len(data.qpos) and qvel_adr < len(data.qvel):
                    knee_angle = data.qpos[qpos_adr]
                    knee_vel = data.qvel[qvel_adr]
                    
                    # 修改：根据前后腿设置不同的限位角度
                    is_rear_knee = (i < 2)  # 索引 0,1 为后腿，2,3 为前腿
                    
                    if is_rear_knee:
                        knee_min = KNEE_LIMIT_ANGLE_REAR_MIN
                        knee_max = KNEE_LIMIT_ANGLE_REAR_MAX
                    else:
                        knee_min = KNEE_LIMIT_ANGLE_FRONT_MIN
                        knee_max = KNEE_LIMIT_ANGLE_FRONT_MAX
                    
                    # 修改：计算距离限位的角度差，使用更柔和的增益
                    limit_torque = 0.0
                    activation_status = " [待机]"
                    
                    # 修改：使用平滑过渡函数，避免扭矩突变
                    # 检查是否超过最小限位
                    if knee_angle < knee_min + KNEE_LIMIT_THRESHOLD:
                        angle_to_limit = knee_angle - knee_min
                        # 修改：使用非线性增益，角度越接近限位增益越大
                        smooth_factor = min(1.0, (KNEE_LIMIT_THRESHOLD - abs(angle_to_limit)) / KNEE_LIMIT_THRESHOLD)
                        limit_torque = K_KNEE_LIMIT * angle_to_limit * smooth_factor + D_KNEE_LIMIT * knee_vel * smooth_factor
                        limit_torque = max(0.0, limit_torque)  # 只允许正向扭矩（阻止继续弯曲）
                        activation_status = f" [激活 - 下限 (因子={smooth_factor:.2f})]"
                    # 检查是否超过最大限位
                    elif knee_angle > knee_max - KNEE_LIMIT_THRESHOLD:
                        angle_to_limit = knee_angle - knee_max
                        # 修改：使用非线性增益，角度越接近限位增益越大
                        smooth_factor = min(1.0, (KNEE_LIMIT_THRESHOLD - abs(angle_to_limit)) / KNEE_LIMIT_THRESHOLD)
                        limit_torque = K_KNEE_LIMIT * angle_to_limit * smooth_factor + D_KNEE_LIMIT * knee_vel * smooth_factor
                        limit_torque = min(0.0, limit_torque)  # 只允许负向扭矩（阻止继续伸直）
                        activation_status = f" [激活 - 上限 (因子={smooth_factor:.2f})]"
                    
                    # 获取对应执行器
                    actuator_name = knee_actuator_map.get(joint_name)
                    if actuator_name:
                        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
                        if act_id != -1:
                            # 考虑减速比
                            motor_torque = limit_torque / KNEE_GEAR_RATIO
                            # 修改：降低膝关节扭矩限幅，使控制更柔和
                            torque_clamped = np.clip(motor_torque, -CTRL_LIMIT/4, CTRL_LIMIT/4)
                            data.actuator(actuator_name).ctrl[0] = torque_clamped
                            
                            if verbose and VMC_VERBOSE:
                                clamped = torque_clamped != motor_torque
                                status = " [被限幅]" if clamped else ""
                                print(f"[膝关节] {joint_name}: 角度={np.degrees(knee_angle):.2f}°, 限位=[{np.degrees(knee_min):.2f}°, {np.degrees(knee_max):.2f}°]{activation_status}")
                                print(f"  -> 限位扭矩={limit_torque:.2f}N·m -> 电机扭矩={motor_torque:.2f}N·m -> 限幅后={torque_clamped:.2f}N·m{status}")
        except Exception as e:
            if verbose and VMC_VERBOSE:
                print(f"[膝关节限位] 关节 {joint_name} 处理失败：{e}")
    
    if verbose and VMC_VERBOSE:
        print("="*60 + "\n")


# ===================== 3. LQR 核心函数 =====================
def compute_lqr_gain(A, B, Q, R, verbose=True):
    """求解连续时间 LQR 增益矩阵 K。

    参数：
        A: 状态矩阵 (n x n)
        B: 输入矩阵 (n x m)
        Q: 状态权重矩阵 (n x n)
        R: 控制权重矩阵 (m x m)
        verbose: 是否输出诊断信息
    
    返回：
        K: 增益矩阵 (m x n)
    """
    if verbose:
        print("\n=== LQR 矩阵诊断 ===")
        cond_A = np.linalg.cond(A)
        print(f"A 矩阵条件数：{cond_A if cond_A != np.inf else 'singular'}")
        print(f"B 矩阵条件数：{np.linalg.cond(B):.2e}")
        print(f"A 范数：[{np.min(np.abs(A)):.2e}, {np.max(np.abs(A)):.2e}]")
        print(f"B 范数：[{np.min(np.abs(B)):.2e}, {np.max(np.abs(B)):.2e}]")
        rank_A = np.linalg.matrix_rank(A, tol=1e-6)
        print(f"A 矩阵秩：{rank_A}/{A.shape[0]}")
        eig_vals = np.linalg.eigvals(A)
        print(f"A 特征值 (绝对值排序前 5): {np.sort(np.abs(eig_vals))[:5]}...\n")

    # 少量正则化以避免条件不良
    eps_reg = 1e-4
    A_reg = A + eps_reg * np.eye(A.shape[0])
    if verbose:
        print(f"使用正则化 eps={eps_reg}, 条件数 {np.linalg.cond(A_reg):.2e}")

    try:
        P = solve_continuous_are(A_reg, B, Q, R)
        if verbose:
            print("LQR 求解成功")
    except ValueError:
        if verbose:
            print("初次求解失败，尝试更大正则化")
        for eps_reg in [1e-3, 1e-2, 1e-1, 0.5, 1.0]:
            try:
                A_reg = A + eps_reg * np.eye(A.shape[0])
                P = solve_continuous_are(A_reg, B, Q, R)
                if verbose:
                    print(f"使用 eps={eps_reg} 成功")
                break
            except Exception:
                continue
        else:
            if verbose:
                print("所有正则化均失败，返回单位矩阵作为 P")
            P = np.eye(A.shape[0])

    return inv(R) @ B.T @ P


# ===================== 4. 终端参数设置 =====================
def get_terminal_input():
    """从终端获取用户输入的目标速度
    
    返回：
        velocity: 目标速度 (m/s)，None 表示退出
    """
    try:
        user_input = input("\n[终端控制] 输入目标速度 (m/s)，或输入 'q' 退出：").strip()
        if user_input.lower() == 'q':
            return None
        velocity = float(user_input)
        return velocity
    except ValueError:
        print("[终端控制] 输入无效，请输入数字或 'q'")
        return 0.0
    except EOFError:
        print("\n[终端控制] 检测到退出信号")
        return None


# ===================== 5. 主仿真循环 =====================
def main():
    """主函数：初始化仿真并运行 LQR 控制"""
    global HEADLESS_MODE, TARGET_HEIGHT
    
    # 检测运行环境
    if HEADLESS_MODE or os.environ.get("DISPLAY") is None:
        HEADLESS_MODE = True
        print("[运行模式] 无头模式 (Headless) - 不使用图形界面")
    else:
        print("[运行模式] 可视化模式 - 使用图形界面")
    
    reset_to_equilibrium(model, data)
    
    A, B = compute_theoretical_AB(data, model)
    if VERBOSE:
        print("理论 A,B 矩阵:")
        print(A)
        print(B)

    # 修改：LQR 权重矩阵配置 (更新为 query 中的配置)
    Q_small = np.diag([0, 15, 500, 800])  # 修改：增加姿态和角速度权重
    R_small = np.array([[10]])  # 修改：增加控制权重，减少控制输出
    K_small = compute_lqr_gain(A, B, Q_small, R_small, verbose=VERBOSE)
    if VERBOSE:
        print("K_small (4x1):\n", K_small)
    
    K = np.zeros((N_CONTROLS, N_STATES))
    K[0, :4] = K_small[0, :]
    K[1, :4] = K_small[0, :]
    if VERBOSE:
        print("扩展后 K (2x4):\n", K)
    
    # 获取初始目标速度
    initial_velocity = get_terminal_input()
    if initial_velocity is None:
        print("[终端控制] 用户选择退出")
        return
    set_target_velocity(initial_velocity)
    
    import time
    
    if HEADLESS_MODE:
        # 无头模式：使用 Renderer 渲染，不显示窗口
        print("[仿真] 无头模式启动，按 Ctrl+C 停止")
        renderer = mujoco.Renderer(model)
        
        step_count = 0
        reset_interval = 500
        vmc_log_interval = 100  # 新增：VMC 日志输出间隔
        
        try:
            while True:
                step_start = data.time

                x_current = get_state(data)
                x_error = x_current - X_REF

                if step_count % reset_interval == 0:
                    for j in WHEEL_JOINTS:
                        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
                        if joint_id != -1:
                            qpos_adr = int(model.jnt_qposadr[joint_id])
                            data.qpos[qpos_adr] = 0.0
                
                # 计算平衡扭矩 u1 (LQR 控制)
                u_balance = -K @ x_error
                
                # 计算驱动扭矩 u2 (速度/位置控制)
                u_drive = compute_drive_torque(data, model, x_current[0], x_current[1])
                
                # VMC 控制 - 计算足端力 (保留)
                fz_left, fz_right, fx_left, fy_left, fx_right, fy_right = compute_vmc_foot_forces(data, model)
                
                # VMC 控制 - 将足端力映射到腿部关节扭矩 (保留)
                leg_torques = compute_leg_joint_torques(data, model, fz_left, fz_right, fx_left, fy_left, fx_right, fy_right)
                
                # VMC 控制 - 施加腿部关节扭矩 (保留，修改：关闭调试日志)
                apply_leg_torques(data, model, leg_torques, verbose=False)  # 修改：关闭 VMC 调试日志
                
                # 总扭矩 = 平衡扭矩 + 驱动扭矩
                u = u_balance + u_drive
                
                u = np.clip(u, -CTRL_LIMIT, CTRL_LIMIT)
                
                set_actuator_ctrl(data, u, fix_hips=True)

                mujoco.mj_step(model, data)
                
                # 定期渲染（可选，用于生成视频或调试）
                if step_count % 100 == 0 and VERBOSE:
                    renderer.update_scene(data)
                
                step_count += 1
                
                if step_count % 500 == 0 and VERBOSE:
                    print(f"step {step_count}: "
                          f"wheel_pos={x_current[0]:.4f}, "
                          f"wheel_vel={x_current[1]:.4f}, "
                          f"body_pitch={x_current[2]:.4f}, "
                          f"body_pitch_vel={x_current[3]:.4f}, "
                          f"u_max={np.max(np.abs(u)):.2f}, "
                          f"target_vel={TARGET_VELOCITY:.2f}, "
                          f"target_height={TARGET_HEIGHT:.2f}")

        except KeyboardInterrupt:
            print("\n[仿真] 用户中断，停止仿真")
        finally:
            renderer.close()
    
    else:
        # 可视化模式：使用 viewer
        with viewer.launch_passive(model, data) as v:
            if VERBOSE:
                print("仿真启动，从平衡位置开始 LQR 控制")
            
            step_count = 0
            reset_interval = 500
            vmc_log_interval = 100  # 新增：VMC 日志输出间隔
            
            while v.is_running():
                step_start = data.time

                x_current = get_state(data)
                x_error = x_current - X_REF
                
                if step_count % reset_interval == 0:
                    for j in WHEEL_JOINTS:
                        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
                        if joint_id != -1:
                            qpos_adr = int(model.jnt_qposadr[joint_id])
                            data.qpos[qpos_adr] = 0.0
                
                # 计算平衡扭矩 u1 (LQR 控制)
                u_balance = -K @ x_error
                
                # 计算驱动扭矩 u2 (速度/位置控制)
                u_drive = compute_drive_torque(data, model, x_current[0], x_current[1])
                
                # VMC 控制 - 每步都执行，日志按间隔输出 (保留，修改：关闭调试日志)
                fz_left, fz_right, fx_left, fy_left, fx_right, fy_right = compute_vmc_foot_forces(data, model)
                leg_torques = compute_leg_joint_torques(data, model, fz_left, fz_right, fx_left, fy_left, fx_right, fy_right)
                apply_leg_torques(data, model, leg_torques, verbose=False)  # 修改：关闭 VMC 调试日志
                
                # 总扭矩 = 平衡扭矩 + 驱动扭矩
                u = u_balance + u_drive
                
                u = np.clip(u, -CTRL_LIMIT, CTRL_LIMIT)
                
                set_actuator_ctrl(data, u, fix_hips=True)

                mujoco.mj_step(model, data)
                v.sync()

                step_count += 1
                
                if step_count % 500 == 0 and VERBOSE:
                    print(f"step {step_count}: "
                          f"wheel_pos={x_current[0]:.4f}, "
                          f"wheel_vel={x_current[1]:.4f}, "
                          f"body_pitch={x_current[2]:.4f}, "
                          f"body_pitch_vel={x_current[3]:.4f}, "
                          f"u_max={np.max(np.abs(u)):.2f}, "
                          f"target_vel={TARGET_VELOCITY:.2f}, "
                          f"target_height={TARGET_HEIGHT:.2f}")

                dt = model.opt.timestep

if __name__ == "__main__":
    main()
