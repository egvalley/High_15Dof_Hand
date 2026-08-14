"""
控制命令码 —— 上位机能下发的功能、各自的定点缩放、以及状态机状态码。

对应固件 router_app_motor_cmd.h 的 RouterRxControlMode 与 motor_state_machine.h 的 MotorStateType。
"""

from enum import IntEnum

from sdk.protocol.wire import RxFrame


# 电机二的 mode = 电机一的 mode + 300 (固件里 202/502、232/532、…、323/623 逐条如此)。
# 所以功能表只列电机一的码，电机二用这个步长换算。
MOTOR_MODE_STRIDE = 300


class MotorFunc(IntEnum):
    """
    Control_Mode 功能码 —— 取值为【电机一】的 RouterRxControlMode，电机二 = 本值 + 300。

    每个成员携带 (电机一功能码, scale)：
      scale = 把浮点物理量转成下发 int16 的乘数，即 param = round(物理量 × scale)，
              逐条取自固件 Dispatch 里的 CMD_SCALE_* 换算。
      物理量一律取【输出轴】单位。
    """

    def __new__(cls, code, scale):
        """让每个成员既是电机一的功能码 int(func)，又携带量化乘数 .scale。"""
        obj = int.__new__(cls, code)
        obj._value_ = code
        obj.scale = scale
        return obj

    #                            M1 code  scale     # 固件换算 (param -> 物理量)
    # —— 基本控制 (输出轴单位) ——
    THETA_GEAR               = (   206,  100.0)   # θ = param/100
    OMEGA_GEAR               = (   207,  100.0)   # ω = param/100
    TORQUE_GEAR              = (   208,    1.0)   # τ(mN·m) = param
    IQ                       = (   209, 1000.0)   # iq = param·0.001
    ID                       = (   210, 1000.0)   # id = param·0.001
    # —— 阻抗 ——
    IMPEDANCE_SPRING         = (   232,  100.0)   # k = param/100
    IMPEDANCE_DAMPER         = (   233, 1000.0)   # d = param/1000
    IMPEDANCE_INERTIA        = (   234,10000.0)   # j = param/10000
    IMPEDANCE_SPRING_ORIGIN  = (   235,  100.0)   # 原点 = param/100
    # —— 各环 PID ——
    CUR_PID_KP               = (   252, 1000.0)   # param·0.001
    CUR_PID_KI               = (   253, 1000.0)
    VEL_PID_KP               = (   254, 1000.0)
    VEL_PID_KI               = (   255, 1000.0)
    POS_PID_KP               = (   256,    1.0)   # param (无缩放)
    POS_PID_KI               = (   257,    1.0)
    # —— 错误清除 (无参数，param 被固件忽略)。逐条对应错误字的一段位域，见 errors.ErrorSeg ——
    CLEAR_INNER_ERROR        = (   270,    1.0)   # 清内环错误   (错误字 bit0 ~bit7)
    CLEAR_OUTER_ERROR        = (   271,    1.0)   # 清外环错误   (错误字 bit8 ~bit15)
    # —— 状态机 ——
    DISPATCH_STATE           = (   272,    1.0)   # 状态机派发；param = 状态码 (见 MotorState)
    # —— 轨迹 (输出轴单位) ——
    TRAJ_INIT                = (   282,    1.0)   # 无参数
    TRAJ_DEINIT              = (   283,    1.0)   # 无参数
    TRAJ_VEL_MAX             = (   284,  100.0)
    TRAJ_ACL_MAX             = (   285,  100.0)
    TRAJ_POS_CMD             = (   286,  100.0)
    # —— 回零 (Homing，输出轴单位) ——
    HOMING_INIT              = (   301,    1.0)   # 触发回零 (无参数)
    HOMING_FORWARD_TORQUE    = (   302,    1.0)   # 前向力矩(mN·m) = param
    HOMING_BACKWARD_TORQUE   = (   303,    1.0)   # 反向力矩(mN·m) = param
    HOMING_FORWARD_POSITION  = (   304,  100.0)   # 前向位置 = param/100
    HOMING_BACKWARD_POSITION = (   305,  100.0)   # 反向位置 = param/100
    # —— 系统辨识 ——
    SYS_IDEN                 = (   311,    1.0)   # 触发系统辨识 (无参数)；上位机暂未开放按钮
    # —— 设备级 ——
    FLASHING_PARAMS          = (   321,    1.0)   # 按配置序号重置控制参数为预设并刷 Flash
    CLEAR_FLASH_ERROR        = (   322,    1.0)   # 清 Flash 错误；Flash 是共享设备，M1/M2 两码同效
    CLEAR_ENCODER_ERROR      = (   323,    1.0)   # 清编码器错误 (错误字 bit16~bit17)；各清自己那路 SPI

    def code_for(self, motor=0):
        """本功能对【第 motor 个电机】(0/1) 的 Control_Mode 值：电机0 = 本值，电机1 = 本值+300。"""
        if motor not in (0, 1):
            raise ValueError(f"无效的电机序号: {motor!r} (应为 0 或 1)")
        return int(self) + MOTOR_MODE_STRIDE * motor

    @classmethod
    def code_of(cls, func, motor=0):
        """(功能, 电机序号) -> Control_Mode 值。func 可为 MotorFunc 成员或其电机一功能码。"""
        return cls(func).code_for(motor)

    @classmethod
    def scale_of(cls, func):
        """浮点物理量 -> int16 定点数 的缩放乘数 (param = round(物理量 × scale))。"""
        return cls(func).scale


# 帧尾冲突自检：功能码低字节撞上帧尾会让接收侧提前切帧。应恒为空元组
TAIL_CONFLICT_MODES = tuple(
    code
    for func in MotorFunc
    for code in (func.code_for(0), func.code_for(1))
    if (code & 0xFF) == RxFrame.TAIL
)


# ==================================================================== 电机状态机
class MotorState:
    """
    电机状态机状态码 —— 逐条对应固件 MotorStateType。

    ★ 只剩【下发】这一个用途：作为 DISPATCH_STATE 命令的参数。
      新协议的反馈里第 0 条曲线已由"状态码"换成 uint32 错误字 (见 errors)，
      上位机不再能从反馈读到当前状态。150~165 这几个错误态码保留是为了还能显式派发，
      日常判错请看错误字。
    """

    CODES = {
        # —— 启动 ——
        "StartupReady": 3,
        "StartupCurrentCalib": 4, "StartupPhaseDiag": 5, "StartupElecAngleDrag": 6,
        "StartupElecAngleDone": 7, "StartupDisable": 20,
        # —— 电流环调试 ——
        "DbgCurrentAlphaBeta": 22, "DbgCurrentOpenLoop": 23,
        "DbgCurrentOnlyIdClosedLoop": 24, "DbgCurrentClosedLoop": 25,
        "DbgCurrentClosedLoop_IqSysIden": 26, "DbgCurrentClosedLoop_IdSysIden": 27,
        "DbgCurrentOpenLoop_UqSysIden": 28, "DbgCurrentOpenLoop_UdSysIden": 29,
        "DbgCurrentDisable": 40,
        # —— 速度环 / 位置环调试 ——
        "DbgVelocityClosedLoop_SysIden": 42, "DbgVelocityDisable": 60,
        "DbgPositionClosedLoop_SysIden": 62, "DbgPositionDisable": 80,
        # —— 应用 ——
        "AppCurrentCtrl": 82, "AppTorqueCtrl": 83, "AppImpedanceCtrl": 84,
        "AppVelocityCtrl": 85, "AppPositionCtrl": 86, "AppDisable": 100,
        # —— 错误态 (内/外环，150+) ——
        "InnerEncoderReadError": 150, "InnerCurrentReadError": 151,
        "InnerParamLoadError": 152, "OuterParamLoadError": 153,
        "OuterThetaeCalibSaveError": 154, "OuterCurrentCalibSaveError": 155,
        "InnerOuterMismatchError": 165,
    }
    NAMES = {v: k for k, v in CODES.items()}

    @classmethod
    def code_of(cls, state):
        """状态名或状态码 -> 状态码；未知状态名抛 ValueError。"""
        if isinstance(state, str):
            if state not in cls.CODES:
                raise ValueError(f"未知状态名: {state}")
            return cls.CODES[state]
        return int(state)

    @classmethod
    def name(cls, code):
        """状态码 -> 状态名 (未知码返回 Unknown(code)，None 原样返回)。"""
        if code is None:
            return None
        return cls.NAMES.get(code, f"Unknown({code})")
