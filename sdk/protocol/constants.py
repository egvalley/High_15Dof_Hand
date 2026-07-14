"""
协议常量唯一定义源 (Single Source of Truth)。

★ 本版已对照 router.h (枚举) + router.c (handler 换算) 校正。
  旧版基于臆测的 FOC/PID/轨迹 mode 号与缩放系数全部作废，改用固件真值。

固件对电机的区分方式已确认：
  靠 Control_Mode 里的 M0/M1/... 后缀区分，而【不是】靠命令前/后半段位置。
  (router.c 的两段循环体完全相同，其并集恰好覆盖 [0, command_num)，
   因此 command_num 奇偶都能被正确派发。)

本固件 handler 实际实现的功能集 (见 MotorFunc)：
  位置 / 速度 / 力矩(实为电流环) / 阻抗弹簧 / 阻抗阻尼 / 阻抗惯量 / 换ID / 状态派发
其余 (Iq、各环PID、轨迹、回零、刷参、系统辨识、清Flash、弹簧原点) 该固件未实现。
"""

from enum import IntEnum


# ==================================================================== Tx 反馈帧
class TxFrame:
    """MCU -> 上位机 反馈帧 (对应 router.c TX_* 常量)。"""

    HEADER = 0x5A
    TAIL = 0x0D
    FIXED_HEADER_SIZE = 10
    TAIL_SIZE = 3
    MAX_CURVE_COUNT = 64
    MAX_FRAME_SIZE = 1024   # = router.h ROUTER_MAX_TX_FRAME_SIZE

    INNER_FREQ = 10000.0
    OUTER_FREQ = 1000.0


# ==================================================================== Rx 命令帧
class RxFrame:
    """上位机 -> MCU 命令帧 (对应 router.c RX_* 常量)。"""

    HEADER = 0x3B
    TAIL = 0x1E
    FIXED_HEADER_SIZE = 5
    TAIL_SIZE = 1
    MAX_COMMANDS = 20        # router.c: command_num > 20 直接丢弃

    CANFD_VALID_SIZES = (8, 12, 16, 20, 24, 32, 48, 64)


# ==================================================================== 通信功能码
class CommFunc(IntEnum):
    """router.h RouterTxCommFunc / RouterRxCommFunc。"""

    TX_NORMAL = 0            # Router_Comm_USB_USART_Tx_Normal
    TX_HIGH = 1             # Router_Comm_USB_USART_Tx_High
    FDCAN_TX_NORMAL = 2     # Router_Comm_FDCAN_Tx_Normal

    RX_USB_USART_COMMAND = 3  # Router_Comm_USB_USART_Cmd
    RX_FDCAN_COMMAND = 4      # Router_Comm_FDCAN_Cmd
    RX_FDCAN_OTA = 5          # Router_Comm_FDCAN_OTA


# ==================================================================== MCU 拓扑
class McuConfig:
    """
    机械手 MCU 硬件拓扑。

    ⚠ 待确认：router.h 的 RouterMCUObject 只显式定义到 MCU5 (0xB5)，
      但 router.c 的告警文案写“支持 0xB0~0xB7”。此处沿用 router.c 的 8 个，
      与旧 GUI 一致。若实际只有 6 个 MCU，把 COUNT 改成 6 即可 (其余层不动)，
      并建议在 router.h 里补齐 MCU6/MCU7 枚举以保持一致。
    """

    COUNT = 8
    BASE_ID = 0xB0
    IDS = tuple(range(BASE_ID, BASE_ID + COUNT))   # 0xB0 ~ 0xB7

    MOTORS_PER_MCU = 2
    CURVES_PER_MOTOR = 5
    LOW_SPEED_CURVE_COUNT = MOTORS_PER_MCU * CURVES_PER_MOTOR   # = 10

    MOTOR_CURVE_FIELDS = ("state", "theta", "omega", "acl", "torque")

    @classmethod
    def to_index(cls, mcu):
        if 0 <= mcu < cls.COUNT:
            return mcu
        if cls.BASE_ID <= mcu < cls.BASE_ID + cls.COUNT:
            return mcu - cls.BASE_ID
        raise ValueError(f"无效的 MCU 标识: {mcu!r} (应为 index 0~7 或 ID 0xB0~0xB7)")

    @classmethod
    def id_of(cls, mcu):
        return cls.IDS[cls.to_index(mcu)]


# ==================================================================== 控制功能
class MotorFunc(IntEnum):
    """
    逻辑控制功能 (与具体电机无关)。
    真正下发的 mode 号由 mode_for(func, motor) 结合电机号查表得到。
    """

    POSITION = 0
    VELOCITY = 1
    TORQUE = 2                 # 注意：固件里实际调用电流环 ServiceCurrentCmd
    IMPEDANCE_SPRING = 3
    IMPEDANCE_DAMPER = 4
    IMPEDANCE_INERTIA = 5
    SWITCH_ID = 6
    MODE_SELECT = 7            # 状态机派发 (ServiceStateCmd)


# router.h RouterControlMode 逐字节抄写：
#   每行 = 某电机的 (Position, Velocity, Torque, Spring, Damper, Inertia, SwitchID, ModeSelect)
_CONTROL_MODE_TABLE = {
    0: (1,   2,   3,   4,   5,   6,   7,   20),
    1: (11,  12,  13,  14,  15,  16,  17,  40),
    2: (41,  42,  43,  44,  45,  46,  47,  60),
    3: (61,  62,  63,  64,  65,  66,  67,  80),
    4: (81,  82,  83,  84,  85,  86,  87,  100),
    5: (101, 102, 103, 104, 105, 106, 107, 120),
    6: (121, 122, 123, 124, 125, 126, 127, 140),
}
_FUNC_ORDER = (
    MotorFunc.POSITION, MotorFunc.VELOCITY, MotorFunc.TORQUE,
    MotorFunc.IMPEDANCE_SPRING, MotorFunc.IMPEDANCE_DAMPER, MotorFunc.IMPEDANCE_INERTIA,
    MotorFunc.SWITCH_ID, MotorFunc.MODE_SELECT,
)

# 展开成 {(func, motor): mode_code}
CONTROL_MODE = {
    (func, motor): _CONTROL_MODE_TABLE[motor][pos]
    for motor, row in _CONTROL_MODE_TABLE.items()
    for pos, func in enumerate(_FUNC_ORDER)
}

NOP = 0   # 固件无此 case，落 default:break，可用作无害占位


def mode_for(func, motor):
    """(逻辑功能, 电机号 0~6) -> 固件 Control_Mode 数值。"""
    try:
        return CONTROL_MODE[(MotorFunc(func), int(motor))]
    except KeyError:
        raise ValueError(f"无对应 Control_Mode: func={func!r}, motor={motor!r}")


# 浮点物理量 -> int16 定点数 的缩放系数 (取自 router.c handler 里的除数)。
#   Position  : param/100   -> ×100
#   Velocity  : param/1000  -> ×1000
#   Torque    : param/1     -> ×1     (实为电流指令, 单位约定见固件)
#   Impedance : param/100   -> ×100   (弹簧/阻尼/惯量三者相同)
#   SwitchID  : 直接 uint16  -> ×1
#   ModeSelect: 原始状态码    -> ×1
FUNC_SCALE = {
    MotorFunc.POSITION: 100.0,
    MotorFunc.VELOCITY: 1000.0,
    MotorFunc.TORQUE: 1.0,
    MotorFunc.IMPEDANCE_SPRING: 100.0,
    MotorFunc.IMPEDANCE_DAMPER: 100.0,
    MotorFunc.IMPEDANCE_INERTIA: 100.0,
    MotorFunc.SWITCH_ID: 1.0,
    MotorFunc.MODE_SELECT: 1.0,
}


def scale_of(func):
    return FUNC_SCALE[MotorFunc(func)]


# ==================================================================== 电机状态机
# 用于：① 解析 Tx 反馈里的 state 曲线；② 作为 MODE_SELECT 命令的参数(状态码)。
# ⚠ 待确认：ServiceStateCmd 接收的状态码是否与下面反馈解析用的枚举完全同一套。
MOTOR_STATE = {
    "StartupError": 1, "StartupReady": 2, "StartupParamsInit": 3,
    "StartupCurrentCalib": 4, "StartupPhaseDiag": 5, "StartupElecAngleDrag": 6,
    "StartupElecAngleDone": 7, "StartupDisable": 20,
    "DbgCurrentError": 21, "DbgCurrentAlphaBeta": 22, "DbgCurrentOpenLoop": 23,
    "DbgCurrentOnlyIdClosedLoop": 24, "DbgCurrentClosedLoop": 25,
    "DbgCurrentClosedLoop_IqSysIden": 26, "DbgCurrentClosedLoop_IdSysIden": 27,
    "DbgCurrentOpenLoop_UqSysIden": 28, "DbgCurrentOpenLoop_UdSysIden": 29,
    "DbgCurrentDisable": 40,
    "DbgVelocityError": 41, "DbgVelocityClosedLoop_SysIden": 42, "DbgVelocityDisable": 60,
    "DbgPositionError": 61, "DbgPositionClosedLoop_SysIden": 62, "DbgPositionDisable": 80,
    "AppError": 81, "AppCurrentCtrl": 82, "AppTorqueCtrl": 83, "AppImpedanceCtrl": 84,
    "AppVelocityCtrl": 85, "AppPositionCtrl": 86, "AppDisable": 100,
    "InnerOuterMismatch": 255,
}
MOTOR_STATE_NAME = {v: k for k, v in MOTOR_STATE.items()}


def state_name(code):
    if code is None:
        return None
    return MOTOR_STATE_NAME.get(code, f"Unknown({code})")
