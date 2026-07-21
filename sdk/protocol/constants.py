"""
协议常量唯一定义源 (Single Source of Truth)。

★ 本版对照 router.h (RouterControlMode / RouterMCUObject 枚举) + router.c
  (DeviceRouter_RxPayloadHandler 的换算) 校正。

电机区分方式 (见 router.c 的两段 for 循环)：一帧的命令列表按【前/后半段位置】分给两个电机——
    前半段 i <  command_num/2  -> 电机0 (固件 M1)
    后半段 i >= command_num/2  -> 电机1 (固件 M2)
  两段 switch 完全相同、共用【同一套 mode 值】；因此 mode 只表示"功能"，与电机无关。
  ⇒ 编码时两半必须等长 (短的一侧用 NOP_MODE 补齐)，否则切分点错位、电机路由出错。

Control_Mode 取值 = router.h RouterControlMode 枚举 (见 MotorFunc)，固件已实现：
  位置 / 速度 / 力矩 · Iq / Id · 阻抗(弹簧/阻尼/惯量/原点) · 各环 PID ·
  状态派发 · 轨迹(init/deinit/vmax/amax/pos) · 回零(init/前向力矩/反向位置) · 刷参 · 清Flash。
"""

from enum import IntEnum


# ==================================================================== Tx 反馈帧
class TxFrame:
    """MCU -> 上位机 反馈帧 (对应 router.c TX_* 常量)。"""

    # 帧结构
    HEADER            = 0x5A
    TAIL              = 0x0D
    FIXED_HEADER_SIZE = 10
    TAIL_SIZE         = 3
    MAX_CURVE_COUNT   = 64
    MAX_FRAME_SIZE    = 1024      # = router.h ROUTER_MAX_TX_FRAME_SIZE

    # timestamp 换算成秒时的基准频率 (按 func 选内环/外环)
    INNER_FREQ        = 10000.0
    OUTER_FREQ        = 200.0


# ==================================================================== Rx 命令帧
class RxFrame:
    """上位机 -> MCU 命令帧 (对应 router.c RX_* 常量)。"""

    # 帧结构
    HEADER            = 0x3B
    TAIL              = 0x1E
    FIXED_HEADER_SIZE = 5
    TAIL_SIZE         = 1
    MAX_COMMANDS      = 20        # router.c: command_num > 20 直接丢弃

    CANFD_VALID_SIZES = (8, 12, 16, 20, 24, 32, 48, 64)


# ==================================================================== 通信功能码
class CommFunc(IntEnum):
    """router.h RouterTxCommFunc / RouterRxCommFunc。"""

    # 发送 (MCU -> 上位机)
    TX_NORMAL            = 0     # Router_Comm_USB_USART_Tx_Normal
    TX_HIGH              = 1     # Router_Comm_USB_USART_Tx_High
    FDCAN_TX_NORMAL      = 2     # Router_Comm_FDCAN_Tx_Normal

    # 接收 (上位机 -> MCU)
    RX_USB_USART_COMMAND = 3     # Router_Comm_USB_USART_Cmd
    RX_FDCAN_COMMAND     = 4     # Router_Comm_FDCAN_Cmd
    RX_FDCAN_OTA         = 5     # Router_Comm_FDCAN_OTA


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
        """
        把 MCU 标识归一化成 0~COUNT-1 的 index。
        入参既可是 index (0~7) 也可是硬件 ID (0xB0~0xB7)；非法值抛 ValueError。
        全项目凡"既接受 index 又接受 ID"的地方都走这里。
        """
        if 0 <= mcu < cls.COUNT:
            return mcu
        if cls.BASE_ID <= mcu < cls.BASE_ID + cls.COUNT:
            return mcu - cls.BASE_ID
        raise ValueError(f"无效的 MCU 标识: {mcu!r} (应为 index 0~7 或 ID 0xB0~0xB7)")

    @classmethod
    def id_of(cls, mcu):
        """由 index 或 ID 得到硬件 ID (0xB0~0xB7)，供组帧时写入目标 ID 字段。"""
        return cls.IDS[cls.to_index(mcu)]


# ==================================================================== 控制功能
class MotorFunc(IntEnum):
    """
    Control_Mode 功能码 —— 逐值对应 router.h 的 RouterControlMode。

    两个电机共用同一套功能码，电机由命令的前/后半段位置区分 (见模块顶部说明)，
    所以 mode 与电机无关。每个成员携带 (功能码, scale)：
      scale = 把浮点物理量转成下发 int16 的乘数，即 param = round(物理量 × scale)。
      物理量一律取【输出轴】单位——齿轮比 GEAR 在固件换算里恰好抵消，见各行注释。
    """

    def __new__(cls, code, scale):
        """让每个成员既是它的功能码 int(func)，又携带量化乘数 .scale (见类文档)。"""
        obj = int.__new__(cls, code)
        obj._value_ = code
        obj.scale = scale
        return obj

    #                          code    scale    # router.c handler (param -> 物理量)
    # —— 基本控制 (输出轴单位) ——
    THETA_GEAR              = (   6,  100.0)   # 位置 θ_m = param·GEAR/100  ⇒ θ_out=param/100
    OMEGA_GEAR              = (   7,  100.0)   # 速度 ω_m = param·GEAR/100  ⇒ ω_out=param/100
    TORQUE_GEAR             = (   8,    1.0)   # 力矩(mN·m) τ_m=param/GEAR ⇒ τ_out=param，param=τ_out(mN·m)
    IQ                      = (   9, 1000.0)   # q 轴电流 iq = param·0.001
    ID                      = (  10, 1000.0)   # d 轴电流 id = param·0.001
    # —— 阻抗 ——
    IMPEDANCE_SPRING        = (  32,   10.0)   # k = param/10
    IMPEDANCE_DAMPER        = (  33,  100.0)   # d = param/100
    IMPEDANCE_INERTIA       = (  34, 1000.0)   # j = param/1000
    IMPEDANCE_SPRING_ORIGIN = (  35,  100.0)   # 原点 θ_m = param·GEAR/100 ⇒ 输出轴 param/100
    # —— 各环 PID ——
    CUR_PID_KP              = (  52, 1000.0)   # param·0.001
    CUR_PID_KI              = (  53, 1000.0)
    VEL_PID_KP              = (  54, 1000.0)
    VEL_PID_KI              = (  55, 1000.0)
    POS_PID_KP              = (  56,    1.0)   # param (无缩放)
    POS_PID_KI              = (  57,    1.0)
    # —— 状态机 ——
    DISPATCH_STATE          = (  72,    1.0)   # 状态机派发；param = 状态码 (见 MotorState)
    # —— 轨迹 (输出轴单位) ——
    TRAJ_INIT               = (  82,    1.0)   # 无参数
    TRAJ_DEINIT             = (  83,    1.0)   # 无参数
    TRAJ_VEL_MAX            = (  84,  100.0)   # param·GEAR/100 ⇒ 输出轴 param/100
    TRAJ_ACL_MAX            = (  85,  100.0)
    TRAJ_POS_CMD            = (  86,  100.0)
    # —— 回零 (Homing，输出轴单位) ——
    HOMING_INIT              = ( 101,    1.0)   # 触发回零 (无参数)
    HOMING_FORWARD_TORQUE    = ( 102,    1.0)   # 前向力矩(mN·m) 换算同 TORQUE_GEAR，param=τ_out(mN·m)
    HOMING_BACKWARD_POSITION = ( 103,  100.0)   # 反向位置 θ_m = param·GEAR/100 ⇒ 输出轴 param/100
    # —— 设备级 ——
    FLASHING_PARAMS          = ( 121,    1.0)   # 按配置序号重置控制参数为预设并刷入 Flash；param = config_index (见 ResetControlParams，0/1)
    CLEAR_FLASH_ERROR        = ( 122,    1.0)   # 清除 Flash 错误 (无参数)

    @classmethod
    def scale_of(cls, func):
        """浮点物理量 -> int16 定点数 的缩放乘数 (param = round(物理量 × scale))。"""
        return cls(func).scale


# 电机路由补齐用的空命令：固件 switch 无此 case -> 落 default 被安全忽略。
# 用于给"未寻址的那个电机"占位，保证命令列表前后两半等长 (见模块顶部说明)。
NOP_MODE = 0


# ==================================================================== 电机状态机
class MotorState:
    """
    电机状态机 —— 逐条对应固件 MotorStateType 枚举 (名称/数值已核对一致)。
    用于：① 解析 Tx 反馈里的 state 曲线；② 作为 DISPATCH_STATE 命令的参数(状态码)。
    反馈解析与命令派发共用同一套：固件 DispatchStateMachine 收的即此枚举。
    """

    CODES = {
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
    NAMES = {v: k for k, v in CODES.items()}

    @classmethod
    def name(cls, code):
        """状态码 -> 状态名 (未知码返回 Unknown(code)，None 原样返回)。"""
        if code is None:
            return None
        return cls.NAMES.get(code, f"Unknown({code})")
