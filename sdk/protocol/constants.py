"""
协议常量唯一定义源 (Single Source of Truth)。
"""

from enum import IntEnum


# ==================================================================== Tx 反馈帧
class TxFrame:
    """
    MCU -> 上位机 反馈帧 (对应 router_tran_handle.c 的 TX_FRAME_* 常量)。

    [0]        帧头 0xCC
    [1]        ID (发送源 = MCU 节点号)
    [2]        Func (RouterTxFuncCode)
    [3..6]     帧计数器 uint32 小端
    [7..]      curve_cnt 个 float32 小端 (按注册索引顺序)
    [tail]     帧尾 0xDD
    """

    HEADER          = 0xCC
    TAIL            = 0xDD
    HEADER_SIZE     = 7            # 帧头(1) + ID(1) + Func(1) + 帧计数器(4)
    TAIL_SIZE       = 1            # 帧尾(1)；无软件 CRC，链路层自带校验
    CURVE_SIZE      = 4            # 每条曲线一个 float32
    MAX_CURVE_COUNT = 14           # ROUTER_TX_MAX_CURVE_CNT
    MAX_FRAME_SIZE  = HEADER_SIZE + MAX_CURVE_COUNT * CURVE_SIZE + TAIL_SIZE   # = 64
    COUNTER_MASK    = 0xFFFFFFFF

    @classmethod
    def frame_size(cls, curve_count):
        """给定曲线数量算整帧长度 (新协议帧里没有长度字段，长度由曲线数量唯一确定)。"""
        return cls.HEADER_SIZE + curve_count * cls.CURVE_SIZE + cls.TAIL_SIZE


# ==================================================================== Rx 命令帧
class RxFrame:
    """
    上位机 -> MCU 命令帧 (对应 router_rec_handle.c 的 RX_FRAME_* 常量)。

    [0]        帧头 0xAA
    [1]        ID (目标 MCU 节点号)
    [2]        Func (RouterRxFuncCode)
    [3..6]     同步时间戳 uint32 小端 (当前固件不解析，用作帧计数器)
    每条命令:  Control_Mode(uint16 小端) + Command(int16 小端)
    [tail]     帧尾 0xBB
    """

    HEADER       = 0xAA
    TAIL         = 0xBB
    HEADER_SIZE  = 7            # 帧头(1) + ID(1) + Func(1) + 时间戳(4)
    TAIL_SIZE    = 1
    CMD_SIZE     = 4            # Control_Mode(2) + Command(2)
    MAX_COMMANDS = 14           # = router_rec_handle.h ROUTER_RX_MAX_CMD_CNT
    MAX_FRAME_SIZE = HEADER_SIZE + MAX_COMMANDS * CMD_SIZE + TAIL_SIZE   # = 64

    CANFD_VALID_SIZES = (8, 12, 16, 20, 24, 32, 48, 64)


# ==================================================================== 通信功能码
class TxFunc(IntEnum):
    """router_tran_handle.h RouterTxFuncCode (MCU -> 上位机)。"""

    USB_USART_NORMAL = 0    # Router_Comm_USB_USART_Tx_Normal，1kHz
    USB_USART_HIGH   = 1    # Router_Comm_USB_USART_Tx_High，  10kHz
    FDCAN_NORMAL     = 2    # Router_Comm_FDCAN_Tx_Normal，    1kHz
    FDCAN_HIGH       = 3    # Router_Comm_FDCAN_Tx_High，      10kHz


TX_ACCEPTED_FUNCS = (TxFunc.USB_USART_NORMAL, TxFunc.FDCAN_NORMAL)

# 低速曲线采样频率
LOW_SPEED_FREQ = 1000.0


class RxFunc(IntEnum):
    """
    router_rec_handle.h RouterRxFuncCode (上位机 -> MCU)。取值已与固件枚举逐条核对。

    ★ 这个字段现在两道关卡都会查，填错 = 整帧被静默丢弃，现象和"串口不通"一模一样：
      ① 网关 (router_app_host_mcu_tranf.c USBToFdcan)：只有 Func == FDCAN_CMD 且目标 ID
         不是网关自己，才会把帧原样转到 FDCAN 上；否则直接 return。
      ② MCU (router_app_motor_cmd.c FrameCheck)：Func 不是这两个值之一即 ERR_INVALID_PARAM。

    所以 PC 侧虽然走 USB 连的网关，Func 也必须填 FDCAN_CMD —— 它标的是【网关往下转发用的
    那条总线】，不是 PC 到网关这一段。USB_USART_CMD 只适用于把命令直连到 MCU 的调试链路。
    """

    USB_USART_CMD = 3       # Router_Comm_USB_USART_Cmd
    FDCAN_CMD     = 4       # Router_Comm_FDCAN_Cmd


# ==================================================================== MCU 拓扑
class McuConfig:
    """
    机械手 MCU 硬件拓扑。

    节点号见 router_common.h RouterCommObject：MCU0~MCU7 = 0xB0~0xB7 (新版头文件已补齐 8 个)。
    本工程只用低速策略 (1kHz)：每个 MCU 管两个电机，各注册 5 条低速曲线
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
# 电机二的 mode = 电机一的 mode + 300 (router_rec_handle.h 里 202/502、232/532、…、322/622
# 逐条如此)。所以功能表只列电机一的码，电机二用这个步长换算。
MOTOR_MODE_STRIDE = 300


class MotorFunc(IntEnum):
    """
    Control_Mode 功能码 —— 取值为【电机一】的 RouterRxControlMode，电机二 = 本值 + 300。

    每个成员携带 (电机一功能码, scale)：
      scale = 把浮点物理量转成下发 int16 的乘数，即 param = round(物理量 × scale)，
              逐条取自 router_rec_handle.c 的 handler 换算 (param -> 物理量)。
      物理量一律取【输出轴】单位。
    """

    def __new__(cls, code, scale):
        """让每个成员既是电机一的功能码 int(func)，又携带量化乘数 .scale (见类文档)。"""
        obj = int.__new__(cls, code)
        obj._value_ = code
        obj.scale = scale
        return obj

    #                            M1 code  scale     # router_rec_handle.c handler (param -> 物理量)
    # —— 基本控制 (输出轴单位) ——
    THETA_GEAR               = (   206,  100.0)   # θ = param/100        (ServiceThetamAccCmd)
    OMEGA_GEAR               = (   207,  100.0)   # ω = param/100        (ServiceVelCmd)
    TORQUE_GEAR              = (   208,    1.0)   # τ(mN·m) = param      (ServiceTorqueCmd)
    IQ                       = (   209, 1000.0)   # iq = param·0.001
    ID                       = (   210, 1000.0)   # id = param·0.001
    # —— 阻抗 (★ 新固件换算已变，见下) ——
    IMPEDANCE_SPRING         = (   232,  100.0)   # k = param/100    (旧协议 /10)
    IMPEDANCE_DAMPER         = (   233, 1000.0)   # d = param/1000   (旧协议 /100)
    IMPEDANCE_INERTIA        = (   234,10000.0)   # j = param/10000  (旧协议 /1000)
    IMPEDANCE_SPRING_ORIGIN  = (   235,  100.0)   # 原点 = param/100
    # —— 各环 PID ——
    CUR_PID_KP               = (   252, 1000.0)   # param·0.001
    CUR_PID_KI               = (   253, 1000.0)
    VEL_PID_KP               = (   254, 1000.0)
    VEL_PID_KI               = (   255, 1000.0)
    POS_PID_KP               = (   256,    1.0)   # param (无缩放)
    POS_PID_KI               = (   257,    1.0)
    # —— 错误清除 (无参数，param 被固件忽略) ——
    CLEAR_INNER_ERROR        = (   270,    1.0)   # 清内环错误，由固件决定是否从内环错误态恢复
    CLEAR_OUTER_ERROR        = (   271,    1.0)   # 清外环错误，由固件决定是否从外环错误态恢复
    # —— 状态机 ——
    DISPATCH_STATE           = (   272,    1.0)   # 状态机派发；param = 状态码 (见 MotorState)
    # —— 轨迹 (输出轴单位) ——
    TRAJ_INIT                = (   282,    1.0)   # 无参数
    TRAJ_DEINIT              = (   283,    1.0)   # 无参数
    TRAJ_VEL_MAX             = (   284,  100.0)
    TRAJ_ACL_MAX             = (   285,  100.0)
    TRAJ_POS_CMD             = (   286,  100.0)
    # —— 回零 (Homing，输出轴单位)。新固件把前/反向的力矩与位置拆成了 4 条命令 ——
    HOMING_INIT              = (   301,    1.0)   # 触发回零 (无参数)
    HOMING_FORWARD_TORQUE    = (   302,    1.0)   # 前向力矩(mN·m) = param
    HOMING_BACKWARD_TORQUE   = (   303,    1.0)   # 反向力矩(mN·m) = param   (新增)
    HOMING_FORWARD_POSITION  = (   304,  100.0)   # 前向位置 = param/100      (新增)
    HOMING_BACKWARD_POSITION = (   305,  100.0)   # 反向位置 = param/100
    # —— 系统辨识 ——
    SYS_IDEN                 = (   311,    1.0)   # 触发系统辨识 (无参数)；上位机暂未开放按钮
    # —— 设备级 ——
    FLASHING_PARAMS          = (   321,    1.0)   # 按配置序号重置控制参数为预设并刷 Flash；param = config_index
    CLEAR_FLASH_ERROR        = (   322,    1.0)   # 清除 Flash 错误 (无参数)

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


# ==================================================================== 帧尾冲突自检 (应恒为空)
TAIL_CONFLICT_MODES = tuple(
    code
    for func in MotorFunc
    for code in (func.code_for(0), func.code_for(1))
    if (code & 0xFF) == RxFrame.TAIL
)


# ==================================================================== 电机状态机
class MotorState:
    """
    电机状态机 —— 逐条对应固件 MotorStateType 枚举 (已与新固件
    motor_state_machine.h 重新核对，本次协议改动未涉及状态码)。
    用于：① 解析 Tx 反馈里的 state 曲线；② 作为 DISPATCH_STATE 命令的参数(状态码)。
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
        # —— 错误 (内/外环，150+) ——
        "InnerEncoderReadError": 150, "InnerCurrentReadError": 151,
        "InnerParamLoadError": 152, "OuterParamLoadError": 153,
        "OuterThetaeCalibSaveError": 154, "OuterCurrentCalibSaveError": 155,
        "InnerOuterMismatchError": 165,
    }
    NAMES = {v: k for k, v in CODES.items()}

    @classmethod
    def name(cls, code):
        """状态码 -> 状态名 (未知码返回 Unknown(code)，None 原样返回)。"""
        if code is None:
            return None
        return cls.NAMES.get(code, f"Unknown({code})")
