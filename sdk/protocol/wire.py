"""
线上帧格式与功能码 —— 纯粹描述"字节怎么排、Func 填什么"，不含任何业务含义。

对应固件 router_app_fix_freq.h (Tx) / router_app_motor_cmd.h (Rx)。
"""

from enum import IntEnum


# ==================================================================== Tx 反馈帧
class TxFrame:
    """
    MCU -> 上位机 反馈帧。

    [0]        帧头 0xCC
    [1]        ID (发送源 = MCU 节点号)
    [2]        Func (TxFunc)
    [3..6]     帧计数器 uint32 小端
    [7..]      curve_cnt 条曲线，每条 4 字节小端 (按注册索引顺序)
    [tail]     帧尾 0xDD

    ★ 曲线不是清一色 float32。固件按注册时的类型把每条曲线塞进一个 32 位原始字，
      上位机必须按同一份注册顺序与类型解读 —— 布局见 topology.CurveLayout。
    """

    HEADER          = 0xCC
    TAIL            = 0xDD
    HEADER_SIZE     = 7      # 帧头(1) + ID(1) + Func(1) + 帧计数器(4)
    TAIL_SIZE       = 1      # 帧尾(1)；无软件 CRC，链路层自带校验
    CURVE_SIZE      = 4      # 每条曲线一个 32 位字 (float32 或 uint32，按注册类型)
    MAX_CURVE_COUNT = 14     # ROUTER_TX_MAX_CURVE_CNT
    MAX_FRAME_SIZE  = HEADER_SIZE + MAX_CURVE_COUNT * CURVE_SIZE + TAIL_SIZE   # = 64
    COUNTER_MASK    = 0xFFFFFFFF

    @classmethod
    def frame_size(cls, curve_count):
        """给定曲线数量算整帧长度 (帧里没有长度字段，长度由曲线数量唯一确定)。"""
        return cls.HEADER_SIZE + curve_count * cls.CURVE_SIZE + cls.TAIL_SIZE


# ==================================================================== Rx 命令帧
class RxFrame:
    """
    上位机 -> MCU 命令帧。

    [0]        帧头 0xAA
    [1]        ID (目标 MCU 节点号)
    [2]        Func (RxFunc)
    [3..6]     同步时间戳 uint32 小端 (当前固件不解析，用作帧计数器)
    每条命令:  Control_Mode(uint16 小端) + Command(int16 小端)
    [tail]     帧尾 0xBB
    """

    HEADER         = 0xAA
    TAIL           = 0xBB
    HEADER_SIZE    = 7       # 帧头(1) + ID(1) + Func(1) + 时间戳(4)
    TAIL_SIZE      = 1
    CMD_SIZE       = 4       # Control_Mode(2) + Command(2)
    MAX_COMMANDS   = 14      # ROUTER_RX_MAX_CMD_CNT
    MAX_FRAME_SIZE = HEADER_SIZE + MAX_COMMANDS * CMD_SIZE + TAIL_SIZE   # = 64

    CANFD_VALID_SIZES = (8, 12, 16, 20, 24, 32, 48, 64)


# ==================================================================== 通信功能码
class TxFunc(IntEnum):
    """
    反馈帧 Func (MCU -> 上位机)，对应固件 RouterTxFuncCode。

    ★ 新固件把功能码收敛成了"只标速度策略、不标总线"两个值 —— 旧协议里区分
      USB/USART 与 FDCAN 的 0~3 四个值已经没有了，低速帧无论走哪条总线都填 NORMAL。
    """

    NORMAL = 0    # Router_Comm_Tx_Normal，1kHz 低速策略
    HIGH   = 1    # Router_Comm_Tx_High，  10kHz 高速策略


# 本工程只跑低速策略，定帧时只认这一个 Func
TX_ACCEPTED_FUNCS = (TxFunc.NORMAL,)

# 低速曲线采样频率 (Hz)，用于把帧计数器折算成硬件时间轴
LOW_SPEED_FREQ = 1000.0


class RxFunc(IntEnum):
    """
    命令帧 Func (上位机 -> MCU)。取值已与固件枚举逐条核对。

    ★ 这个字段两道关卡都会查，填错 = 整帧被静默丢弃，现象和"串口不通"一模一样：
      ① 网关 (router_app_host_mcu_tranf.c USBToFdcan)：只有 Func == FDCAN_CMD 且目标 ID
         不是网关自己，才会把帧原样转到 FDCAN 上；否则直接 return。
      ② MCU (router_app_motor_cmd.c FrameCheck)：Func 不是这两个值之一即 ERR_INVALID_PARAM。

    所以 PC 侧虽然走 USB 连的网关，Func 也必须填 FDCAN_CMD —— 它标的是【网关往下转发用的
    那条总线】，不是 PC 到网关这一段。USB_USART_CMD 只适用于把命令直连到 MCU 的调试链路。
    """

    USB_USART_CMD = 3       # Router_Comm_USB_USART_Cmd
    FDCAN_CMD     = 4       # Router_Comm_FDCAN_Cmd
