"""
硬件拓扑与曲线表 —— "帧里第 N 个 32 位字是谁的哪个量、什么类型"。

MCU 节点号见固件 router_common.h RouterCommObject；曲线顺序与类型必须与固件
RouterAppFixFreq_Init 里的注册顺序逐条对齐，两边改动要同步。
"""

import math
import struct

from sdk.protocol.info_word import is_info_word
from sdk.protocol.wire import TxFrame


# 单个电机的低速曲线表：(字段名, struct 格式)，顺序即固件注册顺序。
# ★ 第 0 条是 uint32 信息字位域 (状态码 + 三段错误，见 info_word)，不是 float ——
#   这是新协议相对旧协议的关键改动，整行不能按清一色 float32 解。
MOTOR_CURVES = (
    ("info",   "I"),
    ("theta",  "f"),
    ("omega",  "f"),
    ("acl",    "f"),
    ("torque", "f"),
)

INFO_FIELD = "info"


# ==================================================================== MCU 拓扑
class McuConfig:
    """
    机械手 MCU 硬件拓扑。

    MCU0~MCU7 = 0xB0~0xB7；每个 MCU 管两个电机，各注册 MOTOR_CURVES 这几条低速曲线 (1kHz)。
    """

    COUNT = 8
    BASE_ID = 0xB0
    IDS = tuple(range(BASE_ID, BASE_ID + COUNT))   # 0xB0 ~ 0xB7

    MOTORS_PER_MCU = 2
    CURVES_PER_MOTOR = len(MOTOR_CURVES)                        # = 5
    LOW_SPEED_CURVE_COUNT = MOTORS_PER_MCU * CURVES_PER_MOTOR   # = 10

    MOTOR_CURVE_FIELDS = tuple(name for name, _fmt in MOTOR_CURVES)

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


# ==================================================================== 曲线布局
class CurveLayout:
    """
    一帧曲线的布局：由曲线数量定出整帧长度、逐条曲线的解包类型、以及哪些下标是信息字。

    曲线数量等于标准的"2 电机 × 5 字段"时按 MOTOR_CURVES 逐条取类型；
    对不上说明下位机注册表被改过、布局未知，退回"全 float32"的通用解法
    (电机字段映射不出来，但原始值仍能取到)。
    """

    def __init__(self, curve_count):
        """由曲线数量构造布局；curve_count 超出 1~TxFrame.MAX_CURVE_COUNT 抛 ValueError。"""
        if not 0 < curve_count <= TxFrame.MAX_CURVE_COUNT:
            raise ValueError(
                f"曲线数量非法: {curve_count} (应为 1~{TxFrame.MAX_CURVE_COUNT})")

        self.curve_count = curve_count
        self.frame_size = TxFrame.frame_size(curve_count)
        self.is_standard = curve_count == McuConfig.LOW_SPEED_CURVE_COUNT

        if self.is_standard:
            formats = [fmt for _name, fmt in MOTOR_CURVES] * McuConfig.MOTORS_PER_MCU
            off = McuConfig.MOTOR_CURVE_FIELDS.index(INFO_FIELD)
            self.info_indexes = frozenset(
                m * McuConfig.CURVES_PER_MOTOR + off
                for m in range(McuConfig.MOTORS_PER_MCU))
        else:
            formats = ["f"] * curve_count
            self.info_indexes = frozenset()

        self._row_fmt = "<" + "".join(formats)

    def unpack(self, buf, offset=TxFrame.HEADER_SIZE):
        """从 buf 的 offset 处按本布局解出整行曲线值 (信息字为 int，其余为 float)。"""
        return struct.unpack_from(self._row_fmt, buf, offset)

    def is_plausible(self, row):
        """
        整行曲线值的合理性检查 —— 帧里没有 CRC，这是定帧的最后一道关卡。

        float 曲线：NaN / Inf 只可能来自错位定帧，曲线本身不会是非有限值。
        信息字曲线：状态段与四段错误位域之外一旦有置位就不是信息字，同样说明错位。
        """
        for i, value in enumerate(row):
            ok = is_info_word(value) if i in self.info_indexes else math.isfinite(value)
            if not ok:
                return False
        return True

    def motor_values(self, row, motor):
        """取第 motor 个电机那 5 条曲线，返回 {字段名: 值}；布局非标准时返回空 dict。"""
        if not self.is_standard:
            return {}
        base = motor * McuConfig.CURVES_PER_MOTOR
        return {name: row[base + off]
                for off, name in enumerate(McuConfig.MOTOR_CURVE_FIELDS)}
