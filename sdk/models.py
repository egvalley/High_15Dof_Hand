"""
数据模型：用 dataclass / Enum 取代原来散落的 dict / None / "both" / 0/1 混用。

GUI 与业务层今后应操作这些结构，而不是直接解析曲线数组或裸字典。
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List


# ==================================================================== 目标选择
class MotorTarget(Enum):
    """控制命令的目标电机。"""

    MOTOR_0 = 0
    MOTOR_1 = 1
    BOTH = 2

    def hits_motor0(self):
        """本目标是否作用于电机0 (MOTOR_0 或 BOTH 时为 True)。用于决定电机0的命令/输入框是否生效。"""
        return self in (MotorTarget.MOTOR_0, MotorTarget.BOTH)

    def hits_motor1(self):
        """本目标是否作用于电机1 (MOTOR_1 或 BOTH 时为 True)。"""
        return self in (MotorTarget.MOTOR_1, MotorTarget.BOTH)

    @property
    def tag(self):
        """给日志用的简短标签：'M0' / 'M1' / 'M0+M1'。"""
        return {MotorTarget.MOTOR_0: "M0",
                MotorTarget.MOTOR_1: "M1",
                MotorTarget.BOTH: "M0+M1"}[self]


# ==================================================================== 命令模型
@dataclass(frozen=True)
class MotorCommand:
    """
    一条 (控制模式, 已量化的 int16 参数)。

    mode 是【带电机后缀】的 Control_Mode (uint16)：电机0 用 MotorFunc 本值，
    电机1 用本值 + 300 (见 MotorFunc.code_for)。新协议不再靠命令位置区分电机。
    """

    mode: int
    value: int


# ==================================================================== 反馈模型
@dataclass
class MotorFeedback:
    """
    单个电机的一帧反馈。

    第 0 条曲线是 uint32 信息字 (info_word)，一个字里装了两类东西，本层把它拆开存：
      状态段 bit24~31 -> state_code / state_name，单值，就是电机状态机当前状态；
      错误段 bit0 ~23 -> error_word / error_names，位域，可以同时报多个，也可以一个不报。
    位的含义与拆位规则由协议层 sdk.protocol.info_word 负责，本层只存拆好的结果。
    界面上这两段分两列显示，不要再合成一列。
    """

    info_word: Optional[int] = None
    state_code: Optional[int] = None
    state_name: Optional[str] = None
    error_word: int = 0
    error_names: List[str] = field(default_factory=list)
    theta: Optional[float] = None
    omega: Optional[float] = None
    acl: Optional[float] = None
    torque: Optional[float] = None

    @property
    def has_frame(self):
        """本条反馈有没有真的解到信息字 (没有则各列显示占位符)。"""
        return self.info_word is not None

    @property
    def has_error(self):
        """本帧是否有任何错误置位 (无反馈时为 False)。"""
        return bool(self.error_word)

    @property
    def state_text(self):
        """状态列显示文本：无反馈为 '—'，否则为状态名 (未知码为 Unknown(n))。"""
        if not self.has_frame:
            return "—"
        return self.state_name or "—"

    @property
    def error_text(self):
        """错误列显示文本：无反馈为 '—'，无错误为 '正常'，多个错误用 ' | ' 并列。"""
        if not self.has_frame:
            return "—"
        return " | ".join(self.error_names) or "正常"


@dataclass
class McuFeedback:
    """
    单个 MCU 的一帧完整反馈快照。

    新协议一帧就是一个采样点 (旧协议的 sample_count 已不存在)，
    frame_counter 为 uint32，hw_time = frame_counter / 1kHz。
    """

    mcu_index: int
    mcu_id: int
    func: int
    curve_count: int
    frame_counter: int
    hw_time: float
    motors: List[MotorFeedback] = field(default_factory=list)
    # 原始曲线值 (按注册索引顺序)，曲线表与"2 电机 × 5 字段"不一致时靠它取数。
    # 元素类型随注册类型走：信息字曲线是 int (uint32 位域)，其余是 float
    curves: List = field(default_factory=list)


# ==================================================================== 通信统计
@dataclass
class LinkStats:
    """单个 MCU 的链路统计快照。"""

    received: int = 0
    lost: int = 0
    loss_rate: float = 0.0
    last_sec_received: int = 0
    last_sec_lost: int = 0
    ever_received: bool = False


# ==================================================================== 执行结果
@dataclass
class CommandResult:
    """一次“发往某个 MCU”的结果，供 GUI 记日志。"""

    mcu_index: int
    ok: bool
    message: str = ""
