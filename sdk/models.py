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
    BOTH = "both"

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
    """一条 (控制模式, 已量化的 int16 参数)。"""

    mode: int
    value: int


# ==================================================================== 反馈模型
@dataclass
class MotorFeedback:
    """单个电机的一帧反馈。"""

    state_code: Optional[int] = None
    state_name: str = "—"
    theta: Optional[float] = None
    omega: Optional[float] = None
    acl: Optional[float] = None
    torque: Optional[float] = None


@dataclass
class McuFeedback:
    """单个 MCU 的一帧完整反馈快照。"""

    mcu_index: int
    mcu_id: int
    func: int
    curve_count: int
    sample_count: int
    frame_counter: int
    hw_time: float
    motors: List[MotorFeedback] = field(default_factory=list)


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
