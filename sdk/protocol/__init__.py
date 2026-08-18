"""
协议层 —— 上位机侧的唯一协议定义源，与固件 router_* 逐条对齐。

按职责分四个模块，改动时只碰对应的那一个：
    wire.py      帧格式与功能码 (字节怎么排、Func 填什么)
    topology.py  MCU 拓扑与曲线表布局 (第 N 个 32 位字是谁的哪个量、什么类型)
    info_word.py 电机信息字位域 (反馈里的状态码与错误怎么拆)
    commands.py  控制命令码与定点缩放 (能下发什么、怎么量化) + 状态码表

外部一律从本包导入 (from sdk.protocol import XXX)，不要直接 import 子模块，
这样以后再拆分/合并模块不会波及调用方。
"""

from sdk.protocol.wire import (
    TxFrame, RxFrame, TxFunc, RxFunc, TX_ACCEPTED_FUNCS, LOW_SPEED_FREQ,
)
from sdk.protocol.info_word import (
    ErrorBit, ErrorSeg, ERROR_VALID_MASK, ERROR_KNOWN_MASK,
    INFO_STATE_MASK, INFO_STATE_SHIFT, INFO_VALID_MASK,
    is_info_word, state_of, errors_of, error_names,
)
from sdk.protocol.topology import McuConfig, CurveLayout, MOTOR_CURVES
from sdk.protocol.commands import (
    MotorFunc, MotorState, MOTOR_MODE_STRIDE, TAIL_CONFLICT_MODES,
)
from sdk.protocol.tx_feedback_codec import TxFeedbackCodec
from sdk.protocol.rx_command_codec import RxCommandCodec

__all__ = [
    # wire
    "TxFrame", "RxFrame", "TxFunc", "RxFunc", "TX_ACCEPTED_FUNCS", "LOW_SPEED_FREQ",
    # info_word
    "ErrorBit", "ErrorSeg", "ERROR_VALID_MASK", "ERROR_KNOWN_MASK",
    "INFO_STATE_MASK", "INFO_STATE_SHIFT", "INFO_VALID_MASK",
    "is_info_word", "state_of", "errors_of", "error_names",
    # topology
    "McuConfig", "CurveLayout", "MOTOR_CURVES",
    # commands
    "MotorFunc", "MotorState", "MOTOR_MODE_STRIDE", "TAIL_CONFLICT_MODES",
    # codec
    "TxFeedbackCodec", "RxCommandCodec",
]
