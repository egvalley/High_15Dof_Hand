"""
应用级配置：硬件默认参数、通信选项、GUI 刷新与阈值。

把这些集中起来，避免像旧代码那样在 GUI 里散写 "COM3" / 115200 / 50ms / 5.0% 。
"""

from dataclasses import dataclass, field

from sdk.protocol import RxFunc, McuConfig


@dataclass
class SerialConfig:
    port: str = "COM3"
    baudrate: int = 115200
    timeout: float = 0.1


@dataclass
class AppConfig:
    serial: SerialConfig = field(default_factory=SerialConfig)

    # 命令帧 Func。本项目是 PC --USB--> 网关 --FDCAN--> MCU，网关只转发 Func == FDCAN_CMD
    # 的帧，所以这里必须是 FDCAN_CMD (详见 protocol.wire.RxFunc)。改成 USB_USART_CMD 会整帧被丢。
    command_func: RxFunc = RxFunc.FDCAN_CMD

    # 下位机注册的低速曲线数量。
    feedback_curve_count: int = McuConfig.LOW_SPEED_CURVE_COUNT

    # GUI 刷新周期 (ms) ≈ 20 Hz
    refresh_ms: int = 50

    # 丢包率着色阈值 (%)
    loss_warn_threshold: float = 0.0
    loss_err_threshold: float = 5.0

    # 日志最大行数
    log_max_lines: int = 400
