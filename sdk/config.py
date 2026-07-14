"""
应用级配置：硬件默认参数、通信选项、GUI 刷新与阈值。

把这些集中起来，避免像旧代码那样在 GUI 里散写 "COM3" / 115200 / 50ms / 5.0% 。
"""

from dataclasses import dataclass, field

from sdk.protocol.constants import CommFunc


@dataclass
class SerialConfig:
    port: str = "COM3"
    baudrate: int = 115200
    timeout: float = 0.1


@dataclass
class AppConfig:
    serial: SerialConfig = field(default_factory=SerialConfig)

    # 命令帧 Func。
    #   物理链路是 USB 虚拟串口 / UART 时，语义上应为 RX_USB_USART_COMMAND。
    #   但当前固件 (router.c) 尚未校验 Func，为不破坏既有可运行行为，
    #   这里默认沿用 FDCAN 命令码；确认固件后可改成 RX_USB_USART_COMMAND。
    command_func: CommFunc = CommFunc.RX_FDCAN_COMMAND

    # 是否把命令帧补齐到 CAN-FD 合法长度。
    #   走 FDCAN 时必须补齐；走纯串口时补不补都能被固件按 command_num 正确解析。
    #   默认跟随 command_func 是否为 FDCAN。
    pad_canfd: bool = None  # None => 自动根据 command_func 判定

    # GUI 刷新周期 (ms) ≈ 20 Hz
    refresh_ms: int = 50

    # 丢包率着色阈值 (%)
    loss_warn_threshold: float = 0.0
    loss_err_threshold: float = 5.0

    # 日志最大行数
    log_max_lines: int = 400

    def resolved_pad_canfd(self):
        """
        解析最终是否补齐 CAN-FD 长度：显式设了 pad_canfd 就用它，
        否则按 command_func 是否为 FDCAN 自动判定。
        用法: SerialCommander(..., pad_canfd=cfg.resolved_pad_canfd())。
        """
        if self.pad_canfd is not None:
            return self.pad_canfd
        return self.command_func == CommFunc.RX_FDCAN_COMMAND
