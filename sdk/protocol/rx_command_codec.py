"""
Rx 命令帧编码器：上位机 -> MCU。

组帧 + 电机路由：把"电机0命令" / "电机1命令"两组 MotorCommand 编码成 0x3B ... 0x1E 帧。
固件按命令在列表中的前/后半段位置区分电机 (见 router.c DeviceRouter_RxPayloadHandler)：
    前半段 (i <  command_num/2) -> 电机0 (M1)
    后半段 (i >= command_num/2) -> 电机1 (M2)
固件切分点是 command_num//2，故两半必须【等长】，短的一侧用 NOP_MODE 补齐，
最终 command_num = 2 × max(两组长度)，前一半属电机0、后一半属电机1，切分点恰好落在中间。

帧格式：
    [0]        帧头 0x3B
    [1]        目标 ID
    [2]        帧计数器
    [3]        Func
    [4]        command_num
    每条命令:  mode(1) + param(int16 小端, 2)
    [tail]     帧尾 0x1E

约束 (router.c)：0 < command_num <= 20，即每个电机最多 10 条命令。
"""

import struct

from sdk.protocol.constants import RxFrame, CommFunc, McuConfig, NOP_MODE
from sdk.models import MotorCommand


class RxCommandCodec:

    @staticmethod
    def clamp_int16(v):
        """四舍五入并夹到 int16 范围 [-32768, 32767]，防止量化溢出破坏帧。"""
        v = int(round(v))
        return max(-32768, min(32767, v))

    @classmethod
    def _canfd_pad(cls, frame):
        """把帧用 0 补齐到下一个 CAN-FD 合法长度 (8/12/…/64)；已超 64 则原样返回。"""
        n = len(frame)
        for s in RxFrame.CANFD_VALID_SIZES:
            if n <= s:
                return frame + bytes(s - n)
        return frame

    def encode(self, mcu, motor0_commands, motor1_commands,
               counter=0, func=CommFunc.RX_FDCAN_COMMAND, pad_canfd=None):
        """
        参数:
            mcu:             index 0~7 或 ID 0xB0~0xB7
            motor0_commands: list[MotorCommand]，发给电机0 (前半段/M1)
            motor1_commands: list[MotorCommand]，发给电机1 (后半段/M2)
            counter:         帧计数器 0~255
            func:            CommFunc
            pad_canfd:       是否补齐到 CAN-FD 合法长度；None 时按 func 是否 FDCAN 判定
        返回:
            bytes
        """
        # 两半用 NOP 补齐到等长，再拼成 [电机0命令... 电机1命令...]，令固件切分点落在正中
        half = max(len(motor0_commands), len(motor1_commands))
        if half == 0:
            raise ValueError("命令列表为空")
        nop = MotorCommand(NOP_MODE, 0)
        m0 = list(motor0_commands) + [nop] * (half - len(motor0_commands))
        m1 = list(motor1_commands) + [nop] * (half - len(motor1_commands))
        commands = m0 + m1

        command_num = len(commands)
        if command_num > RxFrame.MAX_COMMANDS:
            raise ValueError(
                f"命令数量超出下位机上限({RxFrame.MAX_COMMANDS}): {command_num} "
                f"(每个电机最多 {RxFrame.MAX_COMMANDS // 2} 条)")

        target_id = McuConfig.id_of(mcu)
        if pad_canfd is None:
            pad_canfd = (func == CommFunc.RX_FDCAN_COMMAND)

        data = bytearray()
        data.append(RxFrame.HEADER)
        data.append(target_id & 0xFF)
        data.append(counter & 0xFF)
        data.append(int(func) & 0xFF)
        data.append(command_num & 0xFF)
        for cmd in commands:
            data.append(cmd.mode & 0xFF)
            data += struct.pack("<h", self.clamp_int16(cmd.value))
        data.append(RxFrame.TAIL)

        if pad_canfd:
            data = self._canfd_pad(data)

        return bytes(data)
