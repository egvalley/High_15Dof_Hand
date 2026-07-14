"""
Rx 命令帧编码器：上位机 -> MCU。

只做组帧：把一串 MotorCommand 编码成 0x3B ... 0x1E 的字节帧。
电机由每条命令的 mode 自带的 M0/M1 后缀区分 (见 constants.mode_for)，
因此这里不再区分“电机0列表 / 电机1列表”，只是一条扁平命令流。

帧格式 (对照 router.c DeviceRouter_RxPayloadHandler)：
    [0]        帧头 0x3B
    [1]        目标 ID
    [2]        帧计数器
    [3]        Func
    [4]        command_num
    每条命令:  mode(1) + param(int16 小端, 2)
    [tail]     帧尾 0x1E

约束 (router.c)：0 < command_num <= 20。奇偶均可 (两段循环并集覆盖全部)。
"""

import struct

from sdk.protocol.constants import RxFrame, CommFunc, McuConfig


class RxCommandCodec:

    @staticmethod
    def clamp_int16(v):
        v = int(round(v))
        return max(-32768, min(32767, v))

    @classmethod
    def _canfd_pad(cls, frame):
        n = len(frame)
        for s in RxFrame.CANFD_VALID_SIZES:
            if n <= s:
                return frame + bytes(s - n)
        return frame

    def encode(self, mcu, commands,
               counter=0, func=CommFunc.RX_FDCAN_COMMAND, pad_canfd=None):
        """
        参数:
            mcu:      index 0~7 或 ID 0xB0~0xB7
            commands: list[MotorCommand]，每条命令的 mode 已含电机信息
            counter:  帧计数器 0~255
            func:     CommFunc
            pad_canfd: 是否补齐到 CAN-FD 合法长度；None 时按 func 是否 FDCAN 判定
        返回:
            bytes
        """
        command_num = len(commands)
        if command_num == 0:
            raise ValueError("命令列表为空")
        if command_num > RxFrame.MAX_COMMANDS:
            raise ValueError(f"命令数量超出下位机上限({RxFrame.MAX_COMMANDS}): {command_num}")

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
