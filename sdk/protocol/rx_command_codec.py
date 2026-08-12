"""
Rx 命令帧编码器：上位机 -> MCU。

组帧：把一串 MotorCommand 编码成 0xAA … 0xBB 帧。
★ 新协议里电机由【mode 自带的 M1/M2 后缀】区分 (电机二 = 电机一 + 300)，
  不再靠"命令在帧里的前/后半段位置"，所以本层只是把命令平铺进帧，
  旧版的"两半用 NOP_MODE 补齐到等长"机制已彻底废除。

帧格式 (router_rec_handle.c)：
    [0]        帧头 0xAA
    [1]        目标 ID
    [2]        Func
    [3..6]     同步时间戳 uint32 小端 (固件当前不解析，本层填帧计数器)
    每条命令:  Control_Mode(uint16 小端) + Command(int16 小端)
    [tail]     帧尾 0xBB

约束：0 < 命令条数 <= 14 (ROUTER_RX_MAX_CMD_CNT)，满帧 = 7 + 14×4 + 1 = 64 字节。
"""

import struct

from sdk.protocol.constants import RxFrame, RxFunc, McuConfig, TAIL_CONFLICT_MODES


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

    @staticmethod
    def _check_tail_conflict(commands):
        """
        拦截 mode 低字节 == 帧尾 的命令 (现行帧尾 0xBB 下不存在这种 mode，本检查是护栏)。

        固件靠"每 4 字节扫一次、遇帧尾字节即认为帧结束"来定位命令条数，这类命令会连同
        它之后的所有命令被静默丢弃 (详见 constants.TAIL_CONFLICT_MODES 的说明)。
        与其发出去后无声失效，不如在这里抛错。
        """
        bad = [c.mode for c in commands if (c.mode & 0xFF) == RxFrame.TAIL]
        if bad:
            raise ValueError(
                f"命令 mode {bad} 的低字节与帧尾 0x{RxFrame.TAIL:02X} 相同，"
                f"下位机会把它当成帧尾并丢弃其后所有命令 (功能表内的冲突码: "
                f"{list(TAIL_CONFLICT_MODES) or '无'})，需固件侧改帧尾值或改这些 mode 的编号")

    def encode(self, mcu, commands, counter=0, func=RxFunc.FDCAN_CMD):
        """
        参数:
            mcu:      index 0~7 或 ID 0xB0~0xB7
            commands: list[MotorCommand]，mode 已是"带电机后缀"的 Control_Mode，顺序即执行顺序
            counter:  写入时间戳字段的帧计数器 0~0xFFFFFFFF
            func:     RxFunc，默认 FDCAN_CMD —— 经网关转发到 MCU 的唯一可用取值 (见 RxFunc 文档)
        返回:
            bytes
        编码后统一补齐到 CAN-FD 合法长度 (本项目走 FDCAN)。
        """
        # 非法 Func 会被网关和 MCU 双双静默丢帧，在这里先炸掉，别让它变成"发了但没反应"
        func = RxFunc(func)

        commands = list(commands)
        if not commands:
            raise ValueError("命令列表为空")
        if len(commands) > RxFrame.MAX_COMMANDS:
            raise ValueError(
                f"命令数量超出下位机上限({RxFrame.MAX_COMMANDS}): {len(commands)}")
        self._check_tail_conflict(commands)

        target_id = McuConfig.id_of(mcu)

        data = bytearray()
        data.append(RxFrame.HEADER)
        data.append(target_id & 0xFF)
        data.append(int(func) & 0xFF)
        data += struct.pack("<I", counter & 0xFFFFFFFF)
        for cmd in commands:
            data += struct.pack("<H", cmd.mode & 0xFFFF)
            data += struct.pack("<h", self.clamp_int16(cmd.value))
        data.append(RxFrame.TAIL)

        return bytes(self._canfd_pad(data))
