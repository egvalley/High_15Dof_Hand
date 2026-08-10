"""
Tx 反馈帧解码器：MCU -> 上位机。

帧格式 (router_tran_handle.c)：
    [0]        帧头 0xCC
    [1]        ID (发送源 MCU 节点号)
    [2]        Func (RouterTxFuncCode 0~3)
    [3..6]     帧计数器 uint32 小端
    [7..]      curve_count 个 float32 小端 (按注册索引顺序)
    [tail]     帧尾 0xDD


"""

import math
import struct

from sdk.protocol.constants import (TxFrame, McuConfig, MotorState,
                                    TX_ACCEPTED_FUNCS, LOW_SPEED_FREQ)
from sdk.models import McuFeedback, MotorFeedback


class TxFeedbackCodec:
    """有状态的字节流 -> 帧 解码器 (内部维护拼帧缓冲区)。"""

    def __init__(self, curve_count=McuConfig.LOW_SPEED_CURVE_COUNT):
        """
        新建解码器，内部拼帧缓冲区置空。每条串口链路持有一个实例。
        参数: curve_count 下位机注册的曲线数量 (决定帧长；新协议帧里无此字段)。
        """
        if not 0 < curve_count <= TxFrame.MAX_CURVE_COUNT:
            raise ValueError(
                f"曲线数量非法: {curve_count} (应为 1~{TxFrame.MAX_CURVE_COUNT})")
        self.curve_count = curve_count
        self._frame_len = TxFrame.frame_size(curve_count)
        self._row_fmt = f"<{curve_count}f"
        self._buffer = bytearray()

    def reset(self):
        """清空拼帧缓冲区。重新连接串口时调用，避免旧的半截字节污染新数据。"""
        self._buffer.clear()

    # ---------------------------------------------------------------- 主接口
    def feed(self, data):
        """
        投喂新字节，返回本次能完整解析出的 McuFeedback 列表。

        参数:
            data: bytes / bytearray，新到达的原始字节。
        返回:
            list[McuFeedback]
        """
        if data:
            self._buffer.extend(data)
        return self._drain()

    # ---------------------------------------------------------------- 内部
    def _drain(self):
        """
        从缓冲区尽可能多地切出完整帧：定位帧头 -> 按曲线数量取定长帧 -> 校验 -> 解码。
        数据不足则保留等待下次；任一校验失败就丢 1 字节继续找下一个帧头
        (CAN FD 帧尾之后的补零字节也在这里被跳过)。
        返回本轮解出的 list[McuFeedback]。
        """
        buf = self._buffer
        out = []

        while len(buf) >= self._frame_len:
            hi = buf.find(TxFrame.HEADER)
            if hi == -1:
                buf.clear()
                break
            if hi > 0:
                del buf[:hi]
            if len(buf) < self._frame_len:
                break

            # 无 CRC，只能靠这几项定帧：ID 合法 / Func 合法 / 帧尾就位
            mcu_id = buf[1]
            if not (McuConfig.BASE_ID <= mcu_id < McuConfig.BASE_ID + McuConfig.COUNT):
                del buf[:1]
                continue

            func = buf[2]
            if func not in TX_ACCEPTED_FUNCS:
                del buf[:1]
                continue

            if buf[self._frame_len - 1] != TxFrame.TAIL:
                del buf[:1]
                continue

            row = struct.unpack_from(self._row_fmt, buf, TxFrame.HEADER_SIZE)
            if not all(math.isfinite(v) for v in row):
                # NaN / Inf 只可能来自错位定帧 (曲线本身不会是非有限值)
                del buf[:1]
                continue

            frame_counter = struct.unpack_from("<I", buf, 3)[0]
            del buf[:self._frame_len]

            out.append(self._build_feedback(mcu_id, func, frame_counter, row))

        return out

    def _build_feedback(self, mcu_id, func, frame_counter, row):
        """把一帧已校验的数据组成 McuFeedback (曲线值按 2 电机 × 5 字段拆开)。"""
        return McuFeedback(
            mcu_index=McuConfig.to_index(mcu_id),
            mcu_id=mcu_id,
            func=func,
            curve_count=self.curve_count,
            frame_counter=frame_counter,
            # 新协议无时间戳；帧计数器每采样点 +1，按低速采样频率折算成秒
            hw_time=frame_counter / LOW_SPEED_FREQ,
            motors=self._row_to_motors(row),
            curves=list(row),
        )

    @staticmethod
    def _row_to_motors(row):
        """把一行 curve 值按 2 电机 x 5 字段 拆成 MotorFeedback 列表。"""
        if len(row) != McuConfig.LOW_SPEED_CURVE_COUNT:
            # 曲线布局不是标准的"2 电机 × 5 字段"，无法映射电机字段 (原始值仍在 curves 里)
            return [MotorFeedback() for _ in range(McuConfig.MOTORS_PER_MCU)]

        fields = McuConfig.MOTOR_CURVE_FIELDS
        motors = []
        for m in range(McuConfig.MOTORS_PER_MCU):
            base = m * McuConfig.CURVES_PER_MOTOR
            vals = {name: row[base + off] for off, name in enumerate(fields)}

            code = int(round(vals["state"]))
            motors.append(MotorFeedback(
                state_code=code,
                state_name=MotorState.name(code) or "—",
                theta=vals.get("theta"),
                omega=vals.get("omega"),
                acl=vals.get("acl"),
                torque=vals.get("torque"),
            ))
        return motors
