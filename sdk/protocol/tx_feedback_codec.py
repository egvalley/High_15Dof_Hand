"""
Tx 反馈帧解码器：MCU -> 上位机。

职责：吃进原始字节流，吐出解析好的 McuFeedback。
只关心“这些字节是什么意思”，不碰串口、不碰线程、不碰 GUI。

帧格式 (对照 router.c)：
    [0]            帧头 0x5A
    [1]            MCU ID
    [2]            Func
    [3]            曲线数量 N
    [4 .. 4+N-1]   每条曲线的 log2 最大值
    [4+N]          帧计数器
    [5+N .. 8+N]   timestamp uint32 小端
    [9+N]          采样点数量 sample_count
    [10+N ..]      Payload：sample_count 行 x N 列 x int16 小端
    [..]           CRC16 小端 (低字节在前)
    [..]           帧尾 0x0D
"""

import struct

from sdk.protocol.constants import TxFrame, McuConfig, MotorState
from sdk.models import McuFeedback, MotorFeedback


class TxFeedbackCodec:
    """有状态的字节流 -> 帧 解码器 (内部维护拼帧缓冲区)。"""

    def __init__(self):
        """新建解码器，内部拼帧缓冲区置空。每条串口链路持有一个实例。"""
        self._buffer = bytearray()

    def reset(self):
        """清空拼帧缓冲区。重新连接串口时调用，避免旧的半截字节污染新数据。"""
        self._buffer.clear()

    # ---------------------------------------------------------------- 纯函数
    @staticmethod
    def calculate_crc16(data, length):
        """Modbus CRC16 (poly 0xA001, init 0xFFFF)。与固件查表法结果一致。"""
        crc = 0xFFFF
        for i in range(length):
            crc ^= data[i]
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    @staticmethod
    def dequantize(q_val, log2_max):
        """int16 定点数 -> 物理量浮点。"""
        max_abs = float(1 << int(log2_max))
        return q_val / 32767.0 * max_abs

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
        从缓冲区尽可能多地切出完整帧：定位帧头 -> 读长度字段 -> 校验帧尾/CRC -> 解码。
        数据不足则保留等待下次；帧头错位或校验失败时丢 1 字节继续找下一个帧头。
        返回本轮解出的 list[McuFeedback]。
        """
        buf = self._buffer
        out = []

        while len(buf) >= TxFrame.FIXED_HEADER_SIZE + TxFrame.TAIL_SIZE:
            hi = buf.find(TxFrame.HEADER)
            if hi == -1:
                buf.clear()
                break
            if hi > 0:
                del buf[:hi]

            if len(buf) < 4:
                break

            n = buf[3]
            if n == 0 or n > TxFrame.MAX_CURVE_COUNT:
                del buf[:1]
                continue

            if len(buf) < TxFrame.FIXED_HEADER_SIZE + n:
                break

            func = buf[2]
            sample_count = buf[9 + n]
            if sample_count == 0:
                del buf[:1]
                continue

            payload_size = n * sample_count * 2
            frame_len = TxFrame.FIXED_HEADER_SIZE + n + payload_size + TxFrame.TAIL_SIZE
            if frame_len > TxFrame.MAX_FRAME_SIZE:
                del buf[:1]
                continue
            if len(buf) < frame_len:
                break

            frame = bytes(buf[:frame_len])

            # 校验帧尾
            if frame[frame_len - 1] != TxFrame.TAIL:
                del buf[:1]
                continue

            # 校验 CRC (覆盖 帧头 .. Payload 结束)
            crc_len = TxFrame.FIXED_HEADER_SIZE + n + payload_size
            calc_crc = self.calculate_crc16(frame, crc_len)
            recv_crc = (frame[frame_len - 2] << 8) | frame[frame_len - 3]
            if calc_crc != recv_crc:
                del buf[:1]
                continue

            feedback = self._decode_frame(frame, n, sample_count, func)
            del buf[:frame_len]
            if feedback is not None:
                out.append(feedback)

        return out

    def _decode_frame(self, frame, n, sample_count, func):
        """
        把一个已通过 CRC 的完整帧解成 McuFeedback。
        只取最后一行采样 (同帧各行一致)，反量化后按 2 电机 × 5 字段拆开。
        MCU ID 无法识别时打印告警并返回 None。
        """
        mcu_id = frame[1]
        try:
            idx = McuConfig.to_index(mcu_id)
        except ValueError:
            print(f"[警告] 未识别的 MCU ID: 0x{mcu_id:02X} (支持 0xB0~0xB7)")
            return None

        frame_counter = frame[4 + n]
        log2max = [frame[4 + i] for i in range(n)]
        timestamp = struct.unpack_from("<I", frame, 5 + n)[0]
        base = TxFrame.INNER_FREQ if func == 0x01 else TxFrame.OUTER_FREQ
        hw_time = timestamp / base

        # 解析最后一行采样 (整行同帧一致)
        offset = TxFrame.FIXED_HEADER_SIZE + n
        row_stride = n * 2
        last_row_offset = offset + (sample_count - 1) * row_stride
        row = []
        for c in range(n):
            (q_val,) = struct.unpack_from("<h", frame, last_row_offset + c * 2)
            row.append(self.dequantize(q_val, log2max[c]))

        motors = self._rows_to_motors(row, n)

        return McuFeedback(
            mcu_index=idx,
            mcu_id=mcu_id,
            func=func,
            curve_count=n,
            sample_count=sample_count,
            frame_counter=frame_counter,
            hw_time=hw_time,
            motors=motors,
        )

    @staticmethod
    def _rows_to_motors(row, curve_count):
        """把一行 curve 值按 2 电机 x 5 字段 拆成 MotorFeedback 列表。"""
        motors = []
        if curve_count != McuConfig.LOW_SPEED_CURVE_COUNT:
            # 曲线布局不是标准低速帧，无法映射电机字段
            for _ in range(McuConfig.MOTORS_PER_MCU):
                motors.append(MotorFeedback())
            return motors

        fields = McuConfig.MOTOR_CURVE_FIELDS
        for m in range(McuConfig.MOTORS_PER_MCU):
            base = m * McuConfig.CURVES_PER_MOTOR
            vals = {}
            for off, name in enumerate(fields):
                ci = base + off
                vals[name] = row[ci] if ci < len(row) else None

            state_val = vals.get("state")
            code = int(round(state_val)) if state_val is not None else None
            motors.append(MotorFeedback(
                state_code=code,
                state_name=MotorState.name(code) or "—",
                theta=vals.get("theta"),
                omega=vals.get("omega"),
                acl=vals.get("acl"),
                torque=vals.get("torque"),
            ))
        return motors
