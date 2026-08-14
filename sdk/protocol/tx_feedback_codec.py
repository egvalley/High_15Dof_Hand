"""
Tx 反馈帧解码器：MCU -> 上位机。字节流拼帧 -> 定帧校验 -> McuFeedback。

帧格式见 wire.TxFrame，曲线的类型与顺序见 topology.CurveLayout —— 本模块只管
"从哪切帧、切下来靠不靠谱"，具体每条曲线是什么类型一概交给布局对象。
"""

import struct

from sdk.protocol.errors import error_names
from sdk.protocol.topology import CurveLayout, McuConfig
from sdk.protocol.wire import TxFrame, TX_ACCEPTED_FUNCS, LOW_SPEED_FREQ
from sdk.models import McuFeedback, MotorFeedback


class TxFeedbackCodec:
    """有状态的字节流 -> 帧 解码器 (内部维护拼帧缓冲区)。每条串口链路持有一个实例。"""

    def __init__(self, curve_count=McuConfig.LOW_SPEED_CURVE_COUNT):
        """
        新建解码器，内部拼帧缓冲区置空。
        参数: curve_count 下位机注册的曲线数量 (帧里无长度字段，靠它定帧长与曲线布局)。
        """
        self.layout = CurveLayout(curve_count)
        self._buffer = bytearray()

    @property
    def curve_count(self):
        """本解码器按几条曲线定帧。"""
        return self.layout.curve_count

    def reset(self):
        """清空拼帧缓冲区。重新连接串口时调用，避免旧的半截字节污染新数据。"""
        self._buffer.clear()

    # ---------------------------------------------------------------- 主接口
    def feed(self, data):
        """
        投喂新到达的原始字节 (bytes / bytearray)，返回本次能完整解析出的 list[McuFeedback]。
        """
        if data:
            self._buffer.extend(data)
        return self._drain()

    # ---------------------------------------------------------------- 内部
    def _drain(self):
        """
        从缓冲区尽可能多地切出完整帧：定位帧头 -> 按布局取定长帧 -> 校验 -> 解码。
        数据不足则保留等待下次；任一校验失败就丢 1 字节继续找下一个帧头
        (CAN FD 帧尾之后的补零字节也在这里被跳过)。
        """
        buf = self._buffer
        frame_len = self.layout.frame_size
        out = []

        while len(buf) >= frame_len:
            head = buf.find(TxFrame.HEADER)
            if head == -1:
                buf.clear()
                break
            if head > 0:
                del buf[:head]
            if len(buf) < frame_len:
                break

            frame = self._decode(buf)
            if frame is None:
                del buf[:1]     # 定帧不成立，退 1 字节找下一个帧头
                continue

            del buf[:frame_len]
            out.append(frame)

        return out

    def _decode(self, buf):
        """
        校验 buf 开头这一帧并解码；不成立返回 None。

        帧里没有 CRC，只能靠这几项定帧：ID 合法 / Func 合法 / 帧尾就位 / 曲线值合理
        (曲线值的合理性由布局按每条曲线的类型判，见 CurveLayout.is_plausible)。
        """
        mcu_id = buf[1]
        if not (McuConfig.BASE_ID <= mcu_id < McuConfig.BASE_ID + McuConfig.COUNT):
            return None

        func = buf[2]
        if func not in TX_ACCEPTED_FUNCS:
            return None

        if buf[self.layout.frame_size - 1] != TxFrame.TAIL:
            return None

        row = self.layout.unpack(buf)
        if not self.layout.is_plausible(row):
            return None

        frame_counter = struct.unpack_from("<I", buf, 3)[0]
        return McuFeedback(
            mcu_index=McuConfig.to_index(mcu_id),
            mcu_id=mcu_id,
            func=func,
            curve_count=self.curve_count,
            frame_counter=frame_counter,
            # 帧里无时间戳；帧计数器每采样点 +1，按低速采样频率折算成秒
            hw_time=frame_counter / LOW_SPEED_FREQ,
            motors=self._to_motors(row),
            curves=list(row),
        )

    def _to_motors(self, row):
        """把一行曲线值拆成每个电机的 MotorFeedback (布局非标准时给一组空反馈)。"""
        motors = []
        for m in range(McuConfig.MOTORS_PER_MCU):
            vals = self.layout.motor_values(row, m)
            if not vals:
                motors.append(MotorFeedback())
                continue

            # 错误字已按 uint32 解出，逐位拆成错误名 —— 可能一个都没有，也可能同时好几个
            word = int(vals["error"])
            motors.append(MotorFeedback(
                error_word=word,
                error_names=error_names(word),
                theta=vals.get("theta"),
                omega=vals.get("omega"),
                acl=vals.get("acl"),
                torque=vals.get("torque"),
            ))
        return motors
