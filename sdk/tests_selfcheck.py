"""快速自测：验证与 router.h/router.c 一致的编码、解码、mode 映射、未实现拒绝。"""

import struct

from sdk.protocol.constants import (
    CommFunc, McuConfig, MotorFunc, mode_for, scale_of, TxFrame,
)
from sdk.protocol.rx_command_codec import RxCommandCodec
from sdk.protocol.tx_feedback_codec import TxFeedbackCodec
from sdk.models import MotorCommand, MotorTarget
from sdk.serial_commander import SerialCommander, UnsupportedCommand


# ---- 用一个假的 SerialManager 记录最后一帧，避免真实串口 ----
class FakeSM:
    def __init__(self):
        self.last = None
        self.is_connected = True
        self.ser = True
    def write_data(self, data):
        self.last = data
        return True


def test_mode_for_matches_header():
    assert mode_for(MotorFunc.POSITION, 0) == 1
    assert mode_for(MotorFunc.POSITION, 1) == 11
    assert mode_for(MotorFunc.VELOCITY, 0) == 2
    assert mode_for(MotorFunc.TORQUE, 1) == 13
    assert mode_for(MotorFunc.IMPEDANCE_INERTIA, 0) == 6
    assert mode_for(MotorFunc.MODE_SELECT, 0) == 20
    assert mode_for(MotorFunc.MODE_SELECT, 1) == 40
    assert mode_for(MotorFunc.SWITCH_ID, 1) == 17
    print("OK  test_mode_for_matches_header")


def test_scales():
    assert scale_of(MotorFunc.POSITION) == 100.0
    assert scale_of(MotorFunc.VELOCITY) == 1000.0
    assert scale_of(MotorFunc.TORQUE) == 1.0
    assert scale_of(MotorFunc.IMPEDANCE_SPRING) == 100.0
    print("OK  test_scales")


def test_position_both_motors():
    sm = FakeSM()
    cmd = SerialCommander(sm, func=CommFunc.RX_USB_USART_COMMAND, pad_canfd=False)
    ok = cmd.send_position(0xB0, 1.0, 2.0)   # M0=100(0x64), M1=200(0xC8)
    assert ok
    expected = bytes([
        0x3B, 0xB0, 0x00, 0x03, 0x02,
        0x01, 0x64, 0x00,   # mode=Position_M0=1, 100
        0x0B, 0xC8, 0x00,   # mode=Position_M1=11, 200
        0x1E,
    ])
    assert sm.last == expected, sm.last.hex(" ")
    print("OK  test_position_both_motors (12 字节)")


def test_position_single_motor_odd_ok():
    sm = FakeSM()
    cmd = SerialCommander(sm, func=CommFunc.RX_USB_USART_COMMAND, pad_canfd=False)
    cmd.send_position(0, 1.0, None)          # 仅电机0 -> command_num=1 (奇数, 固件可处理)
    assert sm.last[4] == 1
    assert sm.last[5] == 0x01                # Position_M0
    assert len(sm.last) == 9
    print("OK  test_position_single_motor_odd_ok")


def test_velocity_scale():
    sm = FakeSM()
    cmd = SerialCommander(sm, func=CommFunc.RX_USB_USART_COMMAND, pad_canfd=False)
    cmd.send_velocity(0, 1.0, None)          # 1.0 rad/s ×1000 = 1000 = 0x03E8
    p = struct.unpack_from("<h", sm.last, 6)[0]
    assert p == 1000, p
    assert sm.last[5] == 0x02                # Velocity_M0
    print("OK  test_velocity_scale")


def test_impedance_params_both():
    sm = FakeSM()
    cmd = SerialCommander(sm, func=CommFunc.RX_USB_USART_COMMAND, pad_canfd=False)
    cmd.send_impedance_params(0, spring=1.0, damper=2.0, inertia=3.0, motor=MotorTarget.BOTH)
    modes = [sm.last[5 + i * 3] for i in range(sm.last[4])]
    assert modes == [4, 5, 6, 14, 15, 16], modes   # M0 spring/damp/inertia, M1 同
    print("OK  test_impedance_params_both")


def test_state_dispatch():
    sm = FakeSM()
    cmd = SerialCommander(sm, func=CommFunc.RX_USB_USART_COMMAND, pad_canfd=False)
    cmd.send_state(0, "StartupReady", motor=MotorTarget.BOTH)
    assert sm.last[5] == 20 and sm.last[8] == 40   # Mode_Select M0/M1
    assert struct.unpack_from("<h", sm.last, 6)[0] == 2   # StartupReady=2
    print("OK  test_state_dispatch")


def test_canfd_pad():
    sm = FakeSM()
    cmd = SerialCommander(sm, func=CommFunc.RX_FDCAN_COMMAND, pad_canfd=True)
    cmd.send_impedance_params(0, 1, 2, 3, motor=MotorTarget.BOTH)  # 6 命令 -> 24 字节
    assert len(sm.last) == 24, len(sm.last)
    print("OK  test_canfd_pad (24)")


def test_unsupported_raises():
    sm = FakeSM()
    cmd = SerialCommander(sm)
    for fn in ("send_iq", "send_pos_pid", "send_trajectory", "send_homing", "send_sysiden"):
        try:
            getattr(cmd, fn)(0, 0, 0) if fn.endswith("pid") else getattr(cmd, fn)(0)
        except UnsupportedCommand:
            pass
        except TypeError:
            # 某些签名需要不同参数，再试通用调用
            try:
                getattr(cmd, fn)(0, 1, 2, 3)
            except UnsupportedCommand:
                pass
        else:
            raise AssertionError(f"{fn} 应抛 UnsupportedCommand")
    print("OK  test_unsupported_raises")


# ---- Tx 反馈解码 (与之前一致) ----
def _build_tx_low_speed_frame(mcu_id, counter, values10):
    n = 10
    log2 = [8] * n
    body = bytearray()
    body.append(TxFrame.HEADER)
    body.append(mcu_id)
    body.append(CommFunc.TX_NORMAL)
    body.append(n)
    body += bytes(log2)
    body.append(counter)
    body += struct.pack("<I", 1234)
    body.append(1)
    for v in values10:
        q = int(round(v / 256.0 * 32767.0))
        body += struct.pack("<h", q)
    crc = TxFeedbackCodec.calculate_crc16(bytes(body), len(body))
    body += struct.pack("<H", crc)
    body.append(TxFrame.TAIL)
    return bytes(body)


def test_tx_decode_roundtrip():
    codec = TxFeedbackCodec()
    vals = [2, 1.0, -0.5, 0.0, 0.2, 100, -1.0, 0.5, 0.1, -0.2]
    raw = _build_tx_low_speed_frame(0xB2, 7, vals)
    frames = codec.feed(raw)
    assert len(frames) == 1
    fb = frames[0]
    assert fb.mcu_index == 2 and fb.frame_counter == 7
    assert fb.motors[0].state_name == "StartupReady"
    assert fb.motors[1].state_name == "AppDisable"
    print("OK  test_tx_decode_roundtrip")


if __name__ == "__main__":
    test_mode_for_matches_header()
    test_scales()
    test_position_both_motors()
    test_position_single_motor_odd_ok()
    test_velocity_scale()
    test_impedance_params_both()
    test_state_dispatch()
    test_canfd_pad()
    test_unsupported_raises()
    test_tx_decode_roundtrip()
    print("\n全部通过 ✅")
