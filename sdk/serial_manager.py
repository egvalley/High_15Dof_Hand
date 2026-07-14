import serial
import threading
import time
import struct


class SerialManager:
    """
    高自由度机械手 上位机串口管理器 (接收与解析部分)
    """
    # ==================== Tx (下位机 -> 上位机) 协议常量 ====================
    TX_FRAME_HEADER = 0x5A
    TX_FRAME_TAIL   = 0x0D
    TX_HEADER_SIZE  = 10
    TX_TAIL_SIZE    = 3
    TX_FUNC_NORMAL  = 0x00
    TX_FUNC_HIGH    = 0x01
    TX_MAX_CURVE    = 64
    TX_MAX_FRAME    = 1024

    INNER_FREQ = 10000.0
    OUTER_FREQ = 200.0

    # ==================== MCU 列表 ====================
    NUM_MCU     = 8
    MCU_BASE_ID = 0xB0
    MCU_IDS     = list(range(0xB0, 0xB0 + 8))

    # ==================== 低速曲线布局 ====================
    MOTORS_PER_MCU        = 2
    CURVES_PER_MOTOR      = 5
    LOW_SPEED_CURVE_COUNT = MOTORS_PER_MCU * CURVES_PER_MOTOR
    MOTOR_CURVE_FIELDS = ["state", "theta", "omega", "acl", "torque"]

    # ==================== 电机状态枚举 ====================
    MOTOR_STATE = {
        "StartupError": 1, "StartupReady": 2, "StartupParamsInit": 3,
        "StartupCurrentCalib": 4, "StartupPhaseDiag": 5, "StartupElecAngleDrag": 6,
        "StartupElecAngleDone": 7, "StartupDisable": 20,
        "DbgCurrentError": 21, "DbgCurrentAlphaBeta": 22, "DbgCurrentOpenLoop": 23,
        "DbgCurrentOnlyIdClosedLoop": 24, "DbgCurrentClosedLoop": 25,
        "DbgCurrentClosedLoop_IqSysIden": 26, "DbgCurrentClosedLoop_IdSysIden": 27,
        "DbgCurrentOpenLoop_UqSysIden": 28, "DbgCurrentOpenLoop_UdSysIden": 29,
        "DbgCurrentDisable": 40,
        "DbgVelocityError": 41, "DbgVelocityClosedLoop_SysIden": 42, "DbgVelocityDisable": 60,
        "DbgPositionError": 61, "DbgPositionClosedLoop_SysIden": 62, "DbgPositionDisable": 80,
        "AppError": 81, "AppCurrentCtrl": 82, "AppTorqueCtrl": 83, "AppImpedanceCtrl": 84,
        "AppVelocityCtrl": 85, "AppPositionCtrl": 86, "AppDisable": 100,
        "InnerOuterMismatch": 255,
    }
    MOTOR_STATE_NAME = {v: k for k, v in MOTOR_STATE.items()}

    def __init__(self, port, baudrate=115200, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.ser = None
        self.connecting_state = False
        self._stop_event = threading.Event()
        self._read_thread = None

        self.serial_buffer = bytearray()
        self.mcu_frames = [None] * self.NUM_MCU
        self.mcu_stats = [self._new_stats() for _ in range(self.NUM_MCU)]
        self.last_loss_calc_time = time.time()

    @staticmethod
    def _new_stats():
        return {
            "expected_counter": 0,
            "received": 0,
            "lost": 0,
            "loss_rate": 0.0,
            "last_sec_received": 0,
            "last_sec_lost": 0,
            "ever_received": False,
        }

    def connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
            )
            self.connecting_state = True
            self.serial_buffer.clear()
            self.reset_stats()

            self._stop_event.clear()
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()

            print(f"[连接成功] 串口: {self.port}, 波特率: {self.baudrate}")
            return True
        except serial.SerialException as e:
            print(f"[连接失败] 无法打开串口 {self.port}: {e}")
            self.connecting_state = False
            return False

    def disconnect(self):
        self.connecting_state = False
        self._stop_event.set()
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"[断开连接] 串口 {self.port} 已关闭。")

    def reset_stats(self):
        for i in range(self.NUM_MCU):
            self.mcu_stats[i] = self._new_stats()
            self.mcu_frames[i] = None

    @staticmethod
    def _calculate_crc16(data, length):
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
    def _dequantize(q_val, log2_max):
        max_abs = float(1 << int(log2_max))
        return q_val / 32767.0 * max_abs

    def _read_loop(self):
        while not self._stop_event.is_set():
            if not (self.ser and self.ser.is_open):
                time.sleep(0.1)
                continue
            try:
                now = time.time()
                if now - self.last_loss_calc_time >= 1.0:
                    for st in self.mcu_stats:
                        recv, lost = st["received"], st["lost"]
                        total = recv + lost
                        st["last_sec_received"] = recv
                        st["last_sec_lost"] = lost
                        st["loss_rate"] = (lost / total * 100.0) if total > 0 else 0.0
                        st["received"] = 0
                        st["lost"] = 0
                    self.last_loss_calc_time = now

                in_waiting = self.ser.in_waiting
                if in_waiting > 0:
                    self.serial_buffer.extend(self.ser.read(in_waiting))
                else:
                    time.sleep(0.005)
                    continue

                self._parse_buffer()
            except serial.SerialException:
                print("\n[错误] 串口异常，连接可能已断开。")
                self.disconnect()
                break
            except Exception as e:
                print(f"\n[解析异常]: {e}")

    def _parse_buffer(self):
        buf = self.serial_buffer
        while len(buf) >= self.TX_HEADER_SIZE + self.TX_TAIL_SIZE:
            hi = buf.find(self.TX_FRAME_HEADER)
            if hi == -1:
                buf.clear()
                return
            if hi > 0:
                del buf[:hi]

            if len(buf) < 4:
                return

            n = buf[3]
            if n == 0 or n > self.TX_MAX_CURVE:
                del buf[:1]
                continue

            if len(buf) < self.TX_HEADER_SIZE + n:
                return

            func = buf[2]
            sample_count = buf[9 + n]
            if sample_count == 0:
                del buf[:1]
                continue

            payload_size = n * sample_count * 2
            frame_len = self.TX_HEADER_SIZE + n + payload_size + self.TX_TAIL_SIZE
            if frame_len > self.TX_MAX_FRAME:
                del buf[:1]
                continue
            if len(buf) < frame_len:
                return

            frame = bytes(buf[:frame_len])

            if frame[frame_len - 1] != self.TX_FRAME_TAIL:
                del buf[:1]
                continue

            crc_len = self.TX_HEADER_SIZE + n + payload_size
            calc_crc = self._calculate_crc16(frame, crc_len)
            recv_crc = (frame[frame_len - 2] << 8) | frame[frame_len - 3]
            if calc_crc != recv_crc:
                del buf[:1]
                continue

            self._handle_frame(frame, n, sample_count, func)
            del buf[:frame_len]

    def _handle_frame(self, frame, n, sample_count, func):
        mcu_id = frame[1]
        idx = mcu_id - self.MCU_BASE_ID
        if idx < 0 or idx >= self.NUM_MCU:
            print(f"[警告] 未识别的 MCU ID: 0x{mcu_id:02X} (支持 0xB0~0xB7)")
            return

        frame_counter = frame[4 + n]
        log2max = [frame[4 + i] for i in range(n)]
        timestamp = struct.unpack_from("<I", frame, 5 + n)[0]
        base = self.INNER_FREQ if func == self.TX_FUNC_HIGH else self.OUTER_FREQ
        hw_time = timestamp / base

        offset = self.TX_HEADER_SIZE + n
        samples = []
        for _ in range(sample_count):
            row = []
            for c in range(n):
                (q_val,) = struct.unpack_from("<h", frame, offset)
                row.append(self._dequantize(q_val, log2max[c]))
                offset += 2
            samples.append(row)

        parsed = {
            "mcu_id": mcu_id,
            "mcu_index": idx,
            "func": func,
            "curve_count": n,
            "sample_count": sample_count,
            "frame_counter": frame_counter,
            "hw_time": hw_time,
            "log2_max": log2max,
            "samples": samples,
        }
        self.mcu_frames[idx] = parsed
        self._update_stats(idx, frame_counter)

    def _update_stats(self, idx, frame_counter):
        st = self.mcu_stats[idx]
        st["received"] += 1
        if st["ever_received"]:
            expected = st["expected_counter"]
            if frame_counter != expected:
                lost = (frame_counter - expected) & 0xFF
                if 0 < lost < 255:
                    st["lost"] += lost
        st["ever_received"] = True
        st["expected_counter"] = (frame_counter + 1) & 0xFF

    @staticmethod
    def _to_index(mcu):
        if 0 <= mcu < SerialManager.NUM_MCU:
            return mcu
        if SerialManager.MCU_BASE_ID <= mcu < SerialManager.MCU_BASE_ID + SerialManager.NUM_MCU:
            return mcu - SerialManager.MCU_BASE_ID
        raise ValueError(f"无效的 MCU 标识: {mcu!r} (应为 index 0~7 或 ID 0xB0~0xB7)")

    def get_latest_frame(self, mcu):
        return self.mcu_frames[self._to_index(mcu)]

    def motor_fields_from_frame(self, frame, motor_index):
        if not frame or not frame["samples"]:
            return None
        if frame["curve_count"] != self.LOW_SPEED_CURVE_COUNT:
            return None
        if not (0 <= motor_index < self.MOTORS_PER_MCU):
            return None
        row = frame["samples"][-1]
        base = motor_index * self.CURVES_PER_MOTOR
        out = {}
        for off, field in enumerate(self.MOTOR_CURVE_FIELDS):
            ci = base + off
            out[field] = row[ci] if ci < len(row) else None
        state_val = out.get("state")
        code = int(round(state_val)) if state_val is not None else None
        out["state_code"] = code
        out["state_name"] = self.MOTOR_STATE_NAME.get(code, f"Unknown({code})") if code is not None else None
        return out

    def get_mcu_stats(self, mcu):
        return self.mcu_stats[self._to_index(mcu)]

    def write_data(self, data):
        """供发送模块调用的底层写入接口"""
        if not self.is_connected or not self.ser:
            print("[发送失败] 串口未连接")
            return False
        try:
            self.ser.write(data)
            return True
        except Exception as e:
            print(f"[发送错误]: {e}")
            return False