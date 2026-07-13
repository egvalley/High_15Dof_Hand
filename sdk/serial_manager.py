import serial
import threading
import queue
import time
import struct


class SerialManager:
    """
    高自由度机械手 上位机串口管理器

    职责：
      1. 解析下位机(MCU)上报的高速/低速曲线数据帧 (Tx 协议, 帧头 0x5A)
      2. 组装并下发控制指令帧 (Rx 协议, 帧头 0x3B, Func=0x03)

    协议与 router.c / router.h / motor_state_machine.h 严格对应：

    Tx 帧 (MCU -> PC)：
      [0]        帧头 0x5A
      [1]        源 ID (0xB0~0xB7)
      [2]        Func (0=低速 Tx_Normal, 1=高速 Tx_High)
      [3]        曲线数量 N
      [4 .. 4+N-1]   每条曲线的 log2(max_abs)  (定点反量化用)
      [4+N]      帧计数器 (0~255 循环, 用于丢包统计)
      [5+N..8+N] 时间戳 (4 字节, 小端, 单位: 低速 ms / 高速 内环 tick)
      [9+N]      每条曲线的采样点数 M
      [10+N .. ] Payload: N*M 个 int16 (小端), 顺序为 采样点0的N条曲线, 采样点1的N条曲线...
      [.. ]      CRC16 (2 字节, 小端, Modbus 多项式 0xA001, 覆盖 [0 .. 10+N+payload-1])
      [末]       帧尾 0x0D
      反量化: val = q_val / 32767 * (2 ^ log2_max)

    Rx 帧 (PC -> MCU)：
      [0]        帧头 0x3B
      [1]        目标 ID (0xB0~0xB7)
      [2]        帧计数器 (暂不使用)
      [3]        Func: 0x03=USB/USART 直连指令; 0x04=FDCAN 转发指令 (默认, 8 个 MCU 都在 CAN 总线上)
      [4]        命令数量 C
      [5+i*3]    Control_Mode
      [6+i*3 .. 7+i*3]  Command (int16, 小端)
      [末]       帧尾 0x1E
      走 FDCAN 通道时整帧长度须对齐到 CAN FD 合法长度(8/12/16/24/...)，不足部分在帧尾之后补零，
      下位机解析时会忽略帧尾之后的填充字节。
      下位机把 前 C/2 条指令派发给 电机0(app1), 后 C/2 条派发给 电机1(app2)，故 C 必须为偶数。
      线上定点值 = round(物理值 * scale)，见 MODE_SCALE。
    """

    # ==================== Tx (下位机 -> 上位机) 协议常量 ====================
    TX_FRAME_HEADER = 0x5A
    TX_FRAME_TAIL   = 0x0D
    TX_HEADER_SIZE  = 10     # 固定协议头字节数 (不含 N 个 log2max 与 Payload)
    TX_TAIL_SIZE    = 3      # CRC16(2) + 帧尾(1)
    TX_FUNC_NORMAL  = 0x00   # Router_Comm_USB_USART_Tx_Normal (低速)
    TX_FUNC_HIGH    = 0x01   # Router_Comm_USB_USART_Tx_High   (高速)
    TX_MAX_CURVE    = 64     # 曲线数量的合理上限 (防脏数据), router.h 低速上限为 20
    TX_MAX_FRAME    = 1024   # ROUTER_MAX_TX_FRAME_SIZE

    INNER_FREQ = 10000.0     # 高速时间戳基准 (内环 10kHz)
    OUTER_FREQ = 1000.0      # 低速时间戳基准 (osKernelGetTickCount, ms => 1kHz)

    # ==================== Rx (上位机 -> 下位机) 协议常量 ====================
    RX_FRAME_HEADER = 0x3B
    RX_FRAME_TAIL   = 0x1E
    # 指令通道 Func 字节 (对应 router.h RouterRxCommFunc)
    RX_FUNC_USB_USART_CMD = 0x03   # Router_Comm_USB_USART_Cmd (直连 USB/USART 的设备)
    RX_FUNC_FDCAN_CMD     = 0x04   # Router_Comm_FDCAN_Cmd     (经桥接转发到 FDCAN 总线上的 MCU)
    # 8 个电机 MCU 都挂在 FDCAN 总线上, 故指令默认走 FDCAN 通道
    RX_FUNC_CMD           = RX_FUNC_FDCAN_CMD
    # CAN FD 合法帧长(字节): 走 FDCAN 时整帧长度必须对齐到其中之一 (router.c: "需要确保fdcan长度符合")
    CANFD_VALID_SIZES     = (8, 12, 16, 20, 24, 32, 48, 64)

    # ==================== MCU 列表 ====================
    NUM_MCU     = 8
    MCU_BASE_ID = 0xB0
    MCU_IDS     = list(range(0xB0, 0xB0 + 8))  # index 0..7 <-> 0xB0..0xB7

    # ==================== 低速曲线布局 (对应 router.c DeviceRouter_Init 注册顺序) ====================
    MOTORS_PER_MCU        = 2
    CURVES_PER_MOTOR      = 5
    LOW_SPEED_CURVE_COUNT = MOTORS_PER_MCU * CURVES_PER_MOTOR  # 10
    # 每个电机 5 条曲线, 第 1 条为状态机; 顺序即 router.c 注册顺序
    MOTOR_CURVE_FIELDS = ["state", "theta", "omega", "acl", "torque"]

    # ==================== 控制模式枚举 (对应 router.h RouterControlMode) ====================
    MODE_FOC_UQ                      = 2
    MODE_FOC_UD                      = 3
    MODE_FOC_UALPHA                  = 4
    MODE_FOC_UBETA                   = 5
    MODE_FOC_THETA_GEAR              = 6
    MODE_FOC_OMEGA_GEAR              = 7
    MODE_FOC_TORQUE_GEAR             = 8
    MODE_FOC_IQ                      = 9
    MODE_FOC_ID                      = 10
    MODE_FOC_IMPEDANCE_SPRING        = 32
    MODE_FOC_IMPEDANCE_DAMPER        = 33
    MODE_FOC_IMPEDANCE_INERTIA       = 34
    MODE_FOC_IMPEDANCE_SPRING_ORIGIN = 35
    MODE_FOC_CUR_PID_KP              = 52
    MODE_FOC_CUR_PID_KI              = 53
    MODE_FOC_VEL_PID_KP              = 54
    MODE_FOC_VEL_PID_KI              = 55
    MODE_FOC_POS_PID_KP              = 56
    MODE_FOC_POS_PID_KI              = 57
    MODE_FOC_DISPATCH_STATE          = 72
    MODE_APP_TRAJ_INIT               = 82
    MODE_APP_TRAJ_DEINIT             = 83
    MODE_APP_TRAJ_VEL_MAX            = 84
    MODE_APP_TRAJ_ACL_MAX            = 85
    MODE_APP_TRAJ_POS_CMD            = 86
    MODE_APP_HOMING                  = 102
    MODE_APP_FLASHING_PARAMS         = 103
    MODE_APP_SYSIDEN                 = 104
    MODE_DEVICE_CLEAR_FLASH_ERROR    = 122

    # 模式 -> 可读名字 (用于日志)
    MODE_NAME = {
        MODE_FOC_UQ: "FOC_Uq", MODE_FOC_UD: "FOC_Ud",
        MODE_FOC_UALPHA: "FOC_Ualpha", MODE_FOC_UBETA: "FOC_Ubeta",
        MODE_FOC_THETA_GEAR: "Theta_Gear", MODE_FOC_OMEGA_GEAR: "Omega_Gear",
        MODE_FOC_TORQUE_GEAR: "Torque_Gear", MODE_FOC_IQ: "Iq", MODE_FOC_ID: "Id",
        MODE_FOC_IMPEDANCE_SPRING: "Impedance_Spring",
        MODE_FOC_IMPEDANCE_DAMPER: "Impedance_Damper",
        MODE_FOC_IMPEDANCE_INERTIA: "Impedance_Inertia",
        MODE_FOC_IMPEDANCE_SPRING_ORIGIN: "Impedance_Spring_Origin",
        MODE_FOC_CUR_PID_KP: "Cur_Pid_Kp", MODE_FOC_CUR_PID_KI: "Cur_Pid_Ki",
        MODE_FOC_VEL_PID_KP: "Vel_Pid_Kp", MODE_FOC_VEL_PID_KI: "Vel_Pid_Ki",
        MODE_FOC_POS_PID_KP: "Pos_Pid_Kp", MODE_FOC_POS_PID_KI: "Pos_Pid_Ki",
        MODE_FOC_DISPATCH_STATE: "DispatchStateMachine",
        MODE_APP_TRAJ_INIT: "Traj_Init", MODE_APP_TRAJ_DEINIT: "Traj_DeInit",
        MODE_APP_TRAJ_VEL_MAX: "Traj_Vel_Max", MODE_APP_TRAJ_ACL_MAX: "Traj_Acl_Max",
        MODE_APP_TRAJ_POS_CMD: "Traj_Pos_Cmd", MODE_APP_HOMING: "Homing",
        MODE_APP_FLASHING_PARAMS: "FlashingParams", MODE_APP_SYSIDEN: "SysIden",
        MODE_DEVICE_CLEAR_FLASH_ERROR: "ClearFlashError",
    }

    # 模式 -> 定点换算系数: 线上 int16 = round(物理值 * scale)
    # 换算关系全部由 router.c 的 RxPayloadHandler 反推得到。
    # 物理值单位: 位置/速度/加速度/力矩/弹簧原点 = 减速箱输出端 (rad, rad/s, rad/s^2, N·m);
    #             电流 A; 阻抗系数/PID增益 无量纲; 派发状态机 = MotorState 枚举值。
    MODE_SCALE = {
        MODE_FOC_THETA_GEAR:              100.0,
        MODE_FOC_OMEGA_GEAR:              100.0,
        MODE_FOC_TORQUE_GEAR:             100.0,
        MODE_FOC_IQ:                      1000.0,
        MODE_FOC_ID:                      1000.0,
        MODE_FOC_IMPEDANCE_SPRING:        10.0,
        MODE_FOC_IMPEDANCE_DAMPER:        100.0,
        MODE_FOC_IMPEDANCE_INERTIA:       1000.0,
        MODE_FOC_IMPEDANCE_SPRING_ORIGIN: 100.0,
        MODE_FOC_CUR_PID_KP:              1000.0,
        MODE_FOC_CUR_PID_KI:              1000.0,
        MODE_FOC_VEL_PID_KP:              1000.0,
        MODE_FOC_VEL_PID_KI:              1000.0,
        MODE_FOC_POS_PID_KP:              1.0,
        MODE_FOC_POS_PID_KI:              1.0,
        MODE_APP_TRAJ_VEL_MAX:            100.0,
        MODE_APP_TRAJ_ACL_MAX:            100.0,
        MODE_APP_TRAJ_POS_CMD:            100.0,
        MODE_FOC_DISPATCH_STATE:          1.0,
    }

    # ==================== 电机状态枚举 (对应 motor_state_machine.h MotorStateType) ====================
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

    # ==============================================================================================
    def __init__(self, port, baudrate=115200, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.ser = None
        self.is_connected = False
        self._stop_event = threading.Event()
        self._read_thread = None

        # 接收缓冲区
        self.serial_buffer = bytearray()

        # 每个 MCU 的最新解析帧 (整块替换, 读取无需加锁)
        self.mcu_frames = [None] * self.NUM_MCU

        # 每个 MCU 的丢包/帧率统计
        self.mcu_stats = [self._new_stats() for _ in range(self.NUM_MCU)]
        self.last_loss_calc_time = time.time()

        # 未识别 ID 的节流告警
        self._unknown_ids = set()

    @staticmethod
    def _new_stats():
        return {
            "expected_counter": 0,   # 期望的下一帧计数器
            "received": 0,           # 当前1秒窗口内收到的帧
            "lost": 0,               # 当前1秒窗口内丢失的帧
            "loss_rate": 0.0,        # 上一秒的丢包率(%)
            "last_sec_received": 0,  # 上一秒收到帧数(供显示)
            "last_sec_lost": 0,      # 上一秒丢失帧数(供显示)
            "ever_received": False,  # 是否曾经收到过数据
        }

    # ==================== 连接管理 ====================
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
            self.is_connected = True
            self.serial_buffer.clear()
            self.reset_stats()

            self._stop_event.clear()
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()

            print(f"[连接成功] 串口: {self.port}, 波特率: {self.baudrate}")
            return True
        except serial.SerialException as e:
            print(f"[连接失败] 无法打开串口 {self.port}: {e}")
            self.is_connected = False
            return False

    def disconnect(self):
        self.is_connected = False
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

    # ==================== CRC / 反量化 ====================
    @staticmethod
    def _calculate_crc16(data, length):
        """Modbus CRC16 (多项式 0xA001)"""
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
        """反量化: val = q / 32767 * 2^log2_max"""
        max_abs = float(1 << int(log2_max))
        return q_val / 32767.0 * max_abs

    # ==================== 读取线程 ====================
    def _read_loop(self):
        while not self._stop_event.is_set():
            if not (self.ser and self.ser.is_open):
                time.sleep(0.1)
                continue
            try:
                # 1) 周期性(1s)结算丢包率
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

                # 2) 读取全部可用数据
                in_waiting = self.ser.in_waiting
                if in_waiting > 0:
                    self.serial_buffer.extend(self.ser.read(in_waiting))
                else:
                    time.sleep(0.005)
                    continue

                # 3) 解析
                self._parse_buffer()
            except serial.SerialException:
                print("\n[错误] 串口异常，连接可能已断开。")
                self.disconnect()
                break
            except Exception as e:
                print(f"\n[解析异常]: {e}")

    def _parse_buffer(self):
        buf = self.serial_buffer
        # 最短可能帧: 固定头(10) + N(>=1) + 尾(3), 这里用固定头+尾作粗门槛
        while len(buf) >= self.TX_HEADER_SIZE + self.TX_TAIL_SIZE:
            # 找帧头
            hi = buf.find(self.TX_FRAME_HEADER)
            if hi == -1:
                buf.clear()
                return
            if hi > 0:
                del buf[:hi]

            if len(buf) < 4:
                return  # 等待更多数据以读取曲线数量 N

            n = buf[3]  # 曲线数量
            if n == 0 or n > self.TX_MAX_CURVE:
                del buf[:1]
                continue

            # 需要读取采样点数 [9+N] => 至少 10+N 字节
            if len(buf) < self.TX_HEADER_SIZE + n:
                return

            func = buf[2]
            sample_count = buf[9 + n]
            if sample_count == 0:
                del buf[:1]
                continue

            payload_size = n * sample_count * 2  # sizeof(int16)
            frame_len = self.TX_HEADER_SIZE + n + payload_size + self.TX_TAIL_SIZE
            if frame_len > self.TX_MAX_FRAME:
                del buf[:1]
                continue
            if len(buf) < frame_len:
                return  # 等待整帧

            frame = bytes(buf[:frame_len])

            # 校验帧尾
            if frame[frame_len - 1] != self.TX_FRAME_TAIL:
                del buf[:1]
                continue

            # 校验 CRC (覆盖 帧头..Payload)
            crc_len = self.TX_HEADER_SIZE + n + payload_size  # == frame_len - 3
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
            if mcu_id not in self._unknown_ids:
                self._unknown_ids.add(mcu_id)
                print(f"[警告] 未识别的 MCU ID: 0x{mcu_id:02X} (支持 0xB0~0xB7)")
            return

        frame_counter = frame[4 + n]
        log2max = [frame[4 + i] for i in range(n)]
        timestamp = struct.unpack_from("<I", frame, 5 + n)[0]
        base = self.INNER_FREQ if func == self.TX_FUNC_HIGH else self.OUTER_FREQ
        hw_time = timestamp / base

        offset = self.TX_HEADER_SIZE + n  # 10 + N, payload 起点
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
                lost = (frame_counter - expected) & 0xFF  # 计数器 0~255 循环
                if 0 < lost < 255:
                    st["lost"] += lost
        st["ever_received"] = True
        st["expected_counter"] = (frame_counter + 1) & 0xFF

    # ==================== 数据获取接口 ====================
    @staticmethod
    def _to_index(mcu):
        """接受 index(0~7) 或 原始 ID(0xB0~0xB7)，统一返回 index。"""
        if 0 <= mcu < SerialManager.NUM_MCU:
            return mcu
        if SerialManager.MCU_BASE_ID <= mcu < SerialManager.MCU_BASE_ID + SerialManager.NUM_MCU:
            return mcu - SerialManager.MCU_BASE_ID
        raise ValueError(f"无效的 MCU 标识: {mcu!r} (应为 index 0~7 或 ID 0xB0~0xB7)")

    def get_latest_frame(self, mcu):
        return self.mcu_frames[self._to_index(mcu)]

    def get_latest_sample(self, mcu):
        """返回该 MCU 最新一帧的最后一个采样点(所有曲线值列表)，无数据返回 None。"""
        frame = self.mcu_frames[self._to_index(mcu)]
        if frame and frame["samples"]:
            return frame["samples"][-1]
        return None

    def get_curve_value(self, mcu, curve_index):
        sample = self.get_latest_sample(mcu)
        if sample is not None and 0 <= curve_index < len(sample):
            return sample[curve_index]
        return None

    def get_motor_value(self, mcu, motor_index, field):
        """
        获取某 MCU 某电机(0/1)的指定字段(state/theta/omega/acl/torque)。
        仅当帧为标准低速 10 曲线布局时有效, 否则返回 None。
        """
        idx = self._to_index(mcu)
        frame = self.mcu_frames[idx]
        if not frame or not frame["samples"]:
            return None
        if frame["curve_count"] != self.LOW_SPEED_CURVE_COUNT:
            return None
        if field not in self.MOTOR_CURVE_FIELDS:
            return None
        if not (0 <= motor_index < self.MOTORS_PER_MCU):
            return None
        curve_index = motor_index * self.CURVES_PER_MOTOR + self.MOTOR_CURVE_FIELDS.index(field)
        row = frame["samples"][-1]
        if curve_index < len(row):
            return row[curve_index]
        return None

    def get_motor_state(self, mcu, motor_index):
        """返回 (状态码, 状态名)。无数据返回 (None, None)。"""
        val = self.get_motor_value(mcu, motor_index, "state")
        if val is None:
            return None, None
        code = int(round(val))
        return code, self.MOTOR_STATE_NAME.get(code, f"Unknown({code})")

    def motor_fields_from_frame(self, frame, motor_index):
        """
        从单个帧快照中提取某电机(0/1)的全部字段, 保证是同一帧的一致数据。
        返回 dict(state/theta/omega/acl/torque + state_code/state_name) 或 None。
        用法: GUI 每个 MCU 只调一次 get_latest_frame() 取快照, 再对两个电机各调用本函数,
              避免逐字段读取时被后台读线程换帧, 导致一行里混入两帧数据。
        """
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

    def get_all_stats(self):
        return self.mcu_stats

    def has_data(self, mcu):
        return self.mcu_frames[self._to_index(mcu)] is not None

    # ==================== 指令下发 ====================
    def _write_frame(self, data):
        if not self.is_connected or not self.ser:
            print("[发送失败] 串口未连接")
            return False
        try:
            self.ser.write(data)
            return True
        except Exception as e:
            print(f"[发送错误]: {e}")
            return False

    @staticmethod
    def _clamp_int16(v):
        v = int(round(v))
        return max(-32768, min(32767, v))

    @classmethod
    def _canfd_pad(cls, frame):
        """把帧长补零到最近的 CAN FD 合法长度; 填充字节位于帧尾之后, 下位机解析时忽略。"""
        n = len(frame)
        for s in cls.CANFD_VALID_SIZES:
            if n <= s:
                return frame + bytes(s - n)
        return frame  # >64 不处理(命令帧不会到这么长)

    def _quantize(self, mode, value):
        """按 MODE_SCALE 把物理值换算为线上 int16 定点值。"""
        scale = self.MODE_SCALE.get(mode, 1.0)
        return self._clamp_int16(value * scale)

    def build_command_frame(self, mcu, motor0_cmds, motor1_cmds, counter=0, func=None, pad_canfd=None):
        """
        组装 Rx 指令帧 (不发送)，返回 bytes。
        motor0_cmds / motor1_cmds: [(mode:int, param_int16:int), ...] —— 已是线上定点整数。
        两个列表长度必须相等 (下位机按 命令数/2 拆分给电机0/电机1)。
        func:      指令通道 Func 字节, 默认 RX_FUNC_CMD (=FDCAN)。
        pad_canfd: 是否把整帧补零对齐到 CAN FD 合法长度; None 时在 func 为 FDCAN 通道时自动开启。
        """
        if len(motor0_cmds) != len(motor1_cmds):
            raise ValueError(f"电机0/电机1 指令数必须相等: {len(motor0_cmds)} vs {len(motor1_cmds)}")
        if func is None:
            func = self.RX_FUNC_CMD
        if pad_canfd is None:
            pad_canfd = (func == self.RX_FUNC_FDCAN_CMD)
        target_id = self.MCU_IDS[self._to_index(mcu)]
        all_cmds = list(motor0_cmds) + list(motor1_cmds)
        command_num = len(all_cmds)
        if command_num == 0:
            raise ValueError("指令列表为空")
        if command_num > 20:
            raise ValueError(f"命令数量超出下位机上限(20): {command_num}")

        data = bytearray()
        data.append(self.RX_FRAME_HEADER)
        data.append(target_id)
        data.append(counter & 0xFF)
        data.append(func & 0xFF)
        data.append(command_num & 0xFF)
        for mode, param in all_cmds:
            data.append(mode & 0xFF)
            data += struct.pack("<h", self._clamp_int16(param))
        data.append(self.RX_FRAME_TAIL)
        if pad_canfd:
            data = self._canfd_pad(data)
        return bytes(data)

    def send_command(self, mcu, motor0_cmds, motor1_cmds, counter=0):
        """组装并发送 Rx 指令帧。参数同 build_command_frame。"""
        try:
            frame = self.build_command_frame(mcu, motor0_cmds, motor1_cmds, counter)
        except ValueError as e:
            print(f"[发送失败] {e}")
            return False
        return self._write_frame(frame)

    # ---------- 通用 ----------
    def send_raw_command(self, mcu, mode, param_m0, param_m1=None):
        """
        发送原始(未换算)指令: 直接给电机0/电机1 各下发一条 (mode, param) 指令。
        param_m1 为 None 时对两电机使用相同 param。
        """
        if param_m1 is None:
            param_m1 = param_m0
        return self.send_command(mcu, [(mode, param_m0)], [(mode, param_m1)])

    def _send_paired(self, mcu, mode, value_m0, value_m1):
        """同一模式, 按 MODE_SCALE 换算后给两电机各下发一条。"""
        return self.send_command(
            mcu,
            [(mode, self._quantize(mode, value_m0))],
            [(mode, self._quantize(mode, value_m1))],
        )

    def _send_action(self, mcu, mode):
        """无参数的动作类指令(对两电机同时生效, param=0)。"""
        return self.send_command(mcu, [(mode, 0)], [(mode, 0)])

    # ---------- 状态机派发 ----------
    def send_state(self, mcu, state):
        """派发状态机: state 可为 int 或 MOTOR_STATE 键名字符串。"""
        if isinstance(state, str):
            if state not in self.MOTOR_STATE:
                print(f"[发送失败] 未知状态名: {state}")
                return False
            state = self.MOTOR_STATE[state]
        return self.send_command(
            mcu,
            [(self.MODE_FOC_DISPATCH_STATE, int(state))],
            [(self.MODE_FOC_DISPATCH_STATE, int(state))],
        )

    def send_elec_angle_calib(self, mcu):
        return self.send_state(mcu, "StartupElecAngleDrag")

    # ---------- 位置 / 速度 / 力矩 / 电流 ----------
    def send_position(self, mcu, theta0_rad, theta1_rad):
        return self._send_paired(mcu, self.MODE_FOC_THETA_GEAR, theta0_rad, theta1_rad)

    def send_velocity(self, mcu, omega0, omega1):
        return self._send_paired(mcu, self.MODE_FOC_OMEGA_GEAR, omega0, omega1)

    def send_torque(self, mcu, tau0, tau1):
        return self._send_paired(mcu, self.MODE_FOC_TORQUE_GEAR, tau0, tau1)

    def send_iq(self, mcu, iq0, iq1):
        return self._send_paired(mcu, self.MODE_FOC_IQ, iq0, iq1)

    def send_id(self, mcu, id0, id1):
        return self._send_paired(mcu, self.MODE_FOC_ID, id0, id1)

    # ---------- 阻抗控制 ----------
    def send_impedance_spring_origin(self, mcu, origin0, origin1):
        return self._send_paired(mcu, self.MODE_FOC_IMPEDANCE_SPRING_ORIGIN, origin0, origin1)

    def send_impedance_params(self, mcu, spring, damper, inertia):
        """一次性设置 弹簧刚度/阻尼/惯量 (两电机相同)。"""
        m = [
            (self.MODE_FOC_IMPEDANCE_SPRING, self._quantize(self.MODE_FOC_IMPEDANCE_SPRING, spring)),
            (self.MODE_FOC_IMPEDANCE_DAMPER, self._quantize(self.MODE_FOC_IMPEDANCE_DAMPER, damper)),
            (self.MODE_FOC_IMPEDANCE_INERTIA, self._quantize(self.MODE_FOC_IMPEDANCE_INERTIA, inertia)),
        ]
        return self.send_command(mcu, m, list(m))

    # ---------- 轨迹规划 ----------
    def send_trajectory(self, mcu, vel_max, acl_max, pos0, pos1):
        """一帧内设定 速度上限/加速度上限/目标位置 (位置可分电机)。"""
        m0 = [
            (self.MODE_APP_TRAJ_VEL_MAX, self._quantize(self.MODE_APP_TRAJ_VEL_MAX, vel_max)),
            (self.MODE_APP_TRAJ_ACL_MAX, self._quantize(self.MODE_APP_TRAJ_ACL_MAX, acl_max)),
            (self.MODE_APP_TRAJ_POS_CMD, self._quantize(self.MODE_APP_TRAJ_POS_CMD, pos0)),
        ]
        m1 = [
            (self.MODE_APP_TRAJ_VEL_MAX, self._quantize(self.MODE_APP_TRAJ_VEL_MAX, vel_max)),
            (self.MODE_APP_TRAJ_ACL_MAX, self._quantize(self.MODE_APP_TRAJ_ACL_MAX, acl_max)),
            (self.MODE_APP_TRAJ_POS_CMD, self._quantize(self.MODE_APP_TRAJ_POS_CMD, pos1)),
        ]
        return self.send_command(mcu, m0, m1)

    def send_trajectory_pos(self, mcu, pos0, pos1):
        """只更新轨迹目标位置(不改速度/加速度上限)。"""
        return self._send_paired(mcu, self.MODE_APP_TRAJ_POS_CMD, pos0, pos1)

    # ---------- PID 增益 ----------
    def send_pos_pid(self, mcu, kp, ki):
        m = [
            (self.MODE_FOC_POS_PID_KP, self._quantize(self.MODE_FOC_POS_PID_KP, kp)),
            (self.MODE_FOC_POS_PID_KI, self._quantize(self.MODE_FOC_POS_PID_KI, ki)),
        ]
        return self.send_command(mcu, m, list(m))

    def send_vel_pid(self, mcu, kp, ki):
        m = [
            (self.MODE_FOC_VEL_PID_KP, self._quantize(self.MODE_FOC_VEL_PID_KP, kp)),
            (self.MODE_FOC_VEL_PID_KI, self._quantize(self.MODE_FOC_VEL_PID_KI, ki)),
        ]
        return self.send_command(mcu, m, list(m))

    def send_cur_pid(self, mcu, kp, ki):
        m = [
            (self.MODE_FOC_CUR_PID_KP, self._quantize(self.MODE_FOC_CUR_PID_KP, kp)),
            (self.MODE_FOC_CUR_PID_KI, self._quantize(self.MODE_FOC_CUR_PID_KI, ki)),
        ]
        return self.send_command(mcu, m, list(m))

    # ---------- App 动作类 ----------
    def send_homing(self, mcu):
        return self._send_action(mcu, self.MODE_APP_HOMING)

    def send_flashing_params(self, mcu):
        return self._send_action(mcu, self.MODE_APP_FLASHING_PARAMS)

    def send_traj_init(self, mcu):
        return self._send_action(mcu, self.MODE_APP_TRAJ_INIT)

    def send_traj_deinit(self, mcu):
        return self._send_action(mcu, self.MODE_APP_TRAJ_DEINIT)

    def send_sysiden(self, mcu):
        return self._send_action(mcu, self.MODE_APP_SYSIDEN)

    def send_clear_flash_error(self, mcu):
        return self._send_action(mcu, self.MODE_DEVICE_CLEAR_FLASH_ERROR)
