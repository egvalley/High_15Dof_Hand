"""
串口管理器 (重构版)。

职责严格限制为：
    1. 打开 / 关闭串口
    2. 线程安全的 write_data()
    3. 后台线程读取字节流
    4. 交给 TxFeedbackCodec 拼帧 / 解析
    5. 保存每个 MCU 的最新反馈 + 通信统计
    6. 对外提供线程安全快照

它【不再】负责：控制模式枚举、命令帧构造、CRC、GUI 文本、曲线业务含义。
"""

import copy
import threading
import time

import serial

from sdk.protocol.constants import McuConfig
from sdk.protocol.tx_feedback_codec import TxFeedbackCodec
from sdk.models import LinkStats


class SerialManager:

    # 兼容旧调用方：暴露拓扑常量 (真正定义在 McuConfig)
    NUM_MCU = McuConfig.COUNT
    MCU_IDS = McuConfig.IDS
    MCU_BASE_ID = McuConfig.BASE_ID
    MOTORS_PER_MCU = McuConfig.MOTORS_PER_MCU

    def __init__(self, port, baudrate=115200, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.ser = None
        self.is_connected = False

        self._stop_event = threading.Event()
        self._read_thread = None

        self._data_lock = threading.Lock()   # 保护 _frames / _stats
        self._write_lock = threading.Lock()  # 串行化 write

        self._codec = TxFeedbackCodec()
        self._frames = [None] * McuConfig.COUNT       # list[McuFeedback | None]
        self._stats = [LinkStats() for _ in range(McuConfig.COUNT)]
        self._expected_counter = [0] * McuConfig.COUNT
        self._last_loss_calc_time = time.time()

    # ============================================================ 连接管理
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
            self._codec.reset()
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

        # 防止接收线程内部因异常调用 disconnect() 时 join 自己
        current = threading.current_thread()
        if (self._read_thread
                and self._read_thread.is_alive()
                and self._read_thread is not current):
            self._read_thread.join(timeout=1.0)

        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"[断开连接] 串口 {self.port} 已关闭。")

    def reset_stats(self):
        with self._data_lock:
            self._frames = [None] * McuConfig.COUNT
            self._stats = [LinkStats() for _ in range(McuConfig.COUNT)]

    # ============================================================ 接收线程
    def _read_loop(self):
        while not self._stop_event.is_set():
            if not (self.ser and self.ser.is_open):
                time.sleep(0.1)
                continue
            try:
                self._tick_loss_rate()

                in_waiting = self.ser.in_waiting
                if in_waiting > 0:
                    chunk = self.ser.read(in_waiting)
                else:
                    time.sleep(0.005)
                    continue

                for feedback in self._codec.feed(chunk):
                    self._store_feedback(feedback)

            except serial.SerialException:
                print("\n[错误] 串口异常，连接可能已断开。")
                self.disconnect()
                break
            except Exception as e:
                print(f"\n[解析异常]: {e}")

    def _tick_loss_rate(self):
        now = time.time()
        if now - self._last_loss_calc_time < 1.0:
            return
        with self._data_lock:
            for st in self._stats:
                total = st.received + st.lost
                st.last_sec_received = st.received
                st.last_sec_lost = st.lost
                st.loss_rate = (st.lost / total * 100.0) if total > 0 else 0.0
                st.received = 0
                st.lost = 0
        self._last_loss_calc_time = now

    def _store_feedback(self, feedback):
        with self._data_lock:
            idx = feedback.mcu_index
            self._frames[idx] = feedback
            self._update_stats_locked(idx, feedback.frame_counter)

    def _update_stats_locked(self, idx, frame_counter):
        st = self._stats[idx]
        st.received += 1
        if st.ever_received:
            expected = self._expected_counter[idx]
            if frame_counter != expected:
                lost = (frame_counter - expected) & 0xFF
                if 0 < lost < 255:
                    st.lost += lost
        st.ever_received = True
        self._expected_counter[idx] = (frame_counter + 1) & 0xFF

    # ============================================================ 快照查询
    def get_snapshot(self, mcu):
        """
        返回 (McuFeedback|None, LinkStats) 的深拷贝，保证同一次刷新里数据一致。
        """
        idx = McuConfig.to_index(mcu)
        with self._data_lock:
            frame = copy.deepcopy(self._frames[idx])
            stats = copy.deepcopy(self._stats[idx])
        return frame, stats

    # ---- 兼容旧接口 (GUI 迁移期用) ----
    def get_latest_frame(self, mcu):
        frame, _ = self.get_snapshot(mcu)
        return frame

    def get_mcu_stats(self, mcu):
        _, stats = self.get_snapshot(mcu)
        return stats

    # ============================================================ 发送
    def write_data(self, data):
        """
        供 SerialCommander 调用的底层发送接口。
        返回 True 表示完整写入。
        """
        if not self.is_connected or not self.ser or not self.ser.is_open:
            print("[发送失败] 串口未连接或已关闭")
            return False
        try:
            with self._write_lock:
                written = self.ser.write(data)
            if written != len(data):
                print(f"[发送失败] 数据未完整写入: {written}/{len(data)} 字节")
                return False
            return True
        except serial.SerialException as e:
            print(f"[发送错误] 串口异常: {e}")
            self.is_connected = False
            return False
        except Exception as e:
            print(f"[发送错误]: {e}")
            return False
