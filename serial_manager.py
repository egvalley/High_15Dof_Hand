import serial
import threading
import queue
import time
import struct
import math # 新增 math 用于 pi 的计算

class SerialManager:
    # --- 协议常量定义 (原有) ---
    TX_FRAME_HEADER         = 0x5A
    TX_FRAME_TAIL           = 0x0D
    TX_HEADER_SIZE          = 10
    TX_TAIL_SIZE            = 3
    TX_FUNC_NORMAL_SPEED    = 0x00
    TX_FUNC_HIGH_SPEED      = 0x01
    INNER_FREQ              = 10000.0
    OUTER_FREQ              = 1000.0

    # --- 新增: 发送(Rx)协议常量定义 ---
    RX_FRAME_HEADER         = 0x3B
    RX_FRAME_TAIL           = 0x1E
    CONTROL_MODE_ENUM       = { 
        "AppDisable": 0x00,
        "Iq": 0x01,
        "Id": 0x02,
        "Omegam": 0x03,
        "Thetam": 0x04
    }

    RX_TARGET_ID               = { 
        "Finger0": 0xB0,
        "Finger1": 0xBF,
        "Finger2": 0xAA,
        "Finger3": 0x23
    }



    def __init__(self, port, baudrate=115200, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        self.ser = None
        self.is_connected = False
        self._stop_event = threading.Event()
        self._read_thread = None
        
        # 数据缓冲区与队列
        self.serial_buffer = bytearray()
        self.receive_queue_mcu_0 = queue.Queue()
        self.receive_queue_mcu_1 = queue.Queue()
        self.receive_queue_mcu_2 = queue.Queue()
        self.receive_queue_mcu_3 = queue.Queue()
        
        # --- 状态与统计 ---
        self.rx_curve_count = 0
        self.rx_sample_count = 0
        self.rx_data_speed = 500
        
        # 丢包统计
        self.expected_frame_counter = 0
        self.total_packets_received = 0
        self.total_packets_lost = 0
        self.last_loss_calc_time = time.time()
        
        
        # 暴露的当前状态
        self.current_loss_rate = 0.0
        self.last_sec_received = 0
        self.last_sec_lost = 0

        # 保存每个手指的最新有效帧
        self.latest_frame_finger_0 = None
        self.latest_frame_finger_1 = None
        self.latest_frame_finger_2 = None
        self.latest_frame_finger_3 = None

    def connect(self):
        """打开串口并启动读取线程"""
        try:
            self.ser = serial.Serial(
                port=self.port, 
                baudrate=self.baudrate, 
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            self.is_connected = True
            self.serial_buffer.clear()
            
            # 连接时重置所有统计
            self.reset_packet_loss_stats()
            self.current_loss_rate = 0.0
            self.last_sec_received = 0
            self.last_sec_lost = 0
            
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
        """关闭串口"""
        self.is_connected = False
        self._stop_event.set()
        
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
            
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"[断开连接] 串口 {self.port} 已关闭。")

    def reset_packet_loss_stats(self):
        """重置丢包率累加器"""
        self.expected_frame_counter = 0
        self.total_packets_received = 0
        self.total_packets_lost = 0

    # ========================== 发送数据接口 ==========================

    def send_theta_command(self, target_index: int, theta1_rad: float, theta2_rad: float) -> bool:
        """
        根据下位机 Rx 协议发送 2 条关节位置(Thetam)指令
        :param target_index: 目标设备的索引 (0,1,2,3 对应 Finger1-Finger4)
        :param theta1_rad: 电机 1 的目标角度 (弧度)
        :param theta2_rad: 电机 2 的目标角度 (弧度)
        """
        if not self.is_connected or not self.ser:
            print("[发送失败] 串口未连接")
            return False

        # 验证target_index范围
        if target_index < 0 or target_index > 3:
            print(f"[发送失败] target_index 超出范围 (0-3): {target_index}")
            return False
            
        # 通过target_index索引获取对应的target_id
        finger_keys = list(self.RX_TARGET_ID.keys())
        target_id = self.RX_TARGET_ID[finger_keys[target_index]]

        # 1. 浮点数转化为 16-bit 有符号整数 (反量化)
        param1 = int(theta1_rad)
        param2 = int(theta2_rad)

        try:
            data = struct.pack('<BBBBB h B h B',
                self.RX_FRAME_HEADER,      # 0 帧头: 0x3B
                target_id,                 # 1 ID: 路由对象 (从RX_TARGET_ID字典获取)
                0,     # 2 计数器: 0~255
                2,                         # 3 命令数量: 总是2 (M1 和 M2)
                self.CONTROL_MODE_ENUM["Thetam"],     # 4 M1 Mode
                param1,                    # 5-6 M1 Command 
                self.CONTROL_MODE_ENUM["Thetam"],     # 7 M2 Mode
                param2,                    # 8-9 M2 Command
                self.RX_FRAME_TAIL         # 10 帧尾: 0x1E
            )

            # 3. 发送并通过串口发送
            self.ser.write(data)
            
            return True
            
        except Exception as e:
            print(f"[发送错误]: {e}")
            return False

    # ========================== 核心解析逻辑 ==========================

    @staticmethod
    def _calculate_crc16(data: bytearray, length: int) -> int:
        """CRC16 计算"""
        crc = 0xFFFF
        for i in range(length):
            crc ^= data[i]
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    @staticmethod
    def _dequantize_value(q_val: int, max_abs: int) -> float:
        """反量化 16-bit 值为浮点数"""
        if max_abs <= 0.0001:
            max_abs = 1
        normalized_val = q_val / 32767.0
        return normalized_val * max_abs

    def _read_loop(self):
        """后台读取并解析数据的线程循环"""
        while not self._stop_event.is_set():
            if self.ser and self.ser.is_open:
                try:
                    # 1. 周期性计算丢包率 (1秒一次)
                    current_time = time.time()
                    if current_time - self.last_loss_calc_time >= 1.0:
                        total_packets = self.total_packets_received + self.total_packets_lost
                        
                        # 保存过去1秒的绝对数量供UI读取
                        self.last_sec_received = self.total_packets_received
                        self.last_sec_lost = self.total_packets_lost
                        
                        if total_packets > 0:
                            self.current_loss_rate = (self.total_packets_lost / total_packets) * 100.0
                        else:
                            self.current_loss_rate = 0.0 # 若没有数据包更新，强制设为0
                            
                        # 计算完后清零，开启下一秒的统计
                        self.reset_packet_loss_stats()
                        self.last_loss_calc_time = current_time

                    # 2. 读取全部可用数据并追加到缓冲区
                    in_waiting = self.ser.in_waiting
                    if in_waiting > 0:
                        new_data = self.ser.read(in_waiting)
                        self.serial_buffer.extend(new_data)
                    else:
                        time.sleep(0.005) # 没有数据时休息一下
                        continue
                    
                    # 3. 黏包处理与协议解析
                    self._parse_buffer()
                    
                except serial.SerialException:
                    print("\n[错误] 串口异常，连接可能已断开。")
                    self.disconnect()
                    break
                except Exception as e:
                    print(f"\n[解析异常]: {e}")
            else:
                time.sleep(0.1)

    def _parse_buffer(self):
        """解析缓冲区中的数据包"""
        while len(self.serial_buffer) >= self.TX_HEADER_SIZE + self.TX_TAIL_SIZE:
            # 1. 寻找帧头 0x5A
            header_index = self.serial_buffer.find(self.TX_FRAME_HEADER)
            if header_index == -1:
                self.serial_buffer.clear()
                break
            if header_index > 0:
                del self.serial_buffer[:header_index] # 移除脏数据
                
            if len(self.serial_buffer) < self.TX_HEADER_SIZE:
                break

            mcu_id = self.serial_buffer[1]    
            # 2. 解析基础头信息获取帧长度
            func_byte = self.serial_buffer[3]
            curve_count = self.serial_buffer[8]
            sample_count = self.serial_buffer[9]
            
            # 校验参数，防内存溢出计算
            if curve_count == 0 or sample_count == 0:
                del self.serial_buffer[:1]
                continue
                
            payload_byte_size = curve_count * sample_count * 2 # sizeof(int16_t)
            frame_length = self.TX_HEADER_SIZE + curve_count + payload_byte_size + self.TX_TAIL_SIZE
            
            if len(self.serial_buffer) < frame_length:
                break # 长度不够，等待更多数据
                
            frame = self.serial_buffer[:frame_length]
            
            # 3. 校验帧尾
            if frame[frame_length - 1] != self.TX_FRAME_TAIL:
                del self.serial_buffer[:1]
                continue
                
            # 4. 校验 CRC
            crc_data_length = self.TX_HEADER_SIZE + curve_count + payload_byte_size
            calc_crc = self._calculate_crc16(frame, crc_data_length)
            recv_crc = (frame[frame_length - 2] << 8) | frame[frame_length - 3]
            
            if calc_crc == recv_crc:
                # ----------------- 数据包合法，开始解析 -----------------
                self.rx_curve_count = curve_count
                self.rx_sample_count = sample_count
                device_id = frame[1]
                frame_counter = frame[2]
                
                # 丢包检测累加
                self.total_packets_received += 1
                if self.total_packets_received > 1:
                    expected = self.expected_frame_counter
                    actual = frame_counter
                    if actual != expected:
                        lost = (actual - expected) if actual >= expected else (256 - expected + actual)
                        if 0 < lost < 255:
                            self.total_packets_lost += lost
                            
                self.expected_frame_counter = (frame_counter + 1) % 256
                
                # 解析硬件时间戳 (小端 4字节)
                timestamp_ms = struct.unpack('<I', frame[4:8])[0]
                if func_byte == self.TX_FUNC_HIGH_SPEED:
                    base_hw_time = timestamp_ms / self.INNER_FREQ
                    self.rx_data_speed = 10000
                else:
                    base_hw_time = timestamp_ms / self.OUTER_FREQ
                    self.rx_data_speed = 500
                    
                # 解析 Max_Abs 数组
                max_abs_values = [frame[10 + i] for i in range(curve_count)]
                
                # 组织数据结构并解析 Payload
                parsed_frame = {'device_id': device_id, 'hw_time': base_hw_time, 'samples': []}
                
                # 核心解析循环：与 C++ 逻辑保持一比一对应
                current_offset = self.TX_HEADER_SIZE + curve_count
                for s in range(sample_count):
                    sample_data = []
                    for c in range(curve_count):
                        # 从当前偏移量读取一个 int16
                        q_val = struct.unpack_from('<h', frame, current_offset)[0]
                        val = self._dequantize_value(q_val, max_abs_values[c])
                        sample_data.append(val)
                        current_offset += 2 # 步进
                    parsed_frame['samples'].append(sample_data)
                
                # 抛出解析结果并更新最新帧
                matched = False
                if mcu_id == self.RX_TARGET_ID['Finger0']:
                    self.receive_queue_mcu_0.put(parsed_frame)
                    self.latest_frame_finger_0 = parsed_frame
                    matched = True

                if mcu_id == self.RX_TARGET_ID['Finger1']:
                    self.receive_queue_mcu_1.put(parsed_frame)
                    self.latest_frame_finger_1 = parsed_frame
                    matched = True

                if mcu_id == self.RX_TARGET_ID['Finger2']:
                    self.receive_queue_mcu_2.put(parsed_frame)
                    self.latest_frame_finger_2 = parsed_frame
                    matched = True

                if mcu_id == self.RX_TARGET_ID['Finger3']:
                    self.receive_queue_mcu_3.put(parsed_frame)
                    self.latest_frame_finger_3 = parsed_frame
                    matched = True

                if not matched:
                    print(f"[警告] 未识别的 mcu_id: 0x{mcu_id:02X}, 期望值: Finger0=0x01, Finger1=0xBF, Finger2=0xAA, Finger3=0x23")
                
                # 移除已解析的数据
                del self.serial_buffer[:frame_length]
            else:
                del self.serial_buffer[:1]

    # ========================== 获取数据接口 ==========================

    def get_parsed_data_finger_0(self):
        """获取一帧解析好的业务数据字典 (非阻塞)"""
        try:
            return self.receive_queue_mcu_0.get_nowait()
        except queue.Empty:
            return None
        
    def get_parsed_data_finger_1(self):
        """获取一帧解析好的业务数据字典 (非阻塞)"""
        try:
            return self.receive_queue_mcu_1.get_nowait()
        except queue.Empty:
            return None
        
    def get_parsed_data_finger_2(self):
        """获取一帧解析好的业务数据字典 (非阻塞)"""
        try:
            return self.receive_queue_mcu_2.get_nowait()
        except queue.Empty:
            return None
    
    def get_parsed_data_finger_3(self):
        """获取一帧解析好的业务数据字典 (非阻塞)"""
        try:
            return self.receive_queue_mcu_3.get_nowait()
        except queue.Empty:
            return None

    # ========================== 获取最新 Curve 数据接口 ==========================

    def get_latest_curve_finger_0(self):
        """获取手指0的最新curve数据"""
        if self.latest_frame_finger_0 and self.latest_frame_finger_0['samples']:
            return self.latest_frame_finger_0['samples'][-1]
        return None

    def get_latest_curve_finger_1(self):
        """获取手指1的最新curve数据"""
        if self.latest_frame_finger_1 and self.latest_frame_finger_1['samples']:
            return self.latest_frame_finger_1['samples'][-1]
        return None

    def get_latest_curve_finger_2(self):
        """获取手指2的最新curve数据"""
        if self.latest_frame_finger_2 and self.latest_frame_finger_2['samples']:
            return self.latest_frame_finger_2['samples'][-1]
        return None

    def get_latest_curve_finger_3(self):
        """获取手指3的最新curve数据"""
        if self.latest_frame_finger_3 and self.latest_frame_finger_3['samples']:
            return self.latest_frame_finger_3['samples'][-1]
        return None