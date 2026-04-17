# 文件名: serial_manager.py
import serial
import threading
import queue
import time
import struct

class SerialManager:
    # --- 协议常量定义 ---
    FRAME_HEADER      = 0x5A
    FRAME_TAIL        = 0x0D
    HEADER_SIZE       = 10
    TAIL_SIZE         = 3
    FUNC_NORMAL_SPEED = 0x00
    FUNC_HIGH_SPEED   = 0x01
    INNER_FREQ        = 10000.0
    OUTER_FREQ        = 1000.0

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
        self.receive_queue = queue.Queue()  # 存放解析好的业务数据
        self.hex_queue = queue.Queue()      # 存放原始的Hex字符串
        
        # --- 状态与统计 ---
        self.rx_curve_count = 0
        self.rx_sample_count = 0
        self.rx_data_speed = 500
        
        # 丢包统计(累计变量)
        self.expected_frame_counter = 0
        self.total_packets_received = 0
        self.total_packets_lost = 0
        self.last_loss_calc_time = time.time()
        
        # 新增：向外暴露的当前状态 (每秒更新一次)
        self.current_loss_rate = 0.0
        self.last_sec_received = 0
        self.last_sec_lost = 0

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

    def send_raw_data(self, command1: float, command2: float) -> bool:
        """
        根据下位机协议发送2条commands
        """
        if not self.is_connected or not self.ser:
            print("[发送失败] 串口未连接")
            return False
        try:
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
        while len(self.serial_buffer) >= self.HEADER_SIZE + self.TAIL_SIZE:
            # 1. 寻找帧头 0x5A
            header_index = self.serial_buffer.find(self.FRAME_HEADER)
            if header_index == -1:
                self.serial_buffer.clear()
                break
            if header_index > 0:
                del self.serial_buffer[:header_index] # 移除脏数据
                
            if len(self.serial_buffer) < self.HEADER_SIZE:
                break
                
            # 2. 解析基础头信息获取帧长度
            func_byte = self.serial_buffer[3]
            curve_count = self.serial_buffer[8]
            sample_count = self.serial_buffer[9]
            
            # 校验参数，防内存溢出计算
            if curve_count == 0 or sample_count == 0:
                del self.serial_buffer[:1]
                continue
                
            payload_byte_size = curve_count * sample_count * 2 # sizeof(int16_t)
            frame_length = self.HEADER_SIZE + curve_count + payload_byte_size + self.TAIL_SIZE
            
            if len(self.serial_buffer) < frame_length:
                break # 长度不够，等待更多数据
                
            frame = self.serial_buffer[:frame_length]
            
            # 3. 校验帧尾
            if frame[frame_length - 1] != self.FRAME_TAIL:
                del self.serial_buffer[:1]
                continue
                
            # 4. 校验 CRC
            crc_data_length = self.HEADER_SIZE + curve_count + payload_byte_size
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
                if func_byte == self.FUNC_HIGH_SPEED:
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
                current_offset = self.HEADER_SIZE + curve_count
                for s in range(sample_count):
                    sample_data = []
                    for c in range(curve_count):
                        # 从当前偏移量读取一个 int16
                        q_val = struct.unpack_from('<h', frame, current_offset)[0]
                        val = self._dequantize_value(q_val, max_abs_values[c])
                        sample_data.append(val)
                        current_offset += 2 # 步进
                    parsed_frame['samples'].append(sample_data)
                
                # 抛出解析结果
                self.receive_queue.put(parsed_frame)
                
                # 抛出原始Hex (可选使用)
                hex_str = frame.hex(' ').upper() + "\n"
                self.hex_queue.put(hex_str)
                
                # 移除已解析的数据
                del self.serial_buffer[:frame_length]
            else:
                del self.serial_buffer[:1]

    # ========================== 获取数据接口 ==========================

    def get_parsed_data(self):
        """获取一帧解析好的业务数据字典 (非阻塞)"""
        try:
            return self.receive_queue.get_nowait()
        except queue.Empty:
            return None