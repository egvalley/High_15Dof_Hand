import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import struct
from functools import partial
from sdk.serial_manager import SerialManager
from sdk.forward_reverse_formula import TendonDriveKinematics

class KinematicsGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("腱传动机械臂控制与运动学分析")
        self.root.geometry("850x700") # 加宽窗口以容纳更长的文字
        
        # 初始化逻辑类
        self.kin = TendonDriveKinematics()
        self.serial_mgr = None
        
        # 用于缓存待发送的数据,4个手指分别缓存待发送的数据
        self.current_enc = [[0.0, 0.0] for _ in range(4)]

        # 存储4组输入框的引用，防止变量名覆盖
        self.finger_inputs = []
        self.fwd_labels = []
        self.send_btns = []

        self._setup_ui()
        
        # 启动定时器，每50ms刷新一次界面数据
        self.root.after(50, self._update_loop)

    def _setup_ui(self):
        """初始化UI布局"""
        # --- 1. 串口配置区 ---
        serial_frame = ttk.LabelFrame(self.root, text="串口设置")
        serial_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(serial_frame, text="端口:").pack(side="left", padx=5)
        self.port_entry = ttk.Entry(serial_frame, width=8)
        self.port_entry.insert(0, "COM3")
        self.port_entry.pack(side="left", padx=5)

        ttk.Label(serial_frame, text="波特率:").pack(side="left", padx=5)
        self.baud_entry = ttk.Entry(serial_frame, width=8)
        self.baud_entry.insert(0, "115200")
        self.baud_entry.pack(side="left", padx=5)

        self.conn_btn = ttk.Button(serial_frame, text="连接串口", command=self._toggle_serial)
        self.conn_btn.pack(side="left", padx=10)

        self.status_label = ttk.Label(serial_frame, text="状态: 未连接", foreground="red", width=12)
        self.status_label.pack(side="left", padx=5)
        
        # 新增：丢包率与统计数量显示标签
        self.loss_rate_label = ttk.Label(serial_frame, text="丢包率: 0.00% (收: 0, 丢: 0)", foreground="black", font=("Arial", 10, "bold"))
        self.loss_rate_label.pack(side="left", padx=10)

        # --- 2. 实时曲线数据打印区 ---
        data_frame = ttk.LabelFrame(self.root, text="实时串口数据 (Latest Curve Values)")
        data_frame.pack(fill="x", padx=10, pady=5)
        
        self.curve_text = tk.Text(data_frame, height=8, state="disabled", background="#f0f0f0")
        self.curve_text.pack(fill="x", padx=5, pady=5)

        # --- 3. 手动正向运动学 (1-4号手指) ---
        for i in range(1, 5):
            frame = ttk.LabelFrame(self.root, text=f"手指 {i}：关节角度(rad) -> 电机角度(rad)")
            frame.pack(fill="x", padx=10, pady=2)

            ttk.Label(frame, text="中指:").grid(row=0, column=0, padx=5, pady=5)
            e1 = ttk.Entry(frame, width=8)
            e1.grid(row=0, column=1, padx=2, pady=5)

            ttk.Label(frame, text="近指:").grid(row=0, column=2, padx=5, pady=5)
            e2 = ttk.Entry(frame, width=8)
            e2.grid(row=0, column=3, padx=2, pady=5)

            self.finger_inputs.append((e1, e2))

            calc_btn = ttk.Button(frame, text="计算", command=partial(self._manual_calc_and_save, i-1))
            calc_btn.grid(row=0, column=4, padx=5)

            s_btn = ttk.Button(frame, text="发送指令", command=partial(self._send_motor_theta_command, i-1), state="disabled")
            s_btn.grid(row=0, column=5, padx=5)
            self.send_btns.append(s_btn)
            
            res_lab = ttk.Label(frame, text="结果: --", font=("Arial", 9, "bold"))
            res_lab.grid(row=1, column=0, columnspan=6, sticky="w", padx=10, pady=2)
            self.fwd_labels.append(res_lab)

        # --- 4. 自动逆向运动学 ---
        inv_frame = ttk.LabelFrame(self.root, text="实时计算： 电机角度（rad） -> 关节角度（rad）")
        inv_frame.pack(fill="x", padx=10, pady=5)

        select_frame = ttk.Frame(inv_frame)
        select_frame.pack(fill="x", padx=5, pady=5)

        ttk.Label(select_frame, text="选择手指:").grid(row=0, column=0, padx=5)
        self.finger_source = ttk.Combobox(select_frame, values=[f"手指 {i}" for i in range(4)], width=10, state="readonly")
        self.finger_source.current(0)
        self.finger_source.grid(row=0, column=1, padx=5)

        self.inv_input_display = ttk.Label(inv_frame, text="当前提取电机角度(rad): Theta1=0.00，Theta2=0.00")
        self.inv_input_display.pack(pady=5)

        self.inv_result_label = ttk.Label(inv_frame, text="计算结果: --", font=("Arial", 11, "bold"), foreground="blue")
        self.inv_result_label.pack(pady=10)

    # ================= 业务逻辑 =================

    def _toggle_serial(self):
        """切换串口连接状态"""
        if self.serial_mgr and self.serial_mgr.is_connected:
            self.serial_mgr.disconnect()
            self.conn_btn.config(text="连接串口")
            self.status_label.config(text="状态: 未连接", foreground="red")
            self.loss_rate_label.config(text="丢包率: 0.00% (收: 0, 丢: 0)", foreground="black") # 断开时清空显示
            for btn in self.send_btns: btn.config(state="disabled")
        else:
            port = self.port_entry.get()
            try:
                baud = int(self.baud_entry.get())
                self.serial_mgr = SerialManager(port, baud)
                if self.serial_mgr.connect():
                    self.conn_btn.config(text="断开串口")
                    self.status_label.config(text=f"状态: 已连接", foreground="green")
                    for btn in self.send_btns: btn.config(state="normal")
                else:
                    messagebox.showerror("错误", f"无法连接到端口 {port}")
            except ValueError:
                messagebox.showerror("错误", "请输入有效的波特率数字")

    def _manual_calc_and_save(self, index):
        """手动输入计算并保存到待发送变量"""
        try:
            e1_widget, e2_widget = self.finger_inputs[index]
            val1 = float(e1_widget.get())
            val2 = float(e2_widget.get())
            
            # 1. 调用运动学公式 (角度 -> 编码)
            e1, e2 = self.kin.Angles_to_motor_enc(val1, val2)
            
            # 2. 缓存结果到对应手指的列表中 (修改这里)
            self.current_enc[index][0] = e1
            self.current_enc[index][1] = e2
            
            # 3. 更新对应的显示标签
            self.fwd_labels[index].config(
                text=f"计算结果: Enc1={e1} (rad), Enc2={e2} (rad)"
            )
        except Exception as e:
            messagebox.showwarning("输入错误", "请输入有效的数字")

    def _send_motor_theta_command(self, index): # 增加 index 参数
        """发送最新计算出的编码值(弧度)"""
        if not self.serial_mgr or not self.serial_mgr.is_connected:
            messagebox.showwarning("警告", "串口未连接")
            return

        try:
            # 获取对应手指的编码值 (单位是弧度 rad)
            theta1_rad = self.current_enc[index][0]
            theta2_rad = self.current_enc[index][1]
            
            # 调用更新后的发送协议，加入 target_id 参数
            self.serial_mgr.send_theta_command(index, theta1_rad, theta2_rad)
            print(f"已发送指令到设备: M1={theta1_rad:.4f}rad, M2={theta2_rad:.4f}rad")
        except Exception as e:
            messagebox.showerror("发送失败", f"错误: {str(e)}")

    def _update_loop(self):
        """GUI 刷新循环"""
        if self.serial_mgr and self.serial_mgr.is_connected:
            
            # 1. 实时更新 丢包率、接收包裹数、丢失包裹数
            loss_rate = self.serial_mgr.current_loss_rate
            recv_cnt = self.serial_mgr.last_sec_received
            lost_cnt = self.serial_mgr.last_sec_lost
            
            color = "red" if loss_rate > 5.0 else ("orange" if loss_rate > 0 else "green")
            self.loss_rate_label.config(text=f"丢包率: {loss_rate:.2f}% (收: {recv_cnt}, 丢: {lost_cnt})", foreground=color)
            
            # 2. 获取4个手指的最新curve数据并显示
            self.curve_text.config(state="normal")
            self.curve_text.delete(1.0, tk.END)

            display_lines = []
            for i in range(1):
                getter_motor_0_theta_method = getattr(self.serial_mgr, f"get_latest_data_finger_{i}_motor_0_theta")
                getter_motor_1_theta_method = getattr(self.serial_mgr, f"get_latest_data_finger_{i}_motor_1_theta")
                motor_0_theta_data = getter_motor_0_theta_method()
                motor_1_theta_data = getter_motor_1_theta_method()

                if motor_0_theta_data and motor_1_theta_data:
                    # Display motor names with their theta values
                    display_str = f"motor_0_theta:{motor_0_theta_data:.3f}, motor_1_theta:{motor_1_theta_data:.3f}"
                    display_lines.append(f"[手指 {i}] {display_str}")
                else:
                    display_lines.append(f"[手指 {i}] 等待数据中...")

            self.curve_text.insert(tk.END, "\n".join(display_lines))
            self.curve_text.config(state="disabled")

            # 3. 自动逆向运动学计算
            # finger_idx = self.finger_source.current()
            # getter_method = getattr(self.serial_mgr, f"get_latest_curve_finger_{finger_idx}")
            # curve_data = getter_method()

            # if curve_data and len(curve_data) >= 2:
            #     theta1 = curve_data[0]
            #     theta2 = curve_data[1]
            #     self.inv_input_display.config(text=f"提取(手指{finger_idx})电机角度: Theta1={theta1:.2f}°, Theta2={theta2:.2f}°")

            #     e1, e2 = self.kin.Angles_to_motor_enc(theta1, theta2)
            #     self.inv_result_label.config(text=f"关节角度(rad): Joint近={e1:.3f}, Joint中={e2:.3f}")

        self.root.after(50, self._update_loop)

if __name__ == "__main__":
    root = tk.Tk()
    app = KinematicsGUI(root)
    
    def on_closing():
        if app.serial_mgr:
            app.serial_mgr.disconnect()
        root.destroy()
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()