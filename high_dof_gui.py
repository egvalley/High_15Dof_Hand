"""
高自由度机械手上位机 GUI (重构版)。

只负责：
    - 构建界面
    - 读取表单、解析目标 MCU / 目标电机
    - 调用 HandController
    - 展示发送日志
    - 用 SerialManager 快照刷新监控树

不再包含：协议 mode 数值、int16 定点换算、CRC、字节帧构造、
         MCU ID 数学、帧数据布局、丢包计算。
"""

import tkinter as tk
from tkinter import ttk, messagebox

from sdk.config import AppConfig
from sdk.serial_manager import SerialManager
from sdk.serial_commander import SerialCommander
from sdk.hand_controller import HandController
from sdk.models import MotorTarget
from sdk.protocol.constants import McuConfig


class ScrollableFrame(ttk.Frame):
    """带竖向滚动条的容器，内部控件放到 self.body。"""

    def __init__(self, parent, **kw):
        super().__init__(parent, **kw)
        self.canvas = tk.Canvas(self, borderwidth=0, highlightthickness=0)
        vsb = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=vsb.set)

        vsb.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)

        self.body = ttk.Frame(self.canvas)
        self._win = self.canvas.create_window((0, 0), window=self.body, anchor="nw")

        self.body.bind("<Configure>",
                       lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>",
                         lambda e: self.canvas.itemconfigure(self._win, width=e.width))
        self.canvas.bind("<Enter>", lambda e: self.canvas.bind_all("<MouseWheel>", self._on_wheel))
        self.canvas.bind("<Leave>", lambda e: self.canvas.unbind_all("<MouseWheel>"))

    def _on_wheel(self, event):
        self.canvas.yview_scroll(int(-event.delta / 120), "units")


class HighDofHandGUI:

    # 状态机派发按钮: (显示名, 状态键名)
    STATE_BUTTONS = [
        ("就绪", "StartupReady"),
        ("初始角标定", "StartupElecAngleDrag"),
        ("电流环", "AppCurrentCtrl"),
        ("速度环", "AppVelocityCtrl"),
        ("位置环", "AppPositionCtrl"),
        ("力矩环", "AppTorqueCtrl"),
        ("阻抗环", "AppImpedanceCtrl"),
        ("失能", "AppDisable"),
    ]

    def __init__(self, root, config=None):
        self.root = root
        self.config = config or AppConfig()
        self.root.title("高自由度机械手 · MCU 监控与控制台")
        self.root.geometry("1180x840")

        self.manager = None
        self.commander = None
        self.controller = None
        self.entries = {}
        self._ui_connected = False

        self._setup_ui()
        self.root.after(self.config.refresh_ms, self._update_loop)

    # ============================================================ UI 构建
    def _setup_ui(self):
        bar = ttk.LabelFrame(self.root, text="串口设置")
        bar.pack(fill="x", padx=10, pady=6)

        ttk.Label(bar, text="端口:").pack(side="left", padx=(8, 2))
        self.port_entry = ttk.Entry(bar, width=9)
        self.port_entry.insert(0, self.config.serial.port)
        self.port_entry.pack(side="left", padx=2)

        ttk.Label(bar, text="波特率:").pack(side="left", padx=(8, 2))
        self.baud_entry = ttk.Entry(bar, width=9)
        self.baud_entry.insert(0, str(self.config.serial.baudrate))
        self.baud_entry.pack(side="left", padx=2)

        self.conn_btn = ttk.Button(bar, text="连接串口", command=self._toggle_serial)
        self.conn_btn.pack(side="left", padx=10)

        self.status_label = ttk.Label(bar, text="状态: 未连接", foreground="red")
        self.status_label.pack(side="left", padx=6)

        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=10, pady=6)
        self.tab_monitor = ttk.Frame(nb)
        self.tab_control = ttk.Frame(nb)
        nb.add(self.tab_monitor, text="  实时监控  ")
        nb.add(self.tab_control, text="  指令控制  ")

        self._build_monitor_tab()
        self._build_control_tab()

    # ---------------------------------------------------------- 监控页
    def _build_monitor_tab(self):
        info = ttk.Label(
            self.tab_monitor,
            text="8 个 MCU (0xB0~0xB7)，每个 MCU 管控 2 个电机，"
                 "每个电机 5 条低速曲线 (状态机 / θ / ω / 加速度 / 力矩)。",
            foreground="#555",
        )
        info.pack(fill="x", padx=6, pady=(6, 2))

        cols = ("state", "theta", "omega", "acl", "torque", "meta")
        tree = ttk.Treeview(self.tab_monitor, columns=cols, show="tree headings", height=26)
        tree.heading("#0", text="设备 / 电机")
        tree.heading("state", text="状态")
        tree.heading("theta", text="θ (rad)")
        tree.heading("omega", text="ω (rad/s)")
        tree.heading("acl", text="a (rad/s²)")
        tree.heading("torque", text="τ (N·m)")
        tree.heading("meta", text="链路 / 丢包")

        tree.column("#0", width=180, anchor="w")
        tree.column("state", width=170, anchor="center")
        tree.column("theta", width=110, anchor="e")
        tree.column("omega", width=110, anchor="e")
        tree.column("acl", width=110, anchor="e")
        tree.column("torque", width=110, anchor="e")
        tree.column("meta", width=200, anchor="center")

        tree.tag_configure("nodata", foreground="#999")
        tree.tag_configure("ok", foreground="#127a12")
        tree.tag_configure("warn", foreground="#b26a00")
        tree.tag_configure("err", foreground="#c00000")
        tree.tag_configure("mcu", background="#eef2f7")

        vsb = ttk.Scrollbar(self.tab_monitor, orient="vertical", command=tree.yview)
        tree.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        tree.pack(fill="both", expand=True, padx=6, pady=4)
        self.tree = tree

        for i in range(McuConfig.COUNT):
            mcu_id = McuConfig.IDS[i]
            pid = f"mcu{i}"
            tree.insert("", "end", iid=pid, text=f"MCU{i}  (0x{mcu_id:02X})",
                        values=("", "", "", "", "", "无数据"), tags=("mcu", "nodata"), open=True)
            for m in range(McuConfig.MOTORS_PER_MCU):
                tree.insert(pid, "end", iid=f"{pid}_m{m}", text=f"    电机{m}",
                            values=("—", "—", "—", "—", "—", ""), tags=("nodata",))

    # ---------------------------------------------------------- 控制页
    def _build_control_tab(self):
        top = ttk.Frame(self.tab_control)
        top.pack(fill="x", padx=6, pady=6)
        ttk.Label(top, text="目标 MCU:").pack(side="left", padx=(4, 4))
        target_values = [f"MCU{i} (0x{McuConfig.IDS[i]:02X})" for i in range(McuConfig.COUNT)]
        target_values.append("全部广播")
        self.target_combo = ttk.Combobox(top, values=target_values, width=16, state="readonly")
        self.target_combo.current(0)
        self.target_combo.pack(side="left")

        ttk.Label(top, text="   目标电机:").pack(side="left", padx=(4, 4))
        self.motor_combo = ttk.Combobox(top, values=["两个电机", "仅电机0", "仅电机1"],
                                        width=10, state="readonly")
        self.motor_combo.current(0)
        self.motor_combo.pack(side="left")
        self.motor_combo.bind("<<ComboboxSelected>>", self._on_motor_change)
        ttk.Label(top, text="  (选单个电机时, 只有对应那侧的输入框生效)",
                  foreground="#777").pack(side="left")

        paned = ttk.Panedwindow(self.tab_control, orient="vertical")
        paned.pack(fill="both", expand=True, padx=6, pady=4)

        scroll = ScrollableFrame(paned)
        paned.add(scroll, weight=3)
        body = scroll.body

        self._build_state_section(body)
        self._build_action_section(body)
        self._build_value_sections(body)
        self._build_impedance_section(body)
        self._build_traj_section(body)
        self._build_pid_section(body)
        self._build_raw_section(body)

        logf = ttk.LabelFrame(paned, text="发送日志")
        paned.add(logf, weight=1)
        self.log_text = tk.Text(logf, height=7, state="disabled",
                                background="#101418", foreground="#c8e6c9")
        self.log_text.pack(fill="both", expand=True, padx=4, pady=4)

        # 每电机独立输入框对，用于按“目标电机”启用/禁用
        self._motor_entry_pairs = [
            ("pos0", "pos1"), ("vel0", "vel1"), ("tau0", "tau1"), ("iq0", "iq1"),
            ("imp_o0", "imp_o1"), ("tj_p0", "tj_p1"), ("raw_p0", "raw_p1"),
        ]

    def _build_state_section(self, parent):
        f = ttk.LabelFrame(parent, text="状态机派发 (DispatchMotorStateMachine)")
        f.pack(fill="x", padx=6, pady=5)
        for k, (label, state_key) in enumerate(self.STATE_BUTTONS):
            b = ttk.Button(f, text=label, width=12,
                           command=lambda sk=state_key, lb=label: self._do_state(sk, lb))
            b.grid(row=k // 4, column=k % 4, padx=4, pady=4, sticky="ew")
        for c in range(4):
            f.columnconfigure(c, weight=1)

    def _build_action_section(self, parent):
        f = ttk.LabelFrame(parent, text="App 动作")
        f.pack(fill="x", padx=6, pady=5)
        actions = [
            ("回零 Homing", "send_homing"),
            ("刷写参数", "send_flashing_params"),
            ("轨迹初始化", "send_traj_init"),
            ("轨迹反初始化", "send_traj_deinit"),
            ("系统辨识", "send_sysiden"),
            ("清除Flash错误", "send_clear_flash_error"),
        ]
        for k, (label, fn) in enumerate(actions):
            b = ttk.Button(f, text=label, width=14,
                           command=lambda fn=fn, lb=label: self._do_action(fn, lb))
            b.grid(row=k // 3, column=k % 3, padx=4, pady=4, sticky="ew")
        for c in range(3):
            f.columnconfigure(c, weight=1)

    def _labeled_entries(self, parent, specs):
        for i, (key, label, default) in enumerate(specs):
            ttk.Label(parent, text=label).grid(row=0, column=i * 2, padx=(6, 2), pady=4, sticky="e")
            e = ttk.Entry(parent, width=9)
            e.insert(0, str(default))
            e.grid(row=0, column=i * 2 + 1, padx=(0, 6), pady=4, sticky="w")
            self.entries[key] = e

    def _build_value_sections(self, parent):
        specs = [
            ("位置指令 (Theta_Gear, rad)",
             [("pos0", "电机0", "0.0"), ("pos1", "电机1", "0.0")], "send_position", "位置"),
            ("速度指令 (Omega_Gear, rad/s)",
             [("vel0", "电机0", "0.0"), ("vel1", "电机1", "0.0")], "send_velocity", "速度"),
            ("力矩指令 (Torque_Gear, N·m)",
             [("tau0", "电机0", "0.0"), ("tau1", "电机1", "0.0")], "send_torque", "力矩"),
            ("电流指令 (Iq, A)",
             [("iq0", "电机0", "0.0"), ("iq1", "电机1", "0.0")], "send_iq", "电流Iq"),
        ]
        for title, entry_specs, ctrl_method, short in specs:
            f = ttk.LabelFrame(parent, text=title)
            f.pack(fill="x", padx=6, pady=5)
            row = ttk.Frame(f)
            row.pack(side="left")
            self._labeled_entries(row, entry_specs)
            keys = (entry_specs[0][0], entry_specs[1][0])
            ttk.Button(f, text="发送", width=10,
                       command=lambda m=ctrl_method, k=keys, s=short: self._do_pair(m, k, s)
                       ).pack(side="left", padx=8)

    def _build_impedance_section(self, parent):
        f = ttk.LabelFrame(parent, text="阻抗控制")
        f.pack(fill="x", padx=6, pady=5)

        r1 = ttk.Frame(f); r1.pack(fill="x")
        self._labeled_entries(r1, [("imp_o0", "原点电机0(rad)", "0.0"),
                                   ("imp_o1", "原点电机1(rad)", "0.0")])
        ttk.Button(r1, text="发送弹簧原点", width=14,
                   command=lambda: self._do_pair("send_impedance_origin",
                                                 ("imp_o0", "imp_o1"), "弹簧原点")
                   ).grid(row=0, column=4, padx=8)

        r2 = ttk.Frame(f); r2.pack(fill="x")
        self._labeled_entries(r2, [("imp_k", "刚度", "0.0"), ("imp_b", "阻尼", "0.0"),
                                   ("imp_j", "惯量", "0.0")])
        ttk.Button(r2, text="发送刚度/阻尼/惯量", width=18,
                   command=self._do_impedance).grid(row=0, column=6, padx=8)

    def _build_traj_section(self, parent):
        f = ttk.LabelFrame(parent, text="轨迹规划 (一帧内设定 速度上限/加速度上限/目标位置)")
        f.pack(fill="x", padx=6, pady=5)
        r = ttk.Frame(f); r.pack(side="left")
        self._labeled_entries(r, [
            ("tj_v", "vmax(rad/s)", "3.14"), ("tj_a", "amax(rad/s²)", "30.0"),
            ("tj_p0", "位置电机0(rad)", "0.0"), ("tj_p1", "位置电机1(rad)", "0.0"),
        ])
        ttk.Button(f, text="发送轨迹", width=10, command=self._do_traj).pack(side="left", padx=6)
        ttk.Button(f, text="仅更新位置", width=12,
                   command=self._do_traj_pos_only).pack(side="left", padx=2)

    def _build_pid_section(self, parent):
        f = ttk.LabelFrame(parent, text="PID 增益整定")
        f.pack(fill="x", padx=6, pady=5)
        rows = [
            ("位置环", [("pos_kp", "Kp", "0.0"), ("pos_ki", "Ki", "0.0")], "send_pos_pid"),
            ("速度环", [("vel_kp", "Kp", "0.0"), ("vel_ki", "Ki", "0.0")], "send_vel_pid"),
            ("电流环", [("cur_kp", "Kp", "0.0"), ("cur_ki", "Ki", "0.0")], "send_cur_pid"),
        ]
        for k, (label, entry_specs, ctrl_method) in enumerate(rows):
            ttk.Label(f, text=label, width=8).grid(row=k, column=0, padx=4, pady=3, sticky="w")
            sub = ttk.Frame(f)
            sub.grid(row=k, column=1, sticky="w")
            self._labeled_entries(sub, entry_specs)
            keys = (entry_specs[0][0], entry_specs[1][0])
            ttk.Button(f, text="发送", width=8,
                       command=lambda m=ctrl_method, kk=keys, lb=label: self._do_pid(m, kk, lb + "PID")
                       ).grid(row=k, column=2, padx=8)

    def _build_raw_section(self, parent):
        f = ttk.LabelFrame(parent, text="通用原始指令 (mode + 定点参数, 直接下发不做换算)")
        f.pack(fill="x", padx=6, pady=5)
        r = ttk.Frame(f); r.pack(side="left")
        self._labeled_entries(r, [("raw_mode", "mode", "6"),
                                  ("raw_p0", "电机0参数", "0"), ("raw_p1", "电机1参数", "0")])
        ttk.Button(f, text="发送原始指令", width=14,
                   command=self._do_raw).pack(side="left", padx=8)

    # ============================================================ 连接
    def _toggle_serial(self):
        if self.manager and self.manager.is_connected:
            self.manager.disconnect()
            self._ui_connected = False
            self.conn_btn.config(text="连接串口")
            self.status_label.config(text="状态: 未连接", foreground="red")
            self._reset_tree_nodata()
            return

        port = self.port_entry.get().strip()
        try:
            baud = int(self.baud_entry.get())
        except ValueError:
            messagebox.showerror("错误", "请输入有效的波特率数字")
            return

        self.manager = SerialManager(port, baud, self.config.serial.timeout)
        if self.manager.connect():
            self.commander = SerialCommander(
                self.manager,
                func=self.config.command_func,
                pad_canfd=self.config.resolved_pad_canfd(),
            )
            self.controller = HandController(self.manager, self.commander)
            self._ui_connected = True
            self.conn_btn.config(text="断开串口")
            self.status_label.config(text=f"状态: 已连接 {port}", foreground="green")
            self._log(f"已连接 {port} @ {baud}")
        else:
            messagebox.showerror("错误", f"无法连接到端口 {port}")

    # ============================================================ 目标解析
    def _targets(self):
        sel = self.target_combo.current()
        if sel == McuConfig.COUNT:  # 全部广播
            return list(range(McuConfig.COUNT))
        return [sel]

    def _motor_target(self):
        return (MotorTarget.BOTH, MotorTarget.MOTOR_0,
                MotorTarget.MOTOR_1)[self.motor_combo.current()]

    def _on_motor_change(self, event=None):
        t = self._motor_target()
        for k0, k1 in self._motor_entry_pairs:
            self.entries[k0].config(state="normal" if t.hits_motor0() else "disabled")
            self.entries[k1].config(state="normal" if t.hits_motor1() else "disabled")

    def _ensure_connected(self):
        if not (self.controller and self.manager and self.manager.is_connected):
            messagebox.showwarning("警告", "串口未连接")
            return False
        return True

    # ---- 输入读取 ----
    def _getf(self, key):
        return float(self.entries[key].get())

    def _geti(self, key):
        return int(float(self.entries[key].get()))

    def _read_pair(self, k0, k1):
        """按目标电机读取一对浮点；未选中侧返回 None。可能抛 ValueError。"""
        t = self._motor_target()
        v0 = self._getf(k0) if t.hits_motor0() else None
        v1 = self._getf(k1) if t.hits_motor1() else None
        return v0, v1

    def _read_pair_int(self, k0, k1):
        t = self._motor_target()
        v0 = self._geti(k0) if t.hits_motor0() else None
        v1 = self._geti(k1) if t.hits_motor1() else None
        return v0, v1

    # ============================================================ 指令回调
    def _do_state(self, state_key, label):
        if not self._ensure_connected():
            return
        results = self.controller.send_state(self._targets(), state_key, self._motor_target())
        self._show_results(f"状态→{label}", results)

    def _do_action(self, method_name, label):
        if not self._ensure_connected():
            return
        results = self.controller.send_action(self._targets(), method_name, self._motor_target())
        self._show_results(label, results)

    def _do_pair(self, ctrl_method, keys, short):
        if not self._ensure_connected():
            return
        try:
            v0, v1 = self._read_pair(keys[0], keys[1])
        except ValueError:
            messagebox.showwarning("输入错误", f"{short}: 请输入有效数字")
            return
        results = getattr(self.controller, ctrl_method)(self._targets(), v0, v1)
        self._show_results(f"{short} [{v0}, {v1}]", results)

    def _do_pid(self, ctrl_method, keys, label):
        if not self._ensure_connected():
            return
        try:
            kp, ki = self._getf(keys[0]), self._getf(keys[1])
        except ValueError:
            messagebox.showwarning("输入错误", f"{label}: 请输入有效数字")
            return
        results = getattr(self.controller, ctrl_method)(
            self._targets(), kp, ki, self._motor_target())
        self._show_results(f"{label} [kp={kp}, ki={ki}]", results)

    def _do_impedance(self):
        if not self._ensure_connected():
            return
        try:
            k, b, j = self._getf("imp_k"), self._getf("imp_b"), self._getf("imp_j")
        except ValueError:
            messagebox.showwarning("输入错误", "阻抗参数: 请输入有效数字")
            return
        results = self.controller.send_impedance_params(
            self._targets(), k, b, j, self._motor_target())
        self._show_results(f"阻抗 [k={k}, b={b}, j={j}]", results)

    def _do_traj(self):
        if not self._ensure_connected():
            return
        try:
            v, a = self._getf("tj_v"), self._getf("tj_a")
            p0, p1 = self._read_pair("tj_p0", "tj_p1")
        except ValueError:
            messagebox.showwarning("输入错误", "轨迹参数: 请输入有效数字")
            return
        results = self.controller.send_trajectory(self._targets(), v, a, p0, p1)
        self._show_results(f"轨迹 [v={v}, a={a}, p=({p0},{p1})]", results)

    def _do_traj_pos_only(self):
        if not self._ensure_connected():
            return
        try:
            p0, p1 = self._read_pair("tj_p0", "tj_p1")
        except ValueError:
            messagebox.showwarning("输入错误", "轨迹位置: 请输入有效数字")
            return
        results = self.controller.send_trajectory_pos(self._targets(), p0, p1)
        self._show_results(f"轨迹位置 [{p0}, {p1}]", results)

    def _do_raw(self):
        if not self._ensure_connected():
            return
        try:
            mode = self._geti("raw_mode")
            p0, p1 = self._read_pair_int("raw_p0", "raw_p1")
        except ValueError:
            messagebox.showwarning("输入错误", "原始指令: 请输入有效整数")
            return
        results = self.controller.send_raw(self._targets(), mode, p0, p1)
        self._show_results(f"RAW mode={mode} [{p0}, {p1}]", results)

    # ============================================================ 日志
    def _show_results(self, desc, results):
        mtag = self._motor_target().tag
        for r in results:
            if r.message:
                self._log(f"[ERR] MCU{r.mcu_index}/{mtag} {desc}: {r.message}")
            else:
                self._log(f"[{'OK ' if r.ok else 'ERR'}] MCU{r.mcu_index}/{mtag} {desc}")

    def _log(self, msg):
        self.log_text.config(state="normal")
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)
        if int(self.log_text.index("end-1c").split(".")[0]) > self.config.log_max_lines:
            self.log_text.delete("1.0", "200.0")
        self.log_text.config(state="disabled")

    # ============================================================ 刷新
    @staticmethod
    def _fmt(v):
        return f"{v:.3f}" if v is not None else "—"

    def _reset_tree_nodata(self):
        for i in range(McuConfig.COUNT):
            self.tree.item(f"mcu{i}", values=("", "", "", "", "", "无数据"),
                           tags=("mcu", "nodata"))
            for m in range(McuConfig.MOTORS_PER_MCU):
                self.tree.item(f"mcu{i}_m{m}", values=("—", "—", "—", "—", "—", ""),
                               tags=("nodata",))

    def _update_loop(self):
        mgr = self.manager
        if mgr and mgr.is_connected:
            self._ui_connected = True
            for i in range(McuConfig.COUNT):
                frame, stats = mgr.get_snapshot(i)   # 一致快照

                if frame is None:
                    meta, ptag = "无数据", "nodata"
                else:
                    lr = stats.loss_rate
                    if lr > self.config.loss_err_threshold:
                        ptag = "err"
                    elif lr > self.config.loss_warn_threshold:
                        ptag = "warn"
                    else:
                        ptag = "ok"
                    meta = (f"丢包 {lr:.1f}% "
                            f"(收:{stats.last_sec_received} 丢:{stats.last_sec_lost})")

                self.tree.item(f"mcu{i}", values=("", "", "", "", "", meta),
                               tags=("mcu", ptag))

                if frame is None:
                    for m in range(McuConfig.MOTORS_PER_MCU):
                        self.tree.item(f"mcu{i}_m{m}",
                                       values=("—", "—", "—", "—", "—", ""), tags=("nodata",))
                    continue

                for m in range(McuConfig.MOTORS_PER_MCU):
                    md = frame.motors[m] if m < len(frame.motors) else None
                    if md is None:
                        self.tree.item(f"mcu{i}_m{m}",
                                       values=("—", "—", "—", "—", "—", ""), tags=("nodata",))
                        continue
                    self.tree.item(
                        f"mcu{i}_m{m}",
                        values=(
                            md.state_name,
                            self._fmt(md.theta),
                            self._fmt(md.omega),
                            self._fmt(md.acl),
                            self._fmt(md.torque),
                            "",
                        ),
                        tags=(ptag,),
                    )

        elif self._ui_connected:
            # 接收线程因异常自动断开 -> 同步 GUI
            self._ui_connected = False
            self.conn_btn.config(text="连接串口")
            self.status_label.config(text="状态: 串口连接已断开", foreground="red")
            self._reset_tree_nodata()
            self._log("[ERR] 串口连接已断开")

        self.root.after(self.config.refresh_ms, self._update_loop)
