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
from sdk.protocol import McuConfig


class ScrollableFrame(ttk.Frame):
    """带竖向滚动条的容器，内部控件放到 self.body。"""

    def __init__(self, parent, **kw):
        """构建可竖向滚动的容器；调用方把子控件放进 self.body。绑定滚轮/尺寸自适应。"""
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
        """鼠标滚轮事件回调：按滚动量滚动画布 (仅光标在容器内时绑定)。"""
        self.canvas.yview_scroll(int(-event.delta / 120), "units")


class HighDofHandGUI:

    # 监控树的行占位。列顺序 = 状态/错误/θ/ω/a/τ/链路，加减列时改这里即可
    MOTOR_ROW_NODATA = ("—", "—", "—", "—", "—", "—", "")

    # 状态机派发按钮: (显示名, 状态键名)
    STATE_BUTTONS = [
        ("就绪", "StartupReady"),
        ("初始角标定", "StartupElecAngleDrag"),
        ("电流环", "AppCurrentCtrl"),
        ("速度环", "AppVelocityCtrl"),
        ("位置环", "AppPositionCtrl"),
        ("力矩环", "AppTorqueCtrl"),
        ("MIT控制", "AppMITCtrl"),
        ("失能", "AppDisable"),
    ]

    def __init__(self, root, config=None):
        """
        初始化主窗口并挂上定时刷新。
        参数: root=Tk 根窗口；config=AppConfig (缺省则用默认配置)。
        用法: HighDofHandGUI(tk.Tk(), config=AppConfig()) 后调用 root.mainloop()。
        """
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
        """搭建整体界面：顶部串口设置栏 + "实时监控 / 指令控制"两个标签页。"""
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
    @staticmethod
    def _mcu_row(meta):
        """MCU 父行的 values：只有最后的"链路"列有内容，前面各列留空。"""
        return ("",) * (len(HighDofHandGUI.MOTOR_ROW_NODATA) - 1) + (meta,)

    def _build_monitor_tab(self):
        """构建实时监控页：一棵 Treeview，MCU 为父节点、电机为子节点，预建所有行待刷新。"""
        info = ttk.Label(
            self.tab_monitor,
            text="8 个 MCU (0xB0~0xB7)，每个 MCU 管控 2 个电机，"
                 "每个电机 5 条低速曲线 (信息字 / θ / ω / 加速度 / 力矩)。"
                 "信息字高字节是状态机状态码 (状态列)、低 24 位是错误位域 (错误列)："
                 "可同时报多个错误，置位后一直保持，需在指令控制页按段显式清错。",
            foreground="#555",
        )
        info.pack(fill="x", padx=6, pady=(6, 2))

        cols = ("state", "error", "theta", "omega", "acl", "torque", "meta")
        tree = ttk.Treeview(self.tab_monitor, columns=cols, show="tree headings", height=26)
        tree.heading("#0", text="设备 / 电机")
        tree.heading("state", text="状态")
        tree.heading("error", text="错误")
        tree.heading("theta", text="θ (rad)")
        tree.heading("omega", text="ω (rad/s)")
        tree.heading("acl", text="a (rad/s²)")
        tree.heading("torque", text="τ (mN·m)")
        tree.heading("meta", text="链路 / 丢包")

        tree.column("#0", width=170, anchor="w")
        tree.column("state", width=160, anchor="center")
        # 多个错误并列显示，这一列要足够宽
        tree.column("error", width=280, anchor="w")
        tree.column("theta", width=100, anchor="e")
        tree.column("omega", width=100, anchor="e")
        tree.column("acl", width=100, anchor="e")
        tree.column("torque", width=100, anchor="e")
        tree.column("meta", width=190, anchor="center")

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
                        values=self._mcu_row("无数据"), tags=("mcu", "nodata"), open=True)
            for m in range(McuConfig.MOTORS_PER_MCU):
                tree.insert(pid, "end", iid=f"{pid}_m{m}", text=f"    电机{m}",
                            values=self.MOTOR_ROW_NODATA, tags=("nodata",))

    # ---------------------------------------------------------- 控制页
    def _build_control_tab(self):
        """构建指令控制页：顶部目标 MCU/电机选择，中部各控制分区，底部发送日志。"""
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
        ttk.Label(top, text="  (发送的数值按此目标路由到 电机0 / 电机1 / 两者)",
                  foreground="#777").pack(side="left")

        paned = ttk.Panedwindow(self.tab_control, orient="vertical")
        paned.pack(fill="both", expand=True, padx=6, pady=4)

        scroll = ScrollableFrame(paned)
        paned.add(scroll, weight=3)
        body = scroll.body

        self._build_state_section(body)
        self._build_action_section(body)
        self._build_flashing_section(body)
        self._build_value_sections(body)
        self._build_mit_section(body)
        self._build_traj_section(body)
        self._build_homing_section(body)
        self._build_pid_section(body)

        logf = ttk.LabelFrame(paned, text="发送日志")
        paned.add(logf, weight=1)
        self.log_text = tk.Text(logf, height=7, state="disabled",
                                background="#101418", foreground="#c8e6c9")
        self.log_text.pack(fill="both", expand=True, padx=4, pady=4)

    def _build_state_section(self, parent):
        """状态机派发分区：按 STATE_BUTTONS 表铺一排按钮，点击调用 _do_state。"""
        f = ttk.LabelFrame(parent, text="状态机派发 (DispatchMotorStateMachine)")
        f.pack(fill="x", padx=6, pady=5)
        for k, (label, state_key) in enumerate(self.STATE_BUTTONS):
            b = ttk.Button(f, text=label, width=12,
                           command=lambda sk=state_key, lb=label: self._do_state(sk, lb))
            b.grid(row=k // 4, column=k % 4, padx=4, pady=4, sticky="ew")
        for c in range(4):
            f.columnconfigure(c, weight=1)

    def _build_action_section(self, parent):
        """App 动作分区：(按钮文案, commander 方法名) 表驱动，点击调用 _do_action。"""
        f = ttk.LabelFrame(parent, text="App 动作")
        f.pack(fill="x", padx=6, pady=5)
        # 后五条按信息字的分段清错：轨迹 bit0~7 / 内环 bit8~11 / 外环 bit12~15 /
        # 编码器 bit16~19。Flash 错误不在信息字里，是 Flash 设备自己的状态
        actions = [
            ("轨迹初始化", "send_traj_init"),
            ("轨迹反初始化", "send_traj_deinit"),
            ("清除轨迹错误", "send_clear_traj_error"),
            ("清除内环错误", "send_clear_inner_error"),
            ("清除外环错误", "send_clear_outer_error"),
            ("清除编码器错误", "send_clear_encoder_error"),
            ("清除Flash错误", "send_clear_flash_error"),
        ]
        for k, (label, fn) in enumerate(actions):
            b = ttk.Button(f, text=label, width=14,
                           command=lambda fn=fn, lb=label: self._do_action(fn, lb))
            b.grid(row=k // 3, column=k % 3, padx=4, pady=4, sticky="ew")
        for c in range(3):
            f.columnconfigure(c, weight=1)

    def _build_flashing_section(self, parent):
        """刷写参数分区：输入配置序号 config_index，按其重置控制参数为预设并刷入 Flash。"""
        f = ttk.LabelFrame(parent, text="刷写参数 (FlashingParams：按配置序号重置控制参数并刷入 Flash)")
        f.pack(fill="x", padx=6, pady=5)
        row = ttk.Frame(f)
        row.pack(side="left")
        self._labeled_entries(row, [("flash_cfg", "配置序号", "0")])
        ttk.Button(f, text="刷写参数", width=12,
                   command=self._do_flashing_params).pack(side="left", padx=8)

    def _labeled_entries(self, parent, specs):
        """
        在一行里批量铺"标签 + 输入框"，并把输入框登记进 self.entries[key]。
        specs: [(key, 标签文案, 默认值), ...]。供各控制分区复用。
        """
        for i, (key, label, default) in enumerate(specs):
            ttk.Label(parent, text=label).grid(row=0, column=i * 2, padx=(6, 2), pady=4, sticky="e")
            e = ttk.Entry(parent, width=9)
            e.insert(0, str(default))
            e.grid(row=0, column=i * 2 + 1, padx=(0, 6), pady=4, sticky="w")
            self.entries[key] = e

    def _build_value_sections(self, parent):
        """位置/速度/力矩/电流四个"单值 + 发送"分区，表驱动生成，按目标电机路由 (_do_value)。"""
        specs = [
            ("位置指令 (Theta_Gear, rad)", "pos", "0.0", "send_position", "位置"),
            ("速度指令 (Omega_Gear, rad/s)", "vel", "0.0", "send_velocity", "速度"),
            ("力矩指令 (Torque_Gear, mN·m)", "tau", "0.0", "send_torque", "力矩"),
            ("电流指令 (Iq, A)", "iq", "0.0", "send_iq", "电流Iq"),
        ]
        for title, key, default, ctrl_method, short in specs:
            f = ttk.LabelFrame(parent, text=title)
            f.pack(fill="x", padx=6, pady=5)
            row = ttk.Frame(f)
            row.pack(side="left")
            self._labeled_entries(row, [(key, "数值", default)])
            ttk.Button(f, text="发送", width=10,
                       command=lambda m=ctrl_method, k=key, s=short: self._do_value(m, k, s)
                       ).pack(side="left", padx=8)

    def _build_mit_section(self, parent):
        """
        MIT 控制分区 (原"阻抗控制")：一行发位置/速度/扭矩前馈三条单值指令 (_do_value)，
        一行发刚度/阻尼/惯量 (_do_mit_params)。全部是【输出轴】量。
        位置/速度与位置环、速度环共用固件的同一条出端通道，只是命令码不同；
        三个系数只在 AppMITCtrl 状态下被固件接收，其余状态整条丢弃。
        """
        f = ttk.LabelFrame(parent, text="MIT 控制 (出端量：位置/速度/扭矩前馈 + 刚度/阻尼/惯量)")
        f.pack(fill="x", padx=6, pady=5)

        r1 = ttk.Frame(f); r1.pack(fill="x")
        self._labeled_entries(r1, [("mit_p", "位置(rad)", "0.0"), ("mit_v", "速度(rad/s)", "0.0"),
                                   ("mit_t", "扭矩前馈(mN·m)", "0.0")])
        ttk.Button(r1, text="发位置", width=8,
                   command=lambda: self._do_value("send_mit_position", "mit_p", "MIT位置")
                   ).grid(row=0, column=6, padx=(8, 2))
        ttk.Button(r1, text="发速度", width=8,
                   command=lambda: self._do_value("send_mit_velocity", "mit_v", "MIT速度")
                   ).grid(row=0, column=7, padx=2)
        ttk.Button(r1, text="发扭矩前馈", width=12,
                   command=lambda: self._do_value("send_mit_torque_inject", "mit_t", "MIT扭矩前馈")
                   ).grid(row=0, column=8, padx=2)

        r2 = ttk.Frame(f); r2.pack(fill="x")
        self._labeled_entries(r2, [("mit_k", "刚度(mN·m/rad)", "0.0"),
                                   ("mit_b", "阻尼(mN·m·s/rad)", "0.0"),
                                   ("mit_j", "惯量(mN·m·s²/rad)", "0.0")])
        ttk.Button(r2, text="发送刚度/阻尼/惯量", width=18,
                   command=self._do_mit_params).grid(row=0, column=6, padx=8)

    def _build_traj_section(self, parent):
        """
        轨迹分区：vmax/amax/位置三个输入 + 五个按钮，按"这一帧下发哪几条"划分：
        发送轨迹 = vmax+amax+位置 / 仅更新位置 / 仅更新限幅 = vmax+amax / 仅更新vmax / 仅更新amax。
        限幅只在轨迹 Idle 时被固件接收，且到下一条位置指令才参与重新规划。
        """
        f = ttk.LabelFrame(parent, text="轨迹规划 (一帧内设定 速度上限/加速度上限/目标位置)")
        f.pack(fill="x", padx=6, pady=5)
        r = ttk.Frame(f); r.pack(side="left")
        self._labeled_entries(r, [
            ("tj_v", "vmax(rad/s)", "3.14"), ("tj_a", "amax(rad/s²)", "30.0"),
            ("tj_p", "位置(rad)", "0.0"),
        ])
        ttk.Button(f, text="发送轨迹", width=10, command=self._do_traj).pack(side="left", padx=6)
        ttk.Button(f, text="仅更新位置", width=12,
                   command=self._do_traj_pos_only).pack(side="left", padx=2)
        ttk.Button(f, text="仅更新限幅", width=12,
                   command=self._do_traj_limits_only).pack(side="left", padx=2)
        ttk.Button(f, text="仅更新vmax", width=12,
                   command=self._do_traj_vel_only).pack(side="left", padx=2)
        ttk.Button(f, text="仅更新amax", width=12,
                   command=self._do_traj_acl_only).pack(side="left", padx=2)

    def _build_homing_section(self, parent):
        """回零分区：前/反向力矩与前/反向位置四输入 + "更新参数并回零"/"仅更新参数"/"仅回零" 三按钮。"""
        f = ttk.LabelFrame(parent, text="回零 Homing (前/反向力矩 · 前/反向位置)")
        f.pack(fill="x", padx=6, pady=5)
        r = ttk.Frame(f); r.pack(side="left")
        self._labeled_entries(r, [
            ("hm_tau_f", "前向力矩(mN·m)", "0.0"), ("hm_tau_b", "反向力矩(mN·m)", "0.0"),
            ("hm_pos_f", "前向位置(rad)", "0.0"), ("hm_pos_b", "反向位置(rad)", "0.0"),
        ])
        ttk.Button(f, text="更新参数并回零", width=14, command=self._do_homing_full).pack(side="left", padx=6)
        ttk.Button(f, text="仅更新参数", width=12, command=self._do_homing_params).pack(side="left", padx=2)
        ttk.Button(f, text="仅回零", width=10, command=self._do_homing).pack(side="left", padx=2)

    def _build_pid_section(self, parent):
        """PID 分区：位置/速度/电流环各一行 (Kp/Ki + 发送)，表驱动，发送走 _do_pid。"""
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

    # ============================================================ 连接
    def _toggle_serial(self):
        """"连接/断开"按钮回调：已连则断开并复位界面；未连则读端口/波特率、建三层对象并连接。"""
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

        self.manager = SerialManager(port, baud, self.config.serial.timeout,
                                     curve_count=self.config.feedback_curve_count)
        if self.manager.connect():
            self.commander = SerialCommander(
                self.manager,
                func=self.config.command_func,
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
        """把"目标 MCU"下拉框解析成 index 列表：选"全部广播"返回 0~COUNT-1，否则返回单个。"""
        sel = self.target_combo.current()
        if sel == McuConfig.COUNT:  # 全部广播
            return list(range(McuConfig.COUNT))
        return [sel]

    def _motor_target(self):
        """把"目标电机"下拉框解析成 MotorTarget (两个电机/仅电机0/仅电机1)。"""
        return (MotorTarget.BOTH, MotorTarget.MOTOR_0,
                MotorTarget.MOTOR_1)[self.motor_combo.current()]

    def _ensure_connected(self):
        """每个发送回调开头调用：未连接则弹警告返回 False，阻止后续下发。"""
        if not (self.controller and self.manager and self.manager.is_connected):
            messagebox.showwarning("警告", "串口未连接")
            return False
        return True

    # ---- 输入读取 ----
    def _getf(self, key):
        """读取 entries[key] 并转 float (非法输入抛 ValueError，由回调统一捕获提示)。"""
        return float(self.entries[key].get())

    # ============================================================ 指令回调
    # 各 _do_* 为按钮回调，套路一致：确认已连接 -> (读输入) -> 调 controller -> _show_results。
    def _do_state(self, state_key, label):
        """状态机按钮回调：向目标 MCU/电机派发状态 state_key，label 仅用于日志。"""
        if not self._ensure_connected():
            return
        results = self.controller.send_state(self._targets(), state_key, self._motor_target())
        self._show_results(f"状态→{label}", results)

    def _do_action(self, method_name, label):
        """App 动作按钮回调：以 method_name 分派 commander 的无参数动作。"""
        if not self._ensure_connected():
            return
        results = self.controller.send_action(self._targets(), method_name, self._motor_target())
        self._show_results(label, results)

    def _do_flashing_params(self):
        """刷写参数回调：读配置序号 (整数)，按目标电机重置控制参数为预设并刷入 Flash。"""
        if not self._ensure_connected():
            return
        try:
            cfg = int(self.entries["flash_cfg"].get())
        except ValueError:
            messagebox.showwarning("输入错误", "配置序号: 请输入有效整数")
            return
        results = self.controller.send_flashing_params(
            self._targets(), cfg, self._motor_target())
        self._show_results(f"刷写参数 [cfg={cfg}]", results)

    def _do_value(self, ctrl_method, key, short):
        """单值发送回调：读一个浮点，按目标电机调 controller.<ctrl_method>。"""
        if not self._ensure_connected():
            return
        try:
            v = self._getf(key)
        except ValueError:
            messagebox.showwarning("输入错误", f"{short}: 请输入有效数字")
            return
        results = getattr(self.controller, ctrl_method)(
            self._targets(), v, self._motor_target())
        self._show_results(f"{short} [{v}]", results)

    def _do_pid(self, ctrl_method, keys, label):
        """PID 发送回调：读 keys 的 kp/ki，调 controller.<ctrl_method>。"""
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

    def _do_mit_params(self):
        """MIT 系数发送回调：读刚度/阻尼/惯量 (出端量) 并下发到目标电机。"""
        if not self._ensure_connected():
            return
        try:
            k, b, j = self._getf("mit_k"), self._getf("mit_b"), self._getf("mit_j")
        except ValueError:
            messagebox.showwarning("输入错误", "MIT 系数: 请输入有效数字")
            return
        results = self.controller.send_mit_params(
            self._targets(), k, b, j, self._motor_target())
        self._show_results(f"MIT [k={k}, b={b}, j={j}]", results)

    def _do_traj(self):
        """轨迹发送回调：读 vmax/amax 与逐电机目标位置，下发完整轨迹指令。"""
        if not self._ensure_connected():
            return
        try:
            v, a = self._getf("tj_v"), self._getf("tj_a")
            p = self._getf("tj_p")
        except ValueError:
            messagebox.showwarning("输入错误", "轨迹参数: 请输入有效数字")
            return
        results = self.controller.send_trajectory(self._targets(), v, a, p, self._motor_target())
        self._show_results(f"轨迹 [v={v}, a={a}, p={p}]", results)

    def _do_traj_pos_only(self):
        """"仅更新位置"回调：只发轨迹目标位置，沿用上一帧 vmax/amax。"""
        if not self._ensure_connected():
            return
        try:
            p = self._getf("tj_p")
        except ValueError:
            messagebox.showwarning("输入错误", "轨迹位置: 请输入有效数字")
            return
        results = self.controller.send_trajectory_pos(self._targets(), p, self._motor_target())
        self._show_results(f"轨迹位置 [{p}]", results)

    def _do_traj_limits_only(self):
        """"仅更新限幅"回调：只发 vmax/amax，沿用上一帧目标位置。"""
        if not self._ensure_connected():
            return
        try:
            v, a = self._getf("tj_v"), self._getf("tj_a")
        except ValueError:
            messagebox.showwarning("输入错误", "轨迹限幅: 请输入有效数字")
            return
        results = self.controller.send_trajectory_limits(self._targets(), v, a, self._motor_target())
        self._show_results(f"轨迹限幅 [v={v}, a={a}]", results)

    def _homing_inputs(self):
        """读回零四参数 (前向力矩/反向力矩/前向位置/反向位置)；输入非法则弹提示并返回 None。"""
        try:
            return (self._getf("hm_tau_f"), self._getf("hm_tau_b"),
                    self._getf("hm_pos_f"), self._getf("hm_pos_b"))
        except ValueError:
            messagebox.showwarning("输入错误", "回零参数: 请输入有效数字")
            return None

    def _do_traj_vel_only(self):
        """"仅更新vmax"回调：只发速度上限，沿用上一帧 amax 与目标位置。"""
        if not self._ensure_connected():
            return
        try:
            v = self._getf("tj_v")
        except ValueError:
            messagebox.showwarning("输入错误", "轨迹速度: 请输入有效数字")
            return
        results = self.controller.send_trajectory_vel_max(
            self._targets(), v, self._motor_target())
        self._show_results(f"轨迹vmax [{v}]", results)

    def _do_traj_acl_only(self):
        """"仅更新amax"回调：只发加速度上限，沿用上一帧 vmax 与目标位置。"""
        if not self._ensure_connected():
            return
        try:
            a = self._getf("tj_a")
        except ValueError:
            messagebox.showwarning("输入错误", "轨迹加速度: 请输入有效数字")
            return
        results = self.controller.send_trajectory_acl_max(self._targets(), a, self._motor_target())
        self._show_results(f"轨迹amax [{a}]", results)

    def _do_homing_full(self):
        """"更新参数并回零"回调：一帧内下发回零四参数并触发回零。"""
        if not self._ensure_connected():
            return
        vals = self._homing_inputs()
        if vals is None:
            return
        tf, tb, pf, pb = vals
        results = self.controller.send_homing_full(self._targets(), tf, tb, pf, pb,
                                                   self._motor_target())
        self._show_results(f"回零(参数+执行) [τ={tf}/{tb}, pos={pf}/{pb}]", results)

    def _do_homing_params(self):
        """"仅更新参数"回调：只发回零四参数，不触发回零。"""
        if not self._ensure_connected():
            return
        vals = self._homing_inputs()
        if vals is None:
            return
        tf, tb, pf, pb = vals
        results = self.controller.send_homing_params(self._targets(), tf, tb, pf, pb,
                                                     self._motor_target())
        self._show_results(f"回零参数 [τ={tf}/{tb}, pos={pf}/{pb}]", results)

    def _do_homing(self):
        """"仅回零"回调：只触发回零，沿用上一帧参数。"""
        if not self._ensure_connected():
            return
        results = self.controller.send_action(self._targets(), "send_homing", self._motor_target())
        self._show_results("回零(执行)", results)

    # ============================================================ 日志
    def _show_results(self, desc, results):
        """把 controller 返回的 list[CommandResult] 逐条按 [OK/ERR] MCUx/电机 desc 写入日志。"""
        mtag = self._motor_target().tag
        for r in results:
            if r.message:
                self._log(f"[ERR] MCU{r.mcu_index}/{mtag} {desc}: {r.message}")
            else:
                self._log(f"[{'OK ' if r.ok else 'ERR'}] MCU{r.mcu_index}/{mtag} {desc}")

    def _log(self, msg):
        """向日志框追加一行并滚到底；超过 log_max_lines 时截掉最旧的若干行。"""
        self.log_text.config(state="normal")
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)
        if int(self.log_text.index("end-1c").split(".")[0]) > self.config.log_max_lines:
            self.log_text.delete("1.0", "200.0")
        self.log_text.config(state="disabled")

    # ============================================================ 刷新
    @staticmethod
    def _fmt(v):
        """把浮点反馈格式化成 3 位小数字符串；None 显示为破折号。"""
        return f"{v:.3f}" if v is not None else "—"

    def _reset_tree_nodata(self):
        """把监控树所有行复位成"无数据"占位 (断开或清屏时用)。"""
        for i in range(McuConfig.COUNT):
            self.tree.item(f"mcu{i}", values=self._mcu_row("无数据"),
                           tags=("mcu", "nodata"))
            for m in range(McuConfig.MOTORS_PER_MCU):
                self.tree.item(f"mcu{i}_m{m}", values=self.MOTOR_ROW_NODATA,
                               tags=("nodata",))

    def _update_loop(self):
        """
        定时刷新回调 (每 config.refresh_ms 由 root.after 触发一次)：
        取每个 MCU 的一致快照刷新监控树与丢包着色；若接收线程已异常断开则同步界面。
        末尾重新排下一次 after，形成循环。
        """
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

                self.tree.item(f"mcu{i}", values=self._mcu_row(meta),
                               tags=("mcu", ptag))

                if frame is None:
                    for m in range(McuConfig.MOTORS_PER_MCU):
                        self.tree.item(f"mcu{i}_m{m}",
                                       values=self.MOTOR_ROW_NODATA, tags=("nodata",))
                    continue

                for m in range(McuConfig.MOTORS_PER_MCU):
                    md = frame.motors[m] if m < len(frame.motors) else None
                    if md is None:
                        self.tree.item(f"mcu{i}_m{m}",
                                       values=self.MOTOR_ROW_NODATA, tags=("nodata",))
                        continue
                    self.tree.item(
                        f"mcu{i}_m{m}",
                        values=(
                            md.state_text,
                            md.error_text,
                            self._fmt(md.theta),
                            self._fmt(md.omega),
                            self._fmt(md.acl),
                            self._fmt(md.torque),
                            "",
                        ),
                        # 有错误置位就标红，压过按丢包率算出来的颜色
                        tags=("err" if md.has_error else ptag,),
                    )

        elif self._ui_connected:
            # 接收线程因异常自动断开 -> 同步 GUI
            self._ui_connected = False
            self.conn_btn.config(text="连接串口")
            self.status_label.config(text="状态: 串口连接已断开", foreground="red")
            self._reset_tree_nodata()
            self._log("[ERR] 串口连接已断开")

        self.root.after(self.config.refresh_ms, self._update_loop)


def main():
    """
    程序入口：创建 Tk 根窗口、用默认 AppConfig 启动 GUI 并进入事件循环。

    用法：命令行执行 `python high_dof_gui.py`。
    """
    root = tk.Tk()
    HighDofHandGUI(root, config=AppConfig())
    root.mainloop()


if __name__ == "__main__":
    main()
