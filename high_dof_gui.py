import tkinter as tk
from tkinter import ttk, messagebox
from functools import partial

from sdk.serial_manager import SerialManager


class ScrollableFrame(ttk.Frame):
    """一个带竖向滚动条的容器，内部控件放到 self.body 里。"""

    def __init__(self, parent, **kw):
        super().__init__(parent, **kw)
        self.canvas = tk.Canvas(self, borderwidth=0, highlightthickness=0)
        vsb = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=vsb.set)

        vsb.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)

        self.body = ttk.Frame(self.canvas)
        self._win = self.canvas.create_window((0, 0), window=self.body, anchor="nw")

        self.body.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfigure(self._win, width=e.width))
        # 鼠标滚轮 (Windows)
        self.canvas.bind("<Enter>", lambda e: self.canvas.bind_all("<MouseWheel>", self._on_wheel))
        self.canvas.bind("<Leave>", lambda e: self.canvas.unbind_all("<MouseWheel>"))

    def _on_wheel(self, event):
        self.canvas.yview_scroll(int(-event.delta / 120), "units")


class HighDofHandGUI:
    """高自由度机械手 上位机: 8 个 MCU x 2 电机 的监控与控制台。"""

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

    def __init__(self, root):
        self.root = root
        self.root.title("高自由度机械手 · MCU 监控与控制台")
        self.root.geometry("1180x840")

        self.serial_mgr = None
        self.entries = {}          # 控制页输入框引用
        self._ui_connected = False  # UI 认为的连接态(用于检测后台自动断开)

        self._setup_ui()
        self.root.after(50, self._update_loop)

    # ============================================================ UI
    def _setup_ui(self):
        # ---- 串口配置条 ----
        bar = ttk.LabelFrame(self.root, text="串口设置")
        bar.pack(fill="x", padx=10, pady=6)

        ttk.Label(bar, text="端口:").pack(side="left", padx=(8, 2))
        self.port_entry = ttk.Entry(bar, width=9)
        self.port_entry.insert(0, "COM3")
        self.port_entry.pack(side="left", padx=2)

        ttk.Label(bar, text="波特率:").pack(side="left", padx=(8, 2))
        self.baud_entry = ttk.Entry(bar, width=9)
        self.baud_entry.insert(0, "115200")
        self.baud_entry.pack(side="left", padx=2)

        self.conn_btn = ttk.Button(bar, text="连接串口", command=self._toggle_serial)
        self.conn_btn.pack(side="left", padx=10)

        self.status_label = ttk.Label(bar, text="状态: 未连接", foreground="red")
        self.status_label.pack(side="left", padx=6)

        # ---- 主选项卡 ----
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
            text="8 个 MCU (0xB0~0xB7)，每个 MCU 管控 2 个电机，每个电机 5 条低速曲线 (状态机 / θ / ω / 加速度 / 力矩)。",
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

        # 预建 8 个 MCU 节点，每个 2 个电机子节点
        for i in range(SerialManager.NUM_MCU):
            mcu_id = SerialManager.MCU_IDS[i]
            pid = f"mcu{i}"
            tree.insert("", "end", iid=pid, text=f"MCU{i}  (0x{mcu_id:02X})",
                        values=("", "", "", "", "", "无数据"), tags=("mcu", "nodata"), open=True)
            for m in range(SerialManager.MOTORS_PER_MCU):
                tree.insert(pid, "end", iid=f"{pid}_m{m}", text=f"    电机{m}",
                            values=("—", "—", "—", "—", "—", ""), tags=("nodata",))

    # ---------------------------------------------------------- 控制页
    def _build_control_tab(self):
        # 目标 MCU 选择
        top = ttk.Frame(self.tab_control)
        top.pack(fill="x", padx=6, pady=6)
        ttk.Label(top, text="目标 MCU:").pack(side="left", padx=(4, 4))
        target_values = [f"MCU{i} (0x{SerialManager.MCU_IDS[i]:02X})" for i in range(SerialManager.NUM_MCU)]
        target_values.append("全部广播")
        self.target_combo = ttk.Combobox(top, values=target_values, width=16, state="readonly")
        self.target_combo.current(0)
        self.target_combo.pack(side="left")
        ttk.Label(top, text="  (指令同时下发到该 MCU 的 电机0 与 电机1)", foreground="#777").pack(side="left")

        # 上: 可滚动指令区   下: 日志
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

        # 日志
        logf = ttk.LabelFrame(paned, text="发送日志")
        paned.add(logf, weight=1)
        self.log_text = tk.Text(logf, height=7, state="disabled", background="#101418", foreground="#c8e6c9")
        self.log_text.pack(fill="both", expand=True, padx=4, pady=4)

    def _build_state_section(self, parent):
        f = ttk.LabelFrame(parent, text="状态机派发 (DispatchMotorStateMachine)")
        f.pack(fill="x", padx=6, pady=5)
        for k, (label, state_key) in enumerate(self.STATE_BUTTONS):
            b = ttk.Button(f, text=label, width=12,
                           command=partial(self._do_state, state_key, label))
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
                           command=partial(self._do_simple, fn, label))
            b.grid(row=k // 3, column=k % 3, padx=4, pady=4, sticky="ew")
        for c in range(3):
            f.columnconfigure(c, weight=1)

    def _labeled_entries(self, parent, specs):
        """specs: [(key, label, default)]，在 parent 里横排放置 label+entry。返回该 frame。"""
        for i, (key, label, default) in enumerate(specs):
            ttk.Label(parent, text=label).grid(row=0, column=i * 2, padx=(6, 2), pady=4, sticky="e")
            e = ttk.Entry(parent, width=9)
            e.insert(0, str(default))
            e.grid(row=0, column=i * 2 + 1, padx=(0, 6), pady=4, sticky="w")
            self.entries[key] = e

    def _build_value_sections(self, parent):
        # 位置 / 速度 / 力矩 / 电流Iq  —— 每个电机独立输入
        specs = [
            ("位置指令 (Theta_Gear, rad)", [("pos0", "电机0", "0.0"), ("pos1", "电机1", "0.0")], "send_position", ("pos0", "pos1")),
            ("速度指令 (Omega_Gear, rad/s)", [("vel0", "电机0", "0.0"), ("vel1", "电机1", "0.0")], "send_velocity", ("vel0", "vel1")),
            ("力矩指令 (Torque_Gear, N·m)", [("tau0", "电机0", "0.0"), ("tau1", "电机1", "0.0")], "send_torque", ("tau0", "tau1")),
            ("电流指令 (Iq, A)", [("iq0", "电机0", "0.0"), ("iq1", "电机1", "0.0")], "send_iq", ("iq0", "iq1")),
        ]
        for title, entry_specs, fn, keys in specs:
            f = ttk.LabelFrame(parent, text=title)
            f.pack(fill="x", padx=6, pady=5)
            row = ttk.Frame(f)
            row.pack(side="left")
            self._labeled_entries(row, entry_specs)
            ttk.Button(f, text="发送", width=10,
                       command=partial(self._do_paired, fn, keys, title)).pack(side="left", padx=8)

    def _build_impedance_section(self, parent):
        f = ttk.LabelFrame(parent, text="阻抗控制")
        f.pack(fill="x", padx=6, pady=5)

        r1 = ttk.Frame(f); r1.pack(fill="x")
        self._labeled_entries(r1, [("imp_o0", "原点电机0(rad)", "0.0"), ("imp_o1", "原点电机1(rad)", "0.0")])
        ttk.Button(r1, text="发送弹簧原点", width=14,
                   command=partial(self._do_paired, "send_impedance_spring_origin", ("imp_o0", "imp_o1"), "弹簧原点")
                   ).grid(row=0, column=4, padx=8)

        r2 = ttk.Frame(f); r2.pack(fill="x")
        self._labeled_entries(r2, [("imp_k", "刚度", "0.0"), ("imp_b", "阻尼", "0.0"), ("imp_j", "惯量", "0.0")])
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
        ttk.Button(f, text="仅更新位置", width=12, command=self._do_traj_pos_only).pack(side="left", padx=2)

    def _build_pid_section(self, parent):
        f = ttk.LabelFrame(parent, text="PID 增益整定")
        f.pack(fill="x", padx=6, pady=5)
        rows = [
            ("位置环", [("pos_kp", "Kp", "0.0"), ("pos_ki", "Ki", "0.0")], "send_pos_pid", ("pos_kp", "pos_ki")),
            ("速度环", [("vel_kp", "Kp", "0.0"), ("vel_ki", "Ki", "0.0")], "send_vel_pid", ("vel_kp", "vel_ki")),
            ("电流环", [("cur_kp", "Kp", "0.0"), ("cur_ki", "Ki", "0.0")], "send_cur_pid", ("cur_kp", "cur_ki")),
        ]
        for k, (label, entry_specs, fn, keys) in enumerate(rows):
            ttk.Label(f, text=label, width=8).grid(row=k, column=0, padx=4, pady=3, sticky="w")
            sub = ttk.Frame(f)
            sub.grid(row=k, column=1, sticky="w")
            self._labeled_entries(sub, entry_specs)
            ttk.Button(f, text="发送", width=8,
                       command=partial(self._do_kv, fn, keys, label + "PID")
                       ).grid(row=k, column=2, padx=8)

    def _build_raw_section(self, parent):
        f = ttk.LabelFrame(parent, text="通用原始指令 (mode + 定点参数, 直接下发不做换算)")
        f.pack(fill="x", padx=6, pady=5)
        r = ttk.Frame(f); r.pack(side="left")
        self._labeled_entries(r, [("raw_mode", "mode", "6"), ("raw_p0", "电机0参数", "0"), ("raw_p1", "电机1参数", "0")])
        ttk.Button(f, text="发送原始指令", width=14, command=self._do_raw).pack(side="left", padx=8)

    # ============================================================ 业务
    def _toggle_serial(self):
        if self.serial_mgr and self.serial_mgr.is_connected:
            self.serial_mgr.disconnect()
            self._ui_connected = False
            self.conn_btn.config(text="连接串口")
            self.status_label.config(text="状态: 未连接", foreground="red")
            self._reset_tree_nodata()
        else:
            port = self.port_entry.get().strip()
            try:
                baud = int(self.baud_entry.get())
            except ValueError:
                messagebox.showerror("错误", "请输入有效的波特率数字")
                return
            self.serial_mgr = SerialManager(port, baud)
            if self.serial_mgr.connect():
                self._ui_connected = True
                self.conn_btn.config(text="断开串口")
                self.status_label.config(text=f"状态: 已连接 {port}", foreground="green")
                self._log(f"已连接 {port} @ {baud}")
            else:
                messagebox.showerror("错误", f"无法连接到端口 {port}")

    # ---- 目标解析 ----
    def _targets(self):
        """返回目标 MCU index 列表。"""
        sel = self.target_combo.current()
        if sel == SerialManager.NUM_MCU:  # 全部广播
            return list(range(SerialManager.NUM_MCU))
        return [sel]

    def _ensure_connected(self):
        if not (self.serial_mgr and self.serial_mgr.is_connected):
            messagebox.showwarning("警告", "串口未连接")
            return False
        return True

    def _getf(self, key):
        return float(self.entries[key].get())

    def _geti(self, key):
        return int(float(self.entries[key].get()))

    def _run(self, desc, func):
        """对所有目标执行 func(idx) 并写日志。func 返回 bool。"""
        if not self._ensure_connected():
            return
        for idx in self._targets():
            try:
                ok = func(idx)
            except Exception as e:  # 输入解析等异常
                self._log(f"[ERR] MCU{idx} {desc}: {e}")
                continue
            self._log(f"[{'OK ' if ok else 'ERR'}] MCU{idx} {desc}")

    # ---- 各类指令回调 ----
    def _do_state(self, state_key, label):
        self._run(f"状态→{label}", lambda idx: self.serial_mgr.send_state(idx, state_key))

    def _do_simple(self, fn, label):
        self._run(label, lambda idx: getattr(self.serial_mgr, fn)(idx))

    def _do_paired(self, fn, keys, label):
        try:
            v0, v1 = self._getf(keys[0]), self._getf(keys[1])
        except ValueError:
            messagebox.showwarning("输入错误", f"{label}: 请输入有效数字")
            return
        self._run(f"{label} [{v0}, {v1}]", lambda idx: getattr(self.serial_mgr, fn)(idx, v0, v1))

    def _do_kv(self, fn, keys, label):
        try:
            kp, ki = self._getf(keys[0]), self._getf(keys[1])
        except ValueError:
            messagebox.showwarning("输入错误", f"{label}: 请输入有效数字")
            return
        self._run(f"{label} [kp={kp}, ki={ki}]", lambda idx: getattr(self.serial_mgr, fn)(idx, kp, ki))

    def _do_impedance(self):
        try:
            k, b, j = self._getf("imp_k"), self._getf("imp_b"), self._getf("imp_j")
        except ValueError:
            messagebox.showwarning("输入错误", "阻抗参数: 请输入有效数字")
            return
        self._run(f"阻抗 [k={k}, b={b}, j={j}]",
                  lambda idx: self.serial_mgr.send_impedance_params(idx, k, b, j))

    def _do_traj(self):
        try:
            v, a, p0, p1 = self._getf("tj_v"), self._getf("tj_a"), self._getf("tj_p0"), self._getf("tj_p1")
        except ValueError:
            messagebox.showwarning("输入错误", "轨迹参数: 请输入有效数字")
            return
        self._run(f"轨迹 [v={v}, a={a}, p=({p0},{p1})]",
                  lambda idx: self.serial_mgr.send_trajectory(idx, v, a, p0, p1))

    def _do_traj_pos_only(self):
        try:
            p0, p1 = self._getf("tj_p0"), self._getf("tj_p1")
        except ValueError:
            messagebox.showwarning("输入错误", "轨迹位置: 请输入有效数字")
            return
        self._run(f"轨迹位置 [{p0}, {p1}]",
                  lambda idx: self.serial_mgr.send_trajectory_pos(idx, p0, p1))

    def _do_raw(self):
        try:
            mode, p0, p1 = self._geti("raw_mode"), self._geti("raw_p0"), self._geti("raw_p1")
        except ValueError:
            messagebox.showwarning("输入错误", "原始指令: 请输入有效整数")
            return
        self._run(f"RAW mode={mode} [{p0}, {p1}]",
                  lambda idx: self.serial_mgr.send_raw_command(idx, mode, p0, p1))

    # ---- 日志 ----
    def _log(self, msg):
        self.log_text.config(state="normal")
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)
        # 限制行数
        if int(self.log_text.index("end-1c").split(".")[0]) > 400:
            self.log_text.delete("1.0", "200.0")
        self.log_text.config(state="disabled")

    # ============================================================ 刷新
    @staticmethod
    def _fmt(v):
        return f"{v:.3f}" if v is not None else "—"

    def _reset_tree_nodata(self):
        """把所有 MCU/电机 行复位为无数据。"""
        for i in range(SerialManager.NUM_MCU):
            self.tree.item(f"mcu{i}", values=("", "", "", "", "", "无数据"), tags=("mcu", "nodata"))
            for m in range(SerialManager.MOTORS_PER_MCU):
                self.tree.item(f"mcu{i}_m{m}", values=("—", "—", "—", "—", "—", ""), tags=("nodata",))

    def _update_loop(self):
        mgr = self.serial_mgr
        if mgr and mgr.is_connected:
            self._ui_connected = True
            for i in range(SerialManager.NUM_MCU):
                stats = mgr.get_mcu_stats(i)
                frame = mgr.get_latest_frame(i)   # 每个 MCU 只取一次快照, 保证整行同帧一致

                if frame is None:
                    meta, ptag = "无数据", "nodata"
                else:
                    lr = stats["loss_rate"]
                    ptag = "err" if lr > 5.0 else ("warn" if lr > 0 else "ok")
                    meta = f"丢包 {lr:.1f}% (收:{stats['last_sec_received']} 丢:{stats['last_sec_lost']})"
                self.tree.item(f"mcu{i}", values=("", "", "", "", "", meta), tags=("mcu", ptag))

                for m in range(SerialManager.MOTORS_PER_MCU):
                    fields = mgr.motor_fields_from_frame(frame, m)
                    if fields is None or fields["state_code"] is None:
                        self.tree.item(f"mcu{i}_m{m}", values=("—", "—", "—", "—", "—", ""), tags=("nodata",))
                        continue

                    name = fields["state_name"]
                    state_str = f"{name} ({fields['state_code']})"
                    if name.endswith("Error") or name == "InnerOuterMismatch":
                        mtag = "err"
                    elif "Disable" in name or name == "StartupReady":
                        mtag = "warn"
                    else:
                        mtag = "ok"

                    self.tree.item(
                        f"mcu{i}_m{m}",
                        values=(state_str, self._fmt(fields["theta"]), self._fmt(fields["omega"]),
                                self._fmt(fields["acl"]), self._fmt(fields["torque"]), ""),
                        tags=(mtag,),
                    )
        elif self._ui_connected:
            # 后台读线程因串口异常(如拔出)自动断开时, 复位一次 UI 状态
            self._ui_connected = False
            self.conn_btn.config(text="连接串口")
            self.status_label.config(text="状态: 连接已断开", foreground="red")
            self._reset_tree_nodata()

        self.root.after(50, self._update_loop)


if __name__ == "__main__":
    root = tk.Tk()
    app = HighDofHandGUI(root)

    def on_closing():
        if app.serial_mgr:
            app.serial_mgr.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()
