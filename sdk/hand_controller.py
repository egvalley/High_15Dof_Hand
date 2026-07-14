"""
业务协调层。

把原来堆在 GUI 里的 _run / _do_xxx (选哪些 MCU、循环发送、收集结果)
从界面代码里剥离出来。

约定：所有方法接收 mcu_indices (可迭代的 index 列表) 和已解析好的数值，
返回 list[CommandResult]，由 GUI 负责展示 / 记日志。
"""

from sdk.models import CommandResult, MotorTarget


class HandController:

    def __init__(self, serial_manager, commander):
        """
        参数:
            serial_manager: SerialManager 实例 (保存在 self.sm，本层暂不直接用，
                            预留给未来"发送后读回反馈"的协调逻辑)。
            commander:      SerialCommander 实例，所有下发最终经由它完成。
        用法: controller = HandController(manager, commander)
        """
        self.sm = serial_manager
        self.commander = commander

    # ---------------------------------------------------------------- 通用执行
    def _run(self, mcu_indices, call):
        """
        对每个 MCU 执行 call(commander, idx) -> bool，收集结果。
        call 内抛异常视为失败并记录消息。

        参数:
            mcu_indices: 可迭代的 MCU index 列表 (如 [0, 3] 或 range(8))。
            call:        (commander, idx) -> bool 的回调，通常是一个 lambda，
                         把某个 commander.send_xxx 绑定到当前 idx。
        返回: list[CommandResult]，逐 MCU 记录 ok/异常消息。
        """
        results = []
        for idx in mcu_indices:
            try:
                ok = bool(call(self.commander, idx))
                results.append(CommandResult(idx, ok))
            except Exception as e:
                results.append(CommandResult(idx, False, str(e)))
        return results

    # ------------------------------------------ 类型 A：逐电机两个数值 (v0/v1，None=该电机不发)
    def send_position(self, mcu_indices, pos0, pos1):
        """向选中 MCU 下发位置指令 (输出轴 rad)。pos0->电机0, pos1->电机1。返回 list[CommandResult]。"""
        return self._run(mcu_indices, lambda c, i: c.send_position(i, pos0, pos1))

    def send_velocity(self, mcu_indices, vel0, vel1):
        """向选中 MCU 下发速度指令 (输出轴 rad/s)。vel0->电机0, vel1->电机1。"""
        return self._run(mcu_indices, lambda c, i: c.send_velocity(i, vel0, vel1))

    def send_torque(self, mcu_indices, tau0, tau1):
        """向选中 MCU 下发力矩指令 (输出轴 N·m)。tau0->电机0, tau1->电机1。"""
        return self._run(mcu_indices, lambda c, i: c.send_torque(i, tau0, tau1))

    def send_iq(self, mcu_indices, iq0, iq1):
        """向选中 MCU 下发 q 轴电流指令 (A)。iq0->电机0, iq1->电机1。"""
        return self._run(mcu_indices, lambda c, i: c.send_iq(i, iq0, iq1))

    def send_impedance_origin(self, mcu_indices, o0, o1):
        """向选中 MCU 下发阻抗弹簧原点 (输出轴 rad)。o0->电机0, o1->电机1。"""
        return self._run(mcu_indices, lambda c, i: c.send_impedance_origin(i, o0, o1))

    def send_trajectory_pos(self, mcu_indices, pos0, pos1):
        """只更新轨迹目标位置 (输出轴 rad)，不重设 vmax/amax。pos0->电机0, pos1->电机1。"""
        return self._run(mcu_indices, lambda c, i: c.send_trajectory_pos(i, pos0, pos1))

    # ------------------------------------- 类型 B：共享参数 + 目标电机 (motor 选 M0/M1/两者)
    def send_state(self, mcu_indices, state, motor=MotorTarget.BOTH):
        """派发电机状态机。state 可为状态名 (如 'AppPositionCtrl') 或状态码 int。"""
        return self._run(mcu_indices, lambda c, i: c.send_state(i, state, motor))

    def send_impedance_params(self, mcu_indices, spring, damper, inertia, motor=MotorTarget.BOTH):
        """下发阻抗三参数 刚度/阻尼/惯量 到目标电机。"""
        return self._run(mcu_indices,
                         lambda c, i: c.send_impedance_params(i, spring, damper, inertia, motor))

    def send_pos_pid(self, mcu_indices, kp, ki, motor=MotorTarget.BOTH):
        """整定位置环 PID 的 kp/ki 到目标电机。"""
        return self._run(mcu_indices, lambda c, i: c.send_pos_pid(i, kp, ki, motor))

    def send_vel_pid(self, mcu_indices, kp, ki, motor=MotorTarget.BOTH):
        """整定速度环 PID 的 kp/ki 到目标电机。"""
        return self._run(mcu_indices, lambda c, i: c.send_vel_pid(i, kp, ki, motor))

    def send_cur_pid(self, mcu_indices, kp, ki, motor=MotorTarget.BOTH):
        """整定电流环 PID 的 kp/ki 到目标电机。"""
        return self._run(mcu_indices, lambda c, i: c.send_cur_pid(i, kp, ki, motor))

    def send_action(self, mcu_indices, action_name, motor=MotorTarget.BOTH):
        """
        下发无参数设备级动作。action_name 为 commander 上的方法名，
        如 'send_homing' / 'send_flashing_params' / 'send_traj_init' / 'send_traj_deinit'
        / 'send_sysiden' / 'send_clear_flash_error'。
        用 getattr 分派，便于 GUI 用 (按钮文案, 方法名) 表驱动。
        """
        return self._run(mcu_indices,
                         lambda c, i: getattr(c, action_name)(i, motor))

    # ------------------------------------------------- 类型 C：一帧内设定 vmax/amax + 逐电机位置
    def send_trajectory(self, mcu_indices, vel_max, acl_max, pos0, pos1):
        """下发完整轨迹指令：速度上限/加速度上限 + 逐电机目标位置 (pos0->电机0, pos1->电机1)。"""
        return self._run(mcu_indices,
                         lambda c, i: c.send_trajectory(i, vel_max, acl_max, pos0, pos1))

    # ---------------------------------------------------------------- RAW 透传
    def send_raw(self, mcu_indices, mode, p0, p1):
        """
        直接下发原始 (mode, 定点参数)，不做物理量换算。
        p0->电机0 (前半段), p1->电机1 (后半段)；None 表示该电机不发。
        用于调试固件里尚未包装成高层方法的功能码。
        """
        return self._run(mcu_indices, lambda c, i: c.send_raw_command(i, mode, p0, p1))
