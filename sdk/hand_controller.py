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
        self.sm = serial_manager
        self.commander = commander

    # ---------------------------------------------------------------- 通用执行
    def _run(self, mcu_indices, call):
        """
        对每个 MCU 执行 call(commander, idx) -> bool，收集结果。
        call 内抛异常视为失败并记录消息。
        """
        results = []
        for idx in mcu_indices:
            try:
                ok = bool(call(self.commander, idx))
                results.append(CommandResult(idx, ok))
            except Exception as e:
                results.append(CommandResult(idx, False, str(e)))
        return results

    # ---------------------------------------------------------------- 类型 A
    def send_position(self, mcu_indices, pos0, pos1):
        return self._run(mcu_indices, lambda c, i: c.send_position(i, pos0, pos1))

    def send_velocity(self, mcu_indices, vel0, vel1):
        return self._run(mcu_indices, lambda c, i: c.send_velocity(i, vel0, vel1))

    def send_torque(self, mcu_indices, tau0, tau1):
        return self._run(mcu_indices, lambda c, i: c.send_torque(i, tau0, tau1))

    def send_iq(self, mcu_indices, iq0, iq1):
        return self._run(mcu_indices, lambda c, i: c.send_iq(i, iq0, iq1))

    def send_impedance_origin(self, mcu_indices, o0, o1):
        return self._run(mcu_indices, lambda c, i: c.send_impedance_origin(i, o0, o1))

    def send_trajectory_pos(self, mcu_indices, pos0, pos1):
        return self._run(mcu_indices, lambda c, i: c.send_trajectory_pos(i, pos0, pos1))

    # ---------------------------------------------------------------- 类型 B
    def send_state(self, mcu_indices, state, motor=MotorTarget.BOTH):
        return self._run(mcu_indices, lambda c, i: c.send_state(i, state, motor))

    def send_impedance_params(self, mcu_indices, spring, damper, inertia, motor=MotorTarget.BOTH):
        return self._run(mcu_indices,
                         lambda c, i: c.send_impedance_params(i, spring, damper, inertia, motor))

    def send_pos_pid(self, mcu_indices, kp, ki, motor=MotorTarget.BOTH):
        return self._run(mcu_indices, lambda c, i: c.send_pos_pid(i, kp, ki, motor))

    def send_vel_pid(self, mcu_indices, kp, ki, motor=MotorTarget.BOTH):
        return self._run(mcu_indices, lambda c, i: c.send_vel_pid(i, kp, ki, motor))

    def send_cur_pid(self, mcu_indices, kp, ki, motor=MotorTarget.BOTH):
        return self._run(mcu_indices, lambda c, i: c.send_cur_pid(i, kp, ki, motor))

    def send_action(self, mcu_indices, action_name, motor=MotorTarget.BOTH):
        """action_name 为 commander 上的方法名，如 'send_homing'。"""
        return self._run(mcu_indices,
                         lambda c, i: getattr(c, action_name)(i, motor))

    # ---------------------------------------------------------------- 类型 C
    def send_trajectory(self, mcu_indices, vel_max, acl_max, pos0, pos1):
        return self._run(mcu_indices,
                         lambda c, i: c.send_trajectory(i, vel_max, acl_max, pos0, pos1))

    # ---------------------------------------------------------------- RAW
    def send_raw(self, mcu_indices, mode, p0, p1):
        return self._run(mcu_indices, lambda c, i: c.send_raw_command(i, mode, p0, p1))
