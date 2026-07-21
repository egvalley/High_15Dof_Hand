"""
串口指令发送器 (对齐 router.h / router.c 真值版)。

数据流：高层命令 -> 按功能取 MotorFunc 的 mode 值与 scale 量化
        -> 分成"电机0命令 / 电机1命令"两组 -> RxCommandCodec 按前/后半段编码
        -> SerialManager.write_data。

电机路由：mode 与电机无关，电机由命令在帧里的前/后半段位置决定 (见 constants 顶部说明)。
本层只负责把命令分到 motor0 / motor1 两组，补齐与切分交给编码器。
"""

from sdk.protocol.constants import MotorFunc, CommFunc, MotorState
from sdk.protocol.rx_command_codec import RxCommandCodec
from sdk.models import MotorCommand, MotorTarget


class SerialCommander:

    def __init__(self, serial_manager, func=CommFunc.RX_FDCAN_COMMAND):
        """
        参数:
            serial_manager: SerialManager，编码后的帧由它的 write_data 发出。
            func:           命令帧 Func (CommFunc)，随物理链路选 FDCAN / USB-USART。
        用法: SerialCommander(manager, func=cfg.command_func)
        """
        self.sm = serial_manager
        self.codec = RxCommandCodec()
        self.func = func
        self._tx_counter = 0

    # ============================================================ 底层
    def _next_counter(self):
        """取当前发送帧计数器并自增 (0~255 回绕)，用于下位机检测丢帧。"""
        c = self._tx_counter
        self._tx_counter = (self._tx_counter + 1) & 0xFF
        return c

    def _cmd(self, func, value):
        """一条量化后的命令：mode 取功能码 (与电机无关)，value 按其 scale 量化为 round(value × scale)。"""
        quantized = int(round(value * MotorFunc.scale_of(func)))
        return MotorCommand(int(func), quantized)

    def _send(self, mcu, motor0_cmds, motor1_cmds):
        """
        把电机0 / 电机1 两组命令交编码器组帧并发送。
        两组都为空则直接判失败；编码抛 ValueError (如超命令上限) 时打印并返回 False。
        返回 bool：是否成功写入串口。所有对外 send_* 方法最终都汇到这里。
        """
        if not motor0_cmds and not motor1_cmds:
            print("[发送失败] 命令列表为空")
            return False
        try:
            frame = self.codec.encode(
                mcu, motor0_cmds, motor1_cmds,
                counter=self._next_counter(),
                func=self.func,
            )
        except ValueError as e:
            print(f"[发送失败] {e}")
            return False
        return self.sm.write_data(frame)

    # ---------------------------------------------------------------- 按目标电机分组
    def _targeted(self, mcu, cmds, motor):
        """
        共享的一组命令 cmds，按目标电机发给电机0 / 电机1 / 两者。
        motor 先归一化：MotorTarget 原样使用，0 / 1 / 2 兼容映射成对应枚举。
        """
        if isinstance(motor, MotorTarget):
            t = motor
        else:
            t = {0: MotorTarget.MOTOR_0, 1: MotorTarget.MOTOR_1,
                 2: MotorTarget.BOTH}[motor]
        m0 = list(cmds) if t.hits_motor0() else []
        m1 = list(cmds) if t.hits_motor1() else []
        return self._send(mcu, m0, m1)

    # ============================================================ 基本控制 (单值 + 目标电机)
    # 以下 send_* 把同一数值按 motor (MotorTarget/0/1/2) 发给 电机0 / 电机1 / 两者。
    # mcu 可为 index 0~7 或 ID 0xB0~0xB7；返回 bool (是否成功写入)。
    def send_position(self, mcu, pos, motor=MotorTarget.BOTH):
        """下发位置指令 θ (输出轴 rad) 到目标电机。"""
        return self._targeted(mcu, [self._cmd(MotorFunc.THETA_GEAR, pos)], motor)

    def send_velocity(self, mcu, vel, motor=MotorTarget.BOTH):
        """下发速度指令 ω (输出轴 rad/s) 到目标电机。"""
        return self._targeted(mcu, [self._cmd(MotorFunc.OMEGA_GEAR, vel)], motor)

    def send_torque(self, mcu, tau, motor=MotorTarget.BOTH):
        """下发力矩指令 τ (输出轴 mN·m) 到目标电机。"""
        return self._targeted(mcu, [self._cmd(MotorFunc.TORQUE_GEAR, tau)], motor)

    def send_iq(self, mcu, iq, motor=MotorTarget.BOTH):
        """下发 q 轴电流指令 iq (A) 到目标电机。"""
        return self._targeted(mcu, [self._cmd(MotorFunc.IQ, iq)], motor)

    def send_impedance_origin(self, mcu, origin, motor=MotorTarget.BOTH):
        """下发阻抗弹簧原点 (输出轴 rad) 到目标电机。"""
        return self._targeted(mcu, [self._cmd(MotorFunc.IMPEDANCE_SPRING_ORIGIN, origin)], motor)

    # ============================================================ 共享参数 + 目标电机
    # 以下方法把同一组参数按 motor (MotorTarget/0/1/2) 发给 电机0 / 电机1 / 两者。
    def send_impedance_params(self, mcu, spring, damper, inertia, motor=MotorTarget.BOTH):
        """下发阻抗三参数：刚度 spring / 阻尼 damper / 惯量 inertia。"""
        cmds = [self._cmd(MotorFunc.IMPEDANCE_SPRING, spring),
                self._cmd(MotorFunc.IMPEDANCE_DAMPER, damper),
                self._cmd(MotorFunc.IMPEDANCE_INERTIA, inertia)]
        return self._targeted(mcu, cmds, motor)

    def send_pos_pid(self, mcu, kp, ki, motor=MotorTarget.BOTH):
        """整定位置环 PID：kp/ki。"""
        cmds = [self._cmd(MotorFunc.POS_PID_KP, kp), self._cmd(MotorFunc.POS_PID_KI, ki)]
        return self._targeted(mcu, cmds, motor)

    def send_vel_pid(self, mcu, kp, ki, motor=MotorTarget.BOTH):
        """整定速度环 PID：kp/ki。"""
        cmds = [self._cmd(MotorFunc.VEL_PID_KP, kp), self._cmd(MotorFunc.VEL_PID_KI, ki)]
        return self._targeted(mcu, cmds, motor)

    def send_cur_pid(self, mcu, kp, ki, motor=MotorTarget.BOTH):
        """整定电流环 PID：kp/ki。"""
        cmds = [self._cmd(MotorFunc.CUR_PID_KP, kp), self._cmd(MotorFunc.CUR_PID_KI, ki)]
        return self._targeted(mcu, cmds, motor)

    def send_state(self, mcu, state, motor=MotorTarget.BOTH):
        """状态机派发 (DispatchMotorStateMachine)。state 可为状态名或状态码。"""
        if isinstance(state, str):
            if state not in MotorState.CODES:
                print(f"[发送失败] 未知状态名: {state}")
                return False
            state = MotorState.CODES[state]
        cmd = MotorCommand(int(MotorFunc.DISPATCH_STATE), int(state))
        return self._targeted(mcu, [cmd], motor)

    # ============================================================ 轨迹
    def send_trajectory(self, mcu, vel_max, acl_max, pos, motor=MotorTarget.BOTH):
        """一次性设定轨迹的 vmax/amax 与目标位置，下发到目标电机。"""
        cmds = [self._cmd(MotorFunc.TRAJ_VEL_MAX, vel_max),
                self._cmd(MotorFunc.TRAJ_ACL_MAX, acl_max),
                self._cmd(MotorFunc.TRAJ_POS_CMD, pos)]
        return self._targeted(mcu, cmds, motor)

    def send_trajectory_limits(self, mcu, vel_max, acl_max, motor=MotorTarget.BOTH):
        """只更新轨迹的 vmax/amax (不下发目标位置)，沿用上一帧的 pos_cmd。发到目标电机。"""
        cmds = [self._cmd(MotorFunc.TRAJ_VEL_MAX, vel_max),
                self._cmd(MotorFunc.TRAJ_ACL_MAX, acl_max)]
        return self._targeted(mcu, cmds, motor)

    def send_trajectory_pos(self, mcu, pos, motor=MotorTarget.BOTH):
        """只更新轨迹目标位置 (输出轴 rad)，沿用上一帧的 vmax/amax。发到目标电机。"""
        return self._targeted(mcu, [self._cmd(MotorFunc.TRAJ_POS_CMD, pos)], motor)

    # ============================================================ 设备级动作 (无参数)
    def _action(self, mcu, func, motor):
        """下发单条无参数功能码 func (param 恒为 0) 到目标电机 (回零/刷参/轨迹初始化等的公共实现)。"""
        return self._targeted(mcu, [MotorCommand(int(func), 0)], motor)

    def send_traj_init(self, mcu, motor=MotorTarget.BOTH):
        """轨迹模块初始化。"""
        return self._action(mcu, MotorFunc.TRAJ_INIT, motor)

    def send_traj_deinit(self, mcu, motor=MotorTarget.BOTH):
        """轨迹模块反初始化。"""
        return self._action(mcu, MotorFunc.TRAJ_DEINIT, motor)

    # ============================================================ 回零 (Homing)
    # 回零参数：前向力矩 (输出轴 mN·m) 与反向位置 (输出轴 rad)。
    # 三种用法：更新参数并回零 / 仅更新参数 / 仅触发回零，与轨迹三方式对称。
    def send_homing(self, mcu, motor=MotorTarget.BOTH):
        """仅触发回零 (不改参数)，沿用上一帧的前向力矩/反向位置。"""
        return self._action(mcu, MotorFunc.HOMING_INIT, motor)

    def send_homing_params(self, mcu, forward_torque, backward_pos, motor=MotorTarget.BOTH):
        """只更新回零参数 (前向力矩/反向位置)，不触发回零。发到目标电机。"""
        cmds = [self._cmd(MotorFunc.HOMING_FORWARD_TORQUE, forward_torque),
                self._cmd(MotorFunc.HOMING_BACKWARD_POSITION, backward_pos)]
        return self._targeted(mcu, cmds, motor)

    def send_homing_full(self, mcu, forward_torque, backward_pos, motor=MotorTarget.BOTH):
        """一帧内更新回零参数并触发回零 (先设参数后 init，顺序不可颠倒)。发到目标电机。"""
        cmds = [self._cmd(MotorFunc.HOMING_FORWARD_TORQUE, forward_torque),
                self._cmd(MotorFunc.HOMING_BACKWARD_POSITION, backward_pos),
                MotorCommand(int(MotorFunc.HOMING_INIT), 0)]
        return self._targeted(mcu, cmds, motor)

    def send_flashing_params(self, mcu, config_index=0, motor=MotorTarget.BOTH):
        """按配置序号 config_index 重置控制参数为预设并刷入 Flash (对应固件 ResetControlParams，config_index=0/1)。"""
        return self._targeted(mcu, [MotorCommand(int(MotorFunc.FLASHING_PARAMS), int(config_index))], motor)

    def send_clear_flash_error(self, mcu, motor=MotorTarget.BOTH):
        """清除 Flash 错误标志。"""
        return self._action(mcu, MotorFunc.CLEAR_FLASH_ERROR, motor)
