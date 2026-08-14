"""
串口指令发送器 (对齐【新版】router_rec_handle.h / router_rec_handle.c)。

数据流：高层命令 -> 按功能取 MotorFunc 的 scale 量化 -> 按目标电机取 Control_Mode
        (电机0 = 功能码，电机1 = 功能码+300) -> RxCommandCodec 平铺组帧 -> SerialManager.write_data。

"""

from sdk.protocol import MotorFunc, MotorState, RxFunc, RxCommandCodec
from sdk.models import MotorCommand, MotorTarget


class SerialCommander:

    def __init__(self, serial_manager, func=RxFunc.FDCAN_CMD):
        """
        参数:
            serial_manager: SerialManager，编码后的帧由它的 write_data 发出。
            func:           命令帧 Func (RxFunc)，默认 FDCAN_CMD。标的是【网关向下转发用的
                            总线】而不是 PC 这一段，走网关时只能是 FDCAN_CMD (见 RxFunc 文档)。
        用法: SerialCommander(manager, func=cfg.command_func)
        """
        self.sm = serial_manager
        self.codec = RxCommandCodec()
        self.func = func
        self._tx_counter = 0

    # ============================================================ 底层
    def _next_counter(self):
        """取当前发送帧计数器并自增 (uint32 回绕)，写进帧的时间戳字段，用于下位机检测丢帧。"""
        c = self._tx_counter
        self._tx_counter = (self._tx_counter + 1) & 0xFFFFFFFF
        return c

    @staticmethod
    def _motors_of(motor):
        """
        把目标电机归一化成电机序号列表 [0] / [1] / [0, 1]。
        motor 可为 MotorTarget，也兼容 0 / 1 / 2。
        """
        if not isinstance(motor, MotorTarget):
            motor = {0: MotorTarget.MOTOR_0, 1: MotorTarget.MOTOR_1,
                     2: MotorTarget.BOTH}[motor]
        return [m for m, hit in ((0, motor.hits_motor0()), (1, motor.hits_motor1())) if hit]

    # ---------------------------------------------------------------- 组帧发送
    def _targeted(self, mcu, specs, motor):
        """
        把 specs 按目标电机展开成一帧命令并发送，所有对外 send_* 最终都汇到这里。

        参数:
            mcu:   index 0~7 或 ID 0xB0~0xB7
            specs: [(MotorFunc, 物理量 或 None), ...]。None 表示无参数命令 (param 填 0)，
                   数值则按该功能的 scale 量化；顺序即固件执行顺序。
            motor: MotorTarget / 0 / 1 / 2
        返回: bool 是否成功写入串口。
        """
        motors = self._motors_of(motor)
        if not motors or not specs:
            print("[发送失败] 命令列表为空")
            return False

        commands = []
        for m in motors:
            for func, value in specs:
                param = 0 if value is None else round(value * MotorFunc.scale_of(func))
                commands.append(MotorCommand(MotorFunc.code_of(func, m), param))

        try:
            frame = self.codec.encode(
                mcu, commands,
                counter=self._next_counter(),
                func=self.func,
            )
        except ValueError as e:
            print(f"[发送失败] {e}")
            return False
        return self.sm.write_data(frame)

    # ============================================================ 基本控制 (单值 + 目标电机)
    # 以下 send_* 把同一数值按 motor (MotorTarget/0/1/2) 发给 电机0 / 电机1 / 两者。
    # mcu 可为 index 0~7 或 ID 0xB0~0xB7；返回 bool (是否成功写入)。
    def send_position(self, mcu, pos, motor=MotorTarget.BOTH):
        """下发位置指令 θ (输出轴 rad) 到目标电机。"""
        return self._targeted(mcu, [(MotorFunc.THETA_GEAR, pos)], motor)

    def send_velocity(self, mcu, vel, motor=MotorTarget.BOTH):
        """下发速度指令 ω (输出轴 rad/s) 到目标电机。"""
        return self._targeted(mcu, [(MotorFunc.OMEGA_GEAR, vel)], motor)

    def send_torque(self, mcu, tau, motor=MotorTarget.BOTH):
        """下发力矩指令 τ (输出轴 mN·m) 到目标电机。"""
        return self._targeted(mcu, [(MotorFunc.TORQUE_GEAR, tau)], motor)

    def send_iq(self, mcu, iq, motor=MotorTarget.BOTH):
        """下发 q 轴电流指令 iq (A) 到目标电机。"""
        return self._targeted(mcu, [(MotorFunc.IQ, iq)], motor)

    def send_id(self, mcu, i_d, motor=MotorTarget.BOTH):
        """下发 d 轴电流指令 id (A) 到目标电机。"""
        return self._targeted(mcu, [(MotorFunc.ID, i_d)], motor)

    def send_impedance_origin(self, mcu, origin, motor=MotorTarget.BOTH):
        """下发阻抗弹簧原点 (输出轴 rad) 到目标电机。"""
        return self._targeted(mcu, [(MotorFunc.IMPEDANCE_SPRING_ORIGIN, origin)], motor)

    # ============================================================ 共享参数 + 目标电机
    def send_impedance_params(self, mcu, spring, damper, inertia, motor=MotorTarget.BOTH):
        """下发阻抗三参数：刚度 spring / 阻尼 damper / 惯量 inertia。"""
        return self._targeted(mcu, [(MotorFunc.IMPEDANCE_SPRING, spring),
                                    (MotorFunc.IMPEDANCE_DAMPER, damper),
                                    (MotorFunc.IMPEDANCE_INERTIA, inertia)], motor)

    def send_pos_pid(self, mcu, kp, ki, motor=MotorTarget.BOTH):
        """整定位置环 PID：kp/ki。"""
        return self._targeted(mcu, [(MotorFunc.POS_PID_KP, kp),
                                    (MotorFunc.POS_PID_KI, ki)], motor)

    def send_vel_pid(self, mcu, kp, ki, motor=MotorTarget.BOTH):
        """整定速度环 PID：kp/ki。"""
        return self._targeted(mcu, [(MotorFunc.VEL_PID_KP, kp),
                                    (MotorFunc.VEL_PID_KI, ki)], motor)

    def send_cur_pid(self, mcu, kp, ki, motor=MotorTarget.BOTH):
        """整定电流环 PID：kp/ki。"""
        return self._targeted(mcu, [(MotorFunc.CUR_PID_KP, kp),
                                    (MotorFunc.CUR_PID_KI, ki)], motor)

    def send_state(self, mcu, state, motor=MotorTarget.BOTH):
        """状态机派发 (DispatchMotorStateMachine)。state 可为状态名或状态码。"""
        try:
            code = MotorState.code_of(state)
        except ValueError as e:
            print(f"[发送失败] {e}")
            return False
        # 状态码本身就是整数码，scale=1，直接当物理量传即可
        return self._targeted(mcu, [(MotorFunc.DISPATCH_STATE, code)], motor)

    # ============================================================ 轨迹
    def send_trajectory(self, mcu, vel_max, acl_max, pos, motor=MotorTarget.BOTH):
        """一次性设定轨迹的 vmax/amax 与目标位置，下发到目标电机。"""
        return self._targeted(mcu, [(MotorFunc.TRAJ_VEL_MAX, vel_max),
                                    (MotorFunc.TRAJ_ACL_MAX, acl_max),
                                    (MotorFunc.TRAJ_POS_CMD, pos)], motor)

    def send_trajectory_limits(self, mcu, vel_max, acl_max, motor=MotorTarget.BOTH):
        """只更新轨迹的 vmax/amax (不下发目标位置)，沿用上一帧的 pos_cmd。发到目标电机。"""
        return self._targeted(mcu, [(MotorFunc.TRAJ_VEL_MAX, vel_max),
                                    (MotorFunc.TRAJ_ACL_MAX, acl_max)], motor)

    def send_trajectory_acl_max(self, mcu, acl_max, motor=MotorTarget.BOTH):
        """只更新轨迹加速度上限 amax，沿用上一帧的 vmax 与目标位置。发到目标电机。"""
        return self._targeted(mcu, [(MotorFunc.TRAJ_ACL_MAX, acl_max)], motor)

    def send_trajectory_pos(self, mcu, pos, motor=MotorTarget.BOTH):
        """只更新轨迹目标位置 (输出轴 rad)，沿用上一帧的 vmax/amax。发到目标电机。"""
        return self._targeted(mcu, [(MotorFunc.TRAJ_POS_CMD, pos)], motor)

    # ============================================================ 设备级动作 (无参数)
    def _action(self, mcu, func, motor):
        """下发单条无参数功能码 func (param 恒为 0) 到目标电机 (回零/刷参/轨迹初始化等的公共实现)。"""
        return self._targeted(mcu, [(func, None)], motor)

    def send_traj_init(self, mcu, motor=MotorTarget.BOTH):
        """轨迹模块初始化。"""
        return self._action(mcu, MotorFunc.TRAJ_INIT, motor)

    def send_traj_deinit(self, mcu, motor=MotorTarget.BOTH):
        """轨迹模块反初始化。"""
        return self._action(mcu, MotorFunc.TRAJ_DEINIT, motor)

    # ============================================================ 回零 (Homing)
    # 新固件把回零参数拆成 4 条：前向力矩 / 反向力矩 (mN·m) + 前向位置 / 反向位置 (rad)。
    # 三种用法：更新参数并回零 / 仅更新参数 / 仅触发回零，与轨迹三方式对称。
    @staticmethod
    def _homing_specs(forward_torque, backward_torque, forward_pos, backward_pos):
        """回零四参数的 specs (顺序：力矩 -> 位置)，供"仅更新参数"与"参数+执行"复用。"""
        return [(MotorFunc.HOMING_FORWARD_TORQUE, forward_torque),
                (MotorFunc.HOMING_BACKWARD_TORQUE, backward_torque),
                (MotorFunc.HOMING_FORWARD_POSITION, forward_pos),
                (MotorFunc.HOMING_BACKWARD_POSITION, backward_pos)]

    def send_homing(self, mcu, motor=MotorTarget.BOTH):
        """仅触发回零 (不改参数)，沿用上一帧的力矩/位置参数。"""
        return self._action(mcu, MotorFunc.HOMING_INIT, motor)

    def send_homing_params(self, mcu, forward_torque, backward_torque,
                           forward_pos, backward_pos, motor=MotorTarget.BOTH):
        """只更新回零四参数 (前/反向力矩、前/反向位置)，不触发回零。发到目标电机。"""
        return self._targeted(
            mcu, self._homing_specs(forward_torque, backward_torque,
                                    forward_pos, backward_pos), motor)

    def send_homing_full(self, mcu, forward_torque, backward_torque,
                         forward_pos, backward_pos, motor=MotorTarget.BOTH):
        """一帧内更新回零四参数并触发回零 (先设参数后 init，顺序不可颠倒)。发到目标电机。"""
        specs = self._homing_specs(forward_torque, backward_torque,
                                   forward_pos, backward_pos)
        specs.append((MotorFunc.HOMING_INIT, None))
        return self._targeted(mcu, specs, motor)

    # ============================================================ 设备级
    def send_flashing_params(self, mcu, config_index=0, motor=MotorTarget.BOTH):
        """按配置序号 config_index 重置控制参数为预设并刷入 Flash (对应固件 ResetControlParams)。"""
        return self._targeted(mcu, [(MotorFunc.FLASHING_PARAMS, int(config_index))], motor)

    # ============================================================ 错误清除 (无参数)
    # 反馈里的错误字位置位后一直保持，只能靠下面这几条显式清掉，一条清一段位域。
    # 清掉的是"错误标志"，是否据此从错误态恢复由固件自己决定。
    def send_clear_inner_error(self, mcu, motor=MotorTarget.BOTH):
        """清除内环错误 (错误字 bit0~bit7)。发到目标电机。"""
        return self._action(mcu, MotorFunc.CLEAR_INNER_ERROR, motor)

    def send_clear_outer_error(self, mcu, motor=MotorTarget.BOTH):
        """清除外环错误 (错误字 bit8~bit15)。发到目标电机。"""
        return self._action(mcu, MotorFunc.CLEAR_OUTER_ERROR, motor)

    def send_clear_encoder_error(self, mcu, motor=MotorTarget.BOTH):
        """清除编码器错误 (错误字 bit16~bit17)。M1/M2 各挂一路独立 SPI，各清各的。"""
        return self._action(mcu, MotorFunc.CLEAR_ENCODER_ERROR, motor)

    def send_clear_flash_error(self, mcu, motor=MotorTarget.BOTH):
        """清除 Flash 错误标志。Flash 是共享设备，固件两个 mode 都调同一个清除服务。"""
        return self._action(mcu, MotorFunc.CLEAR_FLASH_ERROR, motor)
