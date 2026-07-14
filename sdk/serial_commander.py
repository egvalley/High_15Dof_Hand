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

    def __init__(self, serial_manager, func=CommFunc.RX_FDCAN_COMMAND, pad_canfd=None):
        """
        参数:
            serial_manager: SerialManager，编码后的帧由它的 write_data 发出。
            func:           命令帧 Func (CommFunc)，随物理链路选 FDCAN / USB-USART。
            pad_canfd:      是否补齐到 CAN-FD 合法长度；None 时由编码器按 func 自动判定。
        用法: SerialCommander(manager, func=cfg.command_func, pad_canfd=cfg.resolved_pad_canfd())
        """
        self.sm = serial_manager
        self.codec = RxCommandCodec()
        self.func = func
        self.pad_canfd = pad_canfd
        self._tx_counter = 0

    # ============================================================ 底层
    def _next_counter(self):
        """取当前发送帧计数器并自增 (0~255 回绕)，用于下位机检测丢帧。"""
        c = self._tx_counter
        self._tx_counter = (self._tx_counter + 1) & 0xFF
        return c

    def _quant(self, func, value):
        """按功能码的 scale 把浮点物理量量化成下发整数：round(value × scale)。"""
        return int(round(value * MotorFunc.scale_of(func)))

    def _cmd(self, func, value):
        """一条量化后的命令 (mode 取功能码，与电机无关)。"""
        return MotorCommand(int(func), self._quant(func, value))

    def _bare(self, func, param=0):
        """无参数功能 (回零/刷参/轨迹初始化…) 的命令，param 直接透传。"""
        return MotorCommand(int(func), int(param))

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
                pad_canfd=self.pad_canfd,
            )
        except ValueError as e:
            print(f"[发送失败] {e}")
            return False
        return self.sm.write_data(frame)

    @staticmethod
    def _as_target(motor):
        """把 MotorTarget / 'both' / 0 / 1 统一归一化成 MotorTarget 枚举 (兼容多种调用写法)。"""
        if isinstance(motor, MotorTarget):
            return motor
        return {"both": MotorTarget.BOTH, 0: MotorTarget.MOTOR_0,
                1: MotorTarget.MOTOR_1}[motor]

    # ---------------------------------------------------------------- 两种分组方式
    def _paired(self, mcu, func, v0, v1):
        """逐电机取值：v0->电机0, v1->电机1；None 表示该电机不发 (由编码器补 NOP)。"""
        m0 = [self._cmd(func, v0)] if v0 is not None else []
        m1 = [self._cmd(func, v1)] if v1 is not None else []
        return self._send(mcu, m0, m1)

    def _targeted(self, mcu, cmds, motor):
        """共享的一组命令 cmds，按目标电机发给电机0 / 电机1 / 两者。"""
        t = self._as_target(motor)
        m0 = list(cmds) if t.hits_motor0() else []
        m1 = list(cmds) if t.hits_motor1() else []
        return self._send(mcu, m0, m1)

    # ============================================================ 基本控制 (逐电机取值)
    # 以下 send_* 均为逐电机取值：v0->电机0, v1->电机1，None 表示该电机不发。
    # mcu 可为 index 0~7 或 ID 0xB0~0xB7；返回 bool (是否成功写入)。
    def send_position(self, mcu, pos0=None, pos1=None):
        """下发位置指令 θ (输出轴 rad)。"""
        return self._paired(mcu, MotorFunc.THETA_GEAR, pos0, pos1)

    def send_velocity(self, mcu, vel0=None, vel1=None):
        """下发速度指令 ω (输出轴 rad/s)。"""
        return self._paired(mcu, MotorFunc.OMEGA_GEAR, vel0, vel1)

    def send_torque(self, mcu, tau0=None, tau1=None):
        """下发力矩指令 τ (输出轴 N·m)。"""
        return self._paired(mcu, MotorFunc.TORQUE_GEAR, tau0, tau1)

    def send_iq(self, mcu, iq0=None, iq1=None):
        """下发 q 轴电流指令 iq (A)。"""
        return self._paired(mcu, MotorFunc.IQ, iq0, iq1)

    def send_impedance_origin(self, mcu, origin0=None, origin1=None):
        """下发阻抗弹簧原点 (输出轴 rad)。"""
        return self._paired(mcu, MotorFunc.IMPEDANCE_SPRING_ORIGIN, origin0, origin1)

    # ============================================================ 共享参数 + 目标电机
    # 以下方法把同一组参数按 motor (MotorTarget/'both'/0/1) 发给 电机0 / 电机1 / 两者。
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
    def send_trajectory(self, mcu, vel_max, acl_max, pos0=None, pos1=None):
        """一次性设定轨迹的 vmax/amax 与逐电机目标位置。"""
        def group(pos):
            return [self._cmd(MotorFunc.TRAJ_VEL_MAX, vel_max),
                    self._cmd(MotorFunc.TRAJ_ACL_MAX, acl_max),
                    self._cmd(MotorFunc.TRAJ_POS_CMD, pos)]
        m0 = group(pos0) if pos0 is not None else []
        m1 = group(pos1) if pos1 is not None else []
        return self._send(mcu, m0, m1)

    def send_trajectory_pos(self, mcu, pos0=None, pos1=None):
        """只更新轨迹目标位置 (输出轴 rad)，沿用上一帧的 vmax/amax。逐电机取值。"""
        return self._paired(mcu, MotorFunc.TRAJ_POS_CMD, pos0, pos1)

    # ============================================================ 设备级动作 (无参数)
    def _action(self, mcu, func, motor):
        """下发单条无参数功能码 func 到目标电机 (回零/刷参/轨迹初始化等的公共实现)。"""
        return self._targeted(mcu, [self._bare(func)], motor)

    def send_traj_init(self, mcu, motor=MotorTarget.BOTH):
        """轨迹模块初始化。"""
        return self._action(mcu, MotorFunc.TRAJ_INIT, motor)

    def send_traj_deinit(self, mcu, motor=MotorTarget.BOTH):
        """轨迹模块反初始化。"""
        return self._action(mcu, MotorFunc.TRAJ_DEINIT, motor)

    def send_homing(self, mcu, motor=MotorTarget.BOTH):
        """触发回零。"""
        return self._action(mcu, MotorFunc.HOMING, motor)

    def send_flashing_params(self, mcu, motor=MotorTarget.BOTH):
        """把当前参数刷写进 Flash。"""
        return self._action(mcu, MotorFunc.FLASHING_PARAMS, motor)

    def send_sysiden(self, mcu, motor=MotorTarget.BOTH):
        """触发系统辨识。"""
        return self._action(mcu, MotorFunc.SYS_IDEN, motor)

    def send_clear_flash_error(self, mcu, motor=MotorTarget.BOTH):
        """清除 Flash 错误标志。"""
        return self._action(mcu, MotorFunc.CLEAR_FLASH_ERROR, motor)

    # ============================================================ RAW 透传
    def send_raw_command(self, mcu, mode, param0=None, param1=None):
        """
        直接下发指定 mode 与定点参数，不做换算。
        param0 -> 电机0 (前半段)，param1 -> 电机1 (后半段)；None 表示该电机不发。
        """
        m0 = [MotorCommand(int(mode), int(param0))] if param0 is not None else []
        m1 = [MotorCommand(int(mode), int(param1))] if param1 is not None else []
        return self._send(mcu, m0, m1)
