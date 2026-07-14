"""
串口指令发送器 (对齐 router.h / router.c 真值版)。

数据流：高层命令 -> 按 (功能, 电机) 查 mode 号 -> 按 FUNC_SCALE 量化
        -> 组扁平命令列表 -> RxCommandCodec 编码 -> SerialManager.write_data。

固件支持的功能 (会真正生效)：
    位置 / 速度 / 力矩(电流) / 阻抗(弹簧·阻尼·惯量) / 换ID / 状态派发。
固件未实现的功能 (调用即抛 UnsupportedCommand，避免误发错误 mode)：
    Iq/Id、弹簧原点、各环 PID、轨迹、回零、刷参、系统辨识、清Flash。
"""

from sdk.protocol.constants import (
    MotorFunc, CommFunc, mode_for, scale_of, MOTOR_STATE,
)
from sdk.protocol.rx_command_codec import RxCommandCodec
from sdk.models import MotorCommand, MotorTarget


class UnsupportedCommand(Exception):
    """该固件 (router.c) 未实现的指令。"""


class SerialCommander:

    def __init__(self, serial_manager, func=CommFunc.RX_FDCAN_COMMAND, pad_canfd=None):
        self.sm = serial_manager
        self.codec = RxCommandCodec()
        self.func = func
        self.pad_canfd = pad_canfd
        self._tx_counter = 0

    # ============================================================ 底层
    def _next_counter(self):
        c = self._tx_counter
        self._tx_counter = (self._tx_counter + 1) & 0xFF
        return c

    def _quant(self, func, value):
        return int(round(value * scale_of(func)))

    def _send(self, mcu, commands):
        if not commands:
            print("[发送失败] 命令列表为空")
            return False
        try:
            frame = self.codec.encode(
                mcu, commands,
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
        if isinstance(motor, MotorTarget):
            return motor
        return {"both": MotorTarget.BOTH, 0: MotorTarget.MOTOR_0,
                1: MotorTarget.MOTOR_1}[motor]

    @classmethod
    def _motors_of(cls, motor):
        """把目标解析成电机下标列表 [0] / [1] / [0,1]。"""
        t = cls._as_target(motor)
        out = []
        if t.hits_motor0():
            out.append(0)
        if t.hits_motor1():
            out.append(1)
        return out

    def _value_command(self, func, value, motor):
        """按 (功能, 目标电机) 生成量化后的命令列表。"""
        return [
            MotorCommand(mode_for(func, m), self._quant(func, value))
            for m in self._motors_of(motor)
        ]

    # ============================================================ 逐电机取值 (类型 A)
    # pos0/pos1 分别对应电机0/电机1；None 表示该侧不发。
    def _paired(self, func, v0, v1):
        cmds = []
        if v0 is not None:
            cmds.append(MotorCommand(mode_for(func, 0), self._quant(func, v0)))
        if v1 is not None:
            cmds.append(MotorCommand(mode_for(func, 1), self._quant(func, v1)))
        return cmds

    def send_position(self, mcu, pos0=None, pos1=None):
        return self._send(mcu, self._paired(MotorFunc.POSITION, pos0, pos1))

    def send_velocity(self, mcu, vel0=None, vel1=None):
        return self._send(mcu, self._paired(MotorFunc.VELOCITY, vel0, vel1))

    def send_torque(self, mcu, tau0=None, tau1=None):
        # 注意：固件 Torque 通道实际下发到电流环 (ServiceCurrentCmd)，缩放 ×1。
        return self._send(mcu, self._paired(MotorFunc.TORQUE, tau0, tau1))

    # ============================================================ 共享参数 + 目标电机 (类型 B)
    def send_impedance_params(self, mcu, spring, damper, inertia, motor=MotorTarget.BOTH):
        cmds = []
        for m in self._motors_of(motor):
            cmds.append(MotorCommand(mode_for(MotorFunc.IMPEDANCE_SPRING, m),
                                     self._quant(MotorFunc.IMPEDANCE_SPRING, spring)))
            cmds.append(MotorCommand(mode_for(MotorFunc.IMPEDANCE_DAMPER, m),
                                     self._quant(MotorFunc.IMPEDANCE_DAMPER, damper)))
            cmds.append(MotorCommand(mode_for(MotorFunc.IMPEDANCE_INERTIA, m),
                                     self._quant(MotorFunc.IMPEDANCE_INERTIA, inertia)))
        return self._send(mcu, cmds)

    def send_state(self, mcu, state, motor=MotorTarget.BOTH):
        """状态机派发 (Mode_Select)。state 可为状态名或状态码。"""
        if isinstance(state, str):
            if state not in MOTOR_STATE:
                print(f"[发送失败] 未知状态名: {state}")
                return False
            state = MOTOR_STATE[state]
        cmds = [MotorCommand(mode_for(MotorFunc.MODE_SELECT, m), int(state))
                for m in self._motors_of(motor)]
        return self._send(mcu, cmds)

    def send_switch_id(self, mcu, new_id, motor=MotorTarget.BOTH):
        """修改电机 ID (Switch_ID)。参数为目标 ID，直接以 uint16 下发。"""
        cmds = [MotorCommand(mode_for(MotorFunc.SWITCH_ID, m), int(new_id))
                for m in self._motors_of(motor)]
        return self._send(mcu, cmds)

    # ============================================================ RAW 透传
    def send_raw_command(self, mcu, mode, param0=None, param1=None):
        """
        直接下发指定 mode 与定点参数，不做换算。

        注意：现固件按 mode 自带的 M0/M1 区分电机，因此若要分别控制两个电机，
        请分别填入两条【不同 mode】的原始指令；这里的 param0/param1 只是把
        (mode, param0)、(mode, param1) 各发一条 (同一 mode 发两次通常无意义)。
        """
        cmds = []
        if param0 is not None:
            cmds.append(MotorCommand(int(mode), int(param0)))
        if param1 is not None:
            cmds.append(MotorCommand(int(mode), int(param1)))
        return self._send(mcu, cmds)

    # ============================================================ 固件未实现 (显式拒绝)
    def _unsupported(self, name):
        raise UnsupportedCommand(f"{name}：当前固件 (router.c) 未实现该指令")

    def send_iq(self, mcu, iq0=None, iq1=None):
        self._unsupported("电流 Iq 直控")

    def send_id(self, mcu, id0=None, id1=None):
        self._unsupported("电流 Id 直控")

    def send_impedance_origin(self, mcu, origin0=None, origin1=None):
        self._unsupported("阻抗弹簧原点")

    def send_pos_pid(self, mcu, kp, ki, motor=MotorTarget.BOTH):
        self._unsupported("位置环 PID 整定")

    def send_vel_pid(self, mcu, kp, ki, motor=MotorTarget.BOTH):
        self._unsupported("速度环 PID 整定")

    def send_cur_pid(self, mcu, kp, ki, motor=MotorTarget.BOTH):
        self._unsupported("电流环 PID 整定")

    def send_trajectory(self, mcu, vel_max, acl_max, pos0=None, pos1=None):
        self._unsupported("轨迹规划")

    def send_trajectory_pos(self, mcu, pos0=None, pos1=None):
        self._unsupported("轨迹目标位置")

    def send_homing(self, mcu, motor=MotorTarget.BOTH):
        self._unsupported("回零 Homing")

    def send_flashing_params(self, mcu, motor=MotorTarget.BOTH):
        self._unsupported("刷写参数")

    def send_traj_init(self, mcu, motor=MotorTarget.BOTH):
        self._unsupported("轨迹初始化")

    def send_traj_deinit(self, mcu, motor=MotorTarget.BOTH):
        self._unsupported("轨迹反初始化")

    def send_sysiden(self, mcu, motor=MotorTarget.BOTH):
        self._unsupported("系统辨识")

    def send_clear_flash_error(self, mcu, motor=MotorTarget.BOTH):
        self._unsupported("清除 Flash 错误")


# 已由本固件实现、GUI 可放心使用的功能 (供界面按需灰化未实现项)
FIRMWARE_SUPPORTED = frozenset({
    "send_position", "send_velocity", "send_torque",
    "send_impedance_params", "send_state", "send_switch_id",
    "send_raw_command",
})
