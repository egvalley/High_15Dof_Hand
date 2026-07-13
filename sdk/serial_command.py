import struct
from sdk.serial_manager import SerialManager


class SerialCommander:
    """
    高自由度机械手 上位机串口指令发送器
    """
    # ==================== Rx (上位机 -> 下位机) 协议常量 ====================
    RX_FRAME_HEADER = 0x3B
    RX_FRAME_TAIL   = 0x1E
    RX_FUNC_USB_USART_CMD = 0x03
    RX_FUNC_FDCAN_CMD     = 0x04
    RX_FUNC_CMD           = RX_FUNC_FDCAN_CMD
    CANFD_VALID_SIZES     = (8, 12, 16, 20, 24, 32, 48, 64)

    # ==================== 控制模式枚举 ====================
    MODE_FOC_UQ                      = 2
    MODE_FOC_UD                      = 3
    MODE_FOC_UALPHA                  = 4
    MODE_FOC_UBETA                   = 5
    MODE_FOC_THETA_GEAR              = 6
    MODE_FOC_OMEGA_GEAR              = 7
    MODE_FOC_TORQUE_GEAR             = 8
    MODE_FOC_IQ                      = 9
    MODE_FOC_ID                      = 10
    MODE_FOC_IMPEDANCE_SPRING        = 32
    MODE_FOC_IMPEDANCE_DAMPER        = 33
    MODE_FOC_IMPEDANCE_INERTIA       = 34
    MODE_FOC_IMPEDANCE_SPRING_ORIGIN = 35
    MODE_FOC_CUR_PID_KP              = 52
    MODE_FOC_CUR_PID_KI              = 53
    MODE_FOC_VEL_PID_KP              = 54
    MODE_FOC_VEL_PID_KI              = 55
    MODE_FOC_POS_PID_KP              = 56
    MODE_FOC_POS_PID_KI              = 57
    MODE_FOC_DISPATCH_STATE          = 72
    MODE_APP_TRAJ_INIT               = 82
    MODE_APP_TRAJ_DEINIT             = 83
    MODE_APP_TRAJ_VEL_MAX            = 84
    MODE_APP_TRAJ_ACL_MAX            = 85
    MODE_APP_TRAJ_POS_CMD            = 86
    MODE_APP_HOMING                  = 102
    MODE_APP_FLASHING_PARAMS         = 103
    MODE_APP_SYSIDEN                 = 104
    MODE_DEVICE_CLEAR_FLASH_ERROR    = 122
    MODE_NOP                         = 0

    MODE_NAME = {
        MODE_FOC_UQ: "FOC_Uq", MODE_FOC_UD: "FOC_Ud",
        MODE_FOC_UALPHA: "FOC_Ualpha", MODE_FOC_UBETA: "FOC_Ubeta",
        MODE_FOC_THETA_GEAR: "Theta_Gear", MODE_FOC_OMEGA_GEAR: "Omega_Gear",
        MODE_FOC_TORQUE_GEAR: "Torque_Gear", MODE_FOC_IQ: "Iq", MODE_FOC_ID: "Id",
        MODE_FOC_IMPEDANCE_SPRING: "Impedance_Spring",
        MODE_FOC_IMPEDANCE_DAMPER: "Impedance_Damper",
        MODE_FOC_IMPEDANCE_INERTIA: "Impedance_Inertia",
        MODE_FOC_IMPEDANCE_SPRING_ORIGIN: "Impedance_Spring_Origin",
        MODE_FOC_CUR_PID_KP: "Cur_Pid_Kp", MODE_FOC_CUR_PID_KI: "Cur_Pid_Ki",
        MODE_FOC_VEL_PID_KP: "Vel_Pid_Kp", MODE_FOC_VEL_PID_KI: "Vel_Pid_Ki",
        MODE_FOC_POS_PID_KP: "Pos_Pid_Kp", MODE_FOC_POS_PID_KI: "Pos_Pid_Ki",
        MODE_FOC_DISPATCH_STATE: "DispatchStateMachine",
        MODE_APP_TRAJ_INIT: "Traj_Init", MODE_APP_TRAJ_DEINIT: "Traj_DeInit",
        MODE_APP_TRAJ_VEL_MAX: "Traj_Vel_Max", MODE_APP_TRAJ_ACL_MAX: "Traj_Acl_Max",
        MODE_APP_TRAJ_POS_CMD: "Traj_Pos_Cmd", MODE_APP_HOMING: "Homing",
        MODE_APP_FLASHING_PARAMS: "FlashingParams", MODE_APP_SYSIDEN: "SysIden",
        MODE_DEVICE_CLEAR_FLASH_ERROR: "ClearFlashError",
    }

    MODE_SCALE = {
        MODE_FOC_THETA_GEAR:              100.0,
        MODE_FOC_OMEGA_GEAR:              100.0,
        MODE_FOC_TORQUE_GEAR:             100.0,
        MODE_FOC_IQ:                      1000.0,
        MODE_FOC_ID:                      1000.0,
        MODE_FOC_IMPEDANCE_SPRING:        10.0,
        MODE_FOC_IMPEDANCE_DAMPER:        100.0,
        MODE_FOC_IMPEDANCE_INERTIA:       1000.0,
        MODE_FOC_IMPEDANCE_SPRING_ORIGIN: 100.0,
        MODE_FOC_CUR_PID_KP:              1000.0,
        MODE_FOC_CUR_PID_KI:              1000.0,
        MODE_FOC_VEL_PID_KP:              1000.0,
        MODE_FOC_VEL_PID_KI:              1000.0,
        MODE_FOC_POS_PID_KP:              1.0,
        MODE_FOC_POS_PID_KI:              1.0,
        MODE_APP_TRAJ_VEL_MAX:            100.0,
        MODE_APP_TRAJ_ACL_MAX:            100.0,
        MODE_APP_TRAJ_POS_CMD:            100.0,
        MODE_FOC_DISPATCH_STATE:          1.0,
    }

    def __init__(self, serial_manager):
        self.sm = serial_manager

    @staticmethod
    def _clamp_int16(v):
        v = int(round(v))
        return max(-32768, min(32767, v))

    @classmethod
    def _canfd_pad(cls, frame):
        n = len(frame)
        for s in cls.CANFD_VALID_SIZES:
            if n <= s:
                return frame + bytes(s - n)
        return frame

    def _quantize(self, mode, value):
        scale = self.MODE_SCALE.get(mode, 1.0)
        return self._clamp_int16(value * scale)

    def build_command_frame(self, mcu, motor0_cmds, motor1_cmds, counter=0, func=None, pad_canfd=None):
        if len(motor0_cmds) != len(motor1_cmds):
            raise ValueError(f"电机0/电机1 指令数必须相等: {len(motor0_cmds)} vs {len(motor1_cmds)}")
        if func is None:
            func = self.RX_FUNC_CMD
        if pad_canfd is None:
            pad_canfd = (func == self.RX_FUNC_FDCAN_CMD)
        target_id = SerialManager.MCU_IDS[SerialManager._to_index(mcu)]
        all_cmds = list(motor0_cmds) + list(motor1_cmds)
        command_num = len(all_cmds)
        if command_num == 0:
            raise ValueError("指令列表为空")
        if command_num > 20:
            raise ValueError(f"命令数量超出下位机上限(20): {command_num}")

        data = bytearray()
        data.append(self.RX_FRAME_HEADER)
        data.append(target_id)
        data.append(counter & 0xFF)
        data.append(func & 0xFF)
        data.append(command_num & 0xFF)
        for mode, param in all_cmds:
            data.append(mode & 0xFF)
            data += struct.pack("<h", self._clamp_int16(param))
        data.append(self.RX_FRAME_TAIL)
        if pad_canfd:
            data = self._canfd_pad(data)
        return bytes(data)

    def send_command(self, mcu, motor0_cmds, motor1_cmds, counter=0):
        m0 = list(motor0_cmds or [])
        m1 = list(motor1_cmds or [])
        n = max(len(m0), len(m1))
        if n == 0:
            print("[发送失败] 指令列表为空")
            return False
        m0 += [(self.MODE_NOP, 0)] * (n - len(m0))
        m1 += [(self.MODE_NOP, 0)] * (n - len(m1))
        try:
            frame = self.build_command_frame(mcu, m0, m1, counter)
        except ValueError as e:
            print(f"[发送失败] {e}")
            return False
        return self.sm.write_data(frame)

    @staticmethod
    def _motor_lists(cmds, motor):
        if motor == "both":
            return list(cmds), list(cmds)
        if motor == 0:
            return list(cmds), []
        if motor == 1:
            return [], list(cmds)
        raise ValueError(f"motor 必须为 0 / 1 / 'both': {motor!r}")

    def _send_action(self, mcu, mode, motor="both"):
        m0, m1 = self._motor_lists([(mode, 0)], motor)
        return self.send_command(mcu, m0, m1)

    def send_state(self, mcu, state, motor="both"):
        if isinstance(state, str):
            if state not in SerialManager.MOTOR_STATE:
                print(f"[发送失败] 未知状态名: {state}")
                return False
            state = SerialManager.MOTOR_STATE[state]
        m0, m1 = self._motor_lists([(self.MODE_FOC_DISPATCH_STATE, int(state))], motor)
        return self.send_command(mcu, m0, m1)

    def send_position(self, mcu, theta_rad, motor="both"):
        cmds = [(self.MODE_FOC_THETA_GEAR, self._quantize(self.MODE_FOC_THETA_GEAR, theta_rad))]
        m0, m1 = self._motor_lists(cmds, motor)
        return self.send_command(mcu, m0, m1)

    def send_velocity(self, mcu, omega, motor="both"):
        cmds = [(self.MODE_FOC_OMEGA_GEAR, self._quantize(self.MODE_FOC_OMEGA_GEAR, omega))]
        m0, m1 = self._motor_lists(cmds, motor)
        return self.send_command(mcu, m0, m1)

    def send_torque(self, mcu, tau, motor="both"):
        cmds = [(self.MODE_FOC_TORQUE_GEAR, self._quantize(self.MODE_FOC_TORQUE_GEAR, tau))]
        m0, m1 = self._motor_lists(cmds, motor)
        return self.send_command(mcu, m0, m1)

    def send_iq(self, mcu, iq, motor="both"):
        cmds = [(self.MODE_FOC_IQ, self._quantize(self.MODE_FOC_IQ, iq))]
        m0, m1 = self._motor_lists(cmds, motor)
        return self.send_command(mcu, m0, m1)

    def send_id(self, mcu, id_val, motor="both"):
        cmds = [(self.MODE_FOC_ID, self._quantize(self.MODE_FOC_ID, id_val))]
        m0, m1 = self._motor_lists(cmds, motor)
        return self.send_command(mcu, m0, m1)

    def send_impedance_spring_origin(self, mcu, origin, motor="both"):
        cmds = [(self.MODE_FOC_IMPEDANCE_SPRING_ORIGIN, self._quantize(self.MODE_FOC_IMPEDANCE_SPRING_ORIGIN, origin))]
        m0, m1 = self._motor_lists(cmds, motor)
        return self.send_command(mcu, m0, m1)

    def send_impedance_params(self, mcu, spring, damper, inertia, motor="both"):
        cmds = [
            (self.MODE_FOC_IMPEDANCE_SPRING, self._quantize(self.MODE_FOC_IMPEDANCE_SPRING, spring)),
            (self.MODE_FOC_IMPEDANCE_DAMPER, self._quantize(self.MODE_FOC_IMPEDANCE_DAMPER, damper)),
            (self.MODE_FOC_IMPEDANCE_INERTIA, self._quantize(self.MODE_FOC_IMPEDANCE_INERTIA, inertia)),
        ]
        m0, m1 = self._motor_lists(cmds, motor)
        return self.send_command(mcu, m0, m1)

    def send_trajectory_vel_max(self, mcu, vel_max, motor="both"):
        cmds = [(self.MODE_APP_TRAJ_VEL_MAX, self._quantize(self.MODE_APP_TRAJ_VEL_MAX, vel_max))]
        m0, m1 = self._motor_lists(cmds, motor)
        return self.send_command(mcu, m0, m1)

    def send_trajectory_acl_max(self, mcu, acl_max, motor="both"):
        cmds = [(self.MODE_APP_TRAJ_ACL_MAX, self._quantize(self.MODE_APP_TRAJ_ACL_MAX, acl_max))]
        m0, m1 = self._motor_lists(cmds, motor)
        return self.send_command(mcu, m0, m1)

    def send_trajectory_pos(self, mcu, pos, motor="both"):
        cmds = [(self.MODE_APP_TRAJ_POS_CMD, self._quantize(self.MODE_APP_TRAJ_POS_CMD, pos))]
        m0, m1 = self._motor_lists(cmds, motor)
        return self.send_command(mcu, m0, m1)

    def send_trajectory(self, mcu, vel_max, acl_max, pos, motor="both"):
        cmds = [
            (self.MODE_APP_TRAJ_VEL_MAX, self._quantize(self.MODE_APP_TRAJ_VEL_MAX, vel_max)),
            (self.MODE_APP_TRAJ_ACL_MAX, self._quantize(self.MODE_APP_TRAJ_ACL_MAX, acl_max)),
            (self.MODE_APP_TRAJ_POS_CMD, self._quantize(self.MODE_APP_TRAJ_POS_CMD, pos))
        ]
        m0, m1 = self._motor_lists(cmds, motor)
        return self.send_command(mcu, m0, m1)

    def send_pos_pid(self, mcu, kp, ki, motor="both"):
        cmds = [(self.MODE_FOC_POS_PID_KP, self._quantize(self.MODE_FOC_POS_PID_KP, kp)),
                (self.MODE_FOC_POS_PID_KI, self._quantize(self.MODE_FOC_POS_PID_KI, ki))]
        m0, m1 = self._motor_lists(cmds, motor)
        return self.send_command(mcu, m0, m1)

    def send_vel_pid(self, mcu, kp, ki, motor="both"):
        cmds = [(self.MODE_FOC_VEL_PID_KP, self._quantize(self.MODE_FOC_VEL_PID_KP, kp)),
                (self.MODE_FOC_VEL_PID_KI, self._quantize(self.MODE_FOC_VEL_PID_KI, ki))]
        m0, m1 = self._motor_lists(cmds, motor)
        return self.send_command(mcu, m0, m1)

    def send_cur_pid(self, mcu, kp, ki, motor="both"):
        cmds = [(self.MODE_FOC_CUR_PID_KP, self._quantize(self.MODE_FOC_CUR_PID_KP, kp)),
                (self.MODE_FOC_CUR_PID_KI, self._quantize(self.MODE_FOC_CUR_PID_KI, ki))]
        m0, m1 = self._motor_lists(cmds, motor)
        return self.send_command(mcu, m0, m1)

    def send_homing(self, mcu, motor="both"):
        return self._send_action(mcu, self.MODE_APP_HOMING, motor)

    def send_flashing_params(self, mcu, motor="both"):
        return self._send_action(mcu, self.MODE_APP_FLASHING_PARAMS, motor)

    def send_traj_init(self, mcu, motor="both"):
        return self._send_action(mcu, self.MODE_APP_TRAJ_INIT, motor)

    def send_traj_deinit(self, mcu, motor="both"):
        return self._send_action(mcu, self.MODE_APP_TRAJ_DEINIT, motor)

    def send_sysiden(self, mcu, motor="both"):
        return self._send_action(mcu, self.MODE_APP_SYSIDEN, motor)

    def send_clear_flash_error(self, mcu, motor="both"):
        return self._send_action(mcu, self.MODE_DEVICE_CLEAR_FLASH_ERROR, motor)

    def send_raw_command(self, mcu, mode, param0, param1):
        m0 = [(mode, param0)] if param0 is not None else []
        m1 = [(mode, param1)] if param1 is not None else []
        return self.send_command(mcu, m0, m1)