"""
电机信息字 —— 反馈里每个电机的第 0 条曲线 (uint32 位域，不是 float)。

固件 MotorAppInstance_Frequency1KhzCallBack_Mx 每 1ms 把四段东西拼进这一个字上报
(motor_app_instance.h 的 motorX_info_word)，各段互不重叠：

    bit24~bit31  电机状态机状态码   MotorStateType           (motor_state_machine.h)
    bit16~bit23  Device 层错误      目前只有编码器 bit16~17   (encoder_common.h)
    bit8 ~bit15  FOC 错误           内环 bit8~11 / 外环 bit12~15 (motor_error_handle.h)
    bit0 ~bit7   App 层错误         目前只有轨迹规划 bit0~1   (motor_trajectory_planning.h)

★ 两段语义完全不同，所以上位机分两列显示：
    状态段是【单值】—— 一个状态码，查 commands.MotorState 取名；
    错误段是【位域】—— 每个错误独占 1 位，可以同时报若干个，也可以一个都不报 (全 0 = 无错误)。

错误位置位后一直保持，不会因为下一次采样正常而自动消失，只能由上位机显式下发对应的清错命令
清掉，一条命令清一段 (见 commands.MotorFunc 的 CLEAR_INNER/OUTER/ENCODER_ERROR 与
TRAJ_CLEAR_ERROR)。状态段则是每帧实时值，不需要也不能"清"。
"""

from enum import IntEnum


# 状态段：状态码本身 0~255，含义见 commands.MotorState
INFO_STATE_MASK  = 0xFF000000
INFO_STATE_SHIFT = 24


class ErrorSeg(IntEnum):
    """
    错误段的四段位域范围，逐条对应固件各模块自己的错误掩码。

    段内未用到的位留给固件后续扩展 (如 App 段现在只用了 bit0~1，仍按固件的
    MOTOR_TRAJ_PLAN_ERROR_MASK 认整个低字节)。
    """

    APP     = 0x000000FF    # bit0 ~ bit7   MOTOR_TRAJ_PLAN_ERROR_MASK
    INNER   = 0x00000F00    # bit8 ~ bit11  MOTOR_ERROR_INNER_MASK
    OUTER   = 0x0000F000    # bit12~ bit15  MOTOR_ERROR_OUTER_MASK
    ENCODER = 0x00030000    # bit16~ bit17  ENCODER_ERROR_MASK


class ErrorBit(IntEnum):
    """
    错误段里的每一位：成员值即掩码，`.label` 是显示名。

    枚举顺序即 GUI 里多个错误并列时的显示顺序。固件新增错误码时在此加一行即可，
    加到 ErrorSeg 段外的位还要同步放宽对应的段掩码 (否则整帧会被当成错位帧丢掉)。
    """

    def __new__(cls, mask, label):
        """让成员既是位掩码 int(bit)，又携带显示名 .label。"""
        obj = int.__new__(cls, mask)
        obj._value_ = mask
        obj.label = label
        return obj

    #                            掩码       显示名
    # —— App 段 bit0~bit7：轨迹规划 (MotorTrajPlanErrorSystem) ——
    TRAJ_RUNNING_BUSY        = (1 << 0,  "轨迹:执行中收到新指令(已丢弃)")
    TRAJ_WORKING_BROKEN      = (1 << 1,  "轨迹:电机不在位置/阻抗控制")
    # —— FOC 内环 bit8~bit11 (InnerOuterErrorSystem) ——
    INNER_CONTROL_PARAMS     = (1 << 8,  "内环:控制参数丢失")
    INNER_ENCODER_READ       = (1 << 9,  "内环:编码器数据丢失")
    INNER_CURRENT_SAMP       = (1 << 10, "内环:电流采样丢失")
    # —— FOC 外环 bit12~bit15 (InnerOuterErrorSystem) ——
    OUTER_CONTROL_PARAMS     = (1 << 12, "外环:控制参数丢失")
    OUTER_FLASH_PARAMS_SAVE  = (1 << 13, "外环:参数存 Flash 失败")
    # —— Device 段 bit16~bit23：编码器 (EncoderErrorSystem) ——
    ENCODER_SPI_CONNECT      = (1 << 16, "编码器:SPI 断连")
    ENCODER_SPI_CRC          = (1 << 17, "编码器:SPI 数据 CRC 失败")


# 四段错误位域合起来的合法掩码 (bit18~bit23 是 Device 段里固件还没用到的位，故不在其中)
ERROR_VALID_MASK = ErrorSeg.APP | ErrorSeg.INNER | ErrorSeg.OUTER | ErrorSeg.ENCODER

# 信息字整体的合法掩码：状态段 + 错误段。这之外一旦有置位，这 4 字节就不是信息字
INFO_VALID_MASK = INFO_STATE_MASK | ERROR_VALID_MASK

# 所有【已命名】的错误位。段内出现未命名的位不算错帧，只是本上位机还不认识它
ERROR_KNOWN_MASK = 0
for _bit in ErrorBit:
    ERROR_KNOWN_MASK |= _bit
del _bit


def is_info_word(word):
    """
    这 4 字节像不像一个信息字。帧里没有 CRC，这是定帧的合理性检查之一。

    只查"合法位域之外有没有置位"——状态段 0~255 全都放行 (状态机刚上电还没跑到
    StartupReady 时状态值就是 0，认死状态码表会把这些帧全丢掉，反而像掉线)。
    """
    return (word & ~INFO_VALID_MASK) == 0


def state_of(word):
    """信息字 -> 电机状态机状态码 (bit24~bit31)；取名用 commands.MotorState.name。"""
    return (word & INFO_STATE_MASK) >> INFO_STATE_SHIFT


def errors_of(word):
    """信息字 -> 只剩错误段的字 (bit0~bit23)；为 0 即无任何错误。"""
    return word & ~INFO_STATE_MASK


def error_names(word):
    """
    信息字 (或已剥掉状态段的错误字) -> 已置位错误的显示名列表；无错误返回空列表。

    状态段在这里被剥掉，免得状态码被当成错误位拆出来；未命名的位不吞掉，显示成
    未知位(bitN)，这样固件加了新错误码、上位机还没跟上时也能看见。
    """
    errors = errors_of(word)
    if not errors:
        return []
    names = [bit.label for bit in ErrorBit if errors & bit]
    unknown = errors & ~ERROR_KNOWN_MASK
    names += [f"未知位(bit{i})" for i in range(32) if unknown & (1 << i)]
    return names
