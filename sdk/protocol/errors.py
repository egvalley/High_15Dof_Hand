"""
电机错误字 —— 反馈里每个电机的第 0 条曲线 (uint32 位域，不是 float)。

新协议用它取代了旧协议的"状态码"曲线：不再是"当前处于哪个错误态"这种单值，
而是【每个错误独占 1 个 bit】，可以同时报若干个，也可以一个都不报 (全 0 = 无错误)，
所以解读方式是逐位拆，不是查表取名。

位域分三段，逐条对应固件枚举，三段互不重叠：
    bit0 ~ bit7    InnerErrorSystem    内环   (motor_error_handle.h)
    bit8 ~ bit15   OuterErrorSystem    外环   (motor_error_handle.h)
    bit16~ bit17   EncoderErrorSystem  编码器 (encoder_common.h)
固件由 MotorAppInstance_Frequency1KhzCallBack_Mx 把三段或在一起上报。

错误位置位后一直保持，不会因为下一次采样正常而自动消失，只能由上位机显式下发对应的
清错命令清掉 (见 commands.MotorFunc 的 CLEAR_INNER/OUTER/ENCODER_ERROR)。
"""

from enum import IntEnum


class ErrorSeg(IntEnum):
    """错误字的三段位域范围。段内未用到的位留给固件后续扩展。"""

    INNER   = 0x000000FF    # bit0 ~ bit7
    OUTER   = 0x0000FF00    # bit8 ~ bit15
    ENCODER = 0x00030000    # bit16~ bit17


class ErrorBit(IntEnum):
    """
    错误字里的每一位：成员值即掩码，`.label` 是显示名。

    枚举顺序即 GUI 里多个错误并列时的显示顺序。固件新增错误码时在此加一行即可，
    加到 ErrorSeg 段外的位还要同步放宽对应的段掩码。
    """

    def __new__(cls, mask, label):
        """让成员既是位掩码 int(bit)，又携带显示名 .label。"""
        obj = int.__new__(cls, mask)
        obj._value_ = mask
        obj.label = label
        return obj

    #                            掩码       显示名
    INNER_ENCODER_READ       = (1 << 0,  "内环:编码器读取失败")
    INNER_CURRENT_SAMP       = (1 << 1,  "内环:电流采样失败")
    INNER_CONTROL_PARAMS     = (1 << 2,  "内环:控制参数加载失败")
    OUTER_CONTROL_PARAMS     = (1 << 8,  "外环:控制参数加载失败")
    OUTER_THETAE_CALIB_SAVE  = (1 << 9,  "外环:电角度标定保存失败")
    OUTER_CURRENT_CALIB_SAVE = (1 << 10, "外环:电流标定保存失败")
    ENCODER_SPI_CONNECT      = (1 << 16, "编码器:SPI 断连")
    ENCODER_SPI_CRC          = (1 << 17, "编码器:SPI 数据 CRC 失败")


# 三段位域合起来的合法掩码。定帧时用：这之外一旦有置位，这 4 字节就不是错误字 (帧错位)
ERROR_VALID_MASK = ErrorSeg.INNER | ErrorSeg.OUTER | ErrorSeg.ENCODER

# 所有【已命名】的位。段内出现未命名的位不算错帧，只是本上位机还不认识它
ERROR_KNOWN_MASK = 0
for _bit in ErrorBit:
    ERROR_KNOWN_MASK |= _bit
del _bit

def is_error_word(word):
    """这 4 字节像不像一个错误字 (三段位域之外无置位)。仅用于无 CRC 时的定帧合理性检查。"""
    return (word & ~ERROR_VALID_MASK) == 0


def error_names(word):
    """
    错误字 -> 已置位错误的显示名列表；全 0 返回空列表 (无错误)。
    未命名的位不吞掉，显示成 未知位(bitN)，这样固件加了新错误码也能看见。
    """
    if not word:
        return []
    names = [bit.label for bit in ErrorBit if word & bit]
    unknown = word & ~ERROR_KNOWN_MASK
    names += [f"未知位(bit{i})" for i in range(32) if unknown & (1 << i)]
    return names
