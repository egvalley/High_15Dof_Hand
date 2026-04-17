
import math
from cmath import inf
class TendonDriveKinematics:
    """
    腱传动机械臂运动学转换器
    - 正向：给定两个电机的编码值 → 得到关节1角度(theta1)和关节2角度(theta2)
    - 反向：给定两个关节角度 → 得到两个电机的编码值
    关节1就是指根关节，关节2就是指尖关节
    """

    def __init__(self):
        # ---------- 电机编码与角度的转换系数 ----------
        # 电机1编码值 = -theta1_rad * K1，其中 K1 = 10 * 3 * 125 = 3750
        # 125是电机减速比，3是转盘减速比，10是因为上位机位置精确到小数点后一位
        self.ENC1_SCALE = 10 * 3 * 125          # 3750
        # 绳长与电机2编码值的转换：编码值 = 绳长 * K2，K2 = 1250 / 3.87
        #3.87是卷绳箱的半径
        self.ENC2_SCALE = 125*10 / 3.87           # ≈ 322.997

        # ---------- 关节2的“角度→绳长”映射表 ----------
        # 数据格式：(角度(度), 绳长（电机2位置）)
        self.theta2_length_table = [
            (0, 0),
            (15, 1211),
            (30, 2550),
            (45, 3991),
            (60, 5500),
            (75, 7029),
            (90, 8502)
        ]

        # ---------- 关节1几何模型参数（来自原 get_length_from_theta1） ----------
        # 该模型计算关节1在不同角度下对绳长的“基线影响量”
        self.r1 = 5 / 2          # 2.5
        self.R1 = 6 / 2          # 3.0
        self.a1 = 4.96
        self.b1 = 17.11
        self.c1 = 12.59
        self.d1 = 6.79
        # 该模型计算关节2在不同角度下对绳长的“基线影响量”
        self.r2 = 5 / 2
        self.R2 = 9
        self.L2 = 22
        self.b2 = 4.972376
        self.a2 = 10.506368
        self.c2 = 11.623609

        # 关节1安装偏角修正（原代码中 theta += 6/360*2π）（关节1并非完全与水平线平行）
        self.theta1_offset_rad = 6 / 360 * 2 * math.pi   # 6度偏移

    # ================== 内部辅助方法 ==================
    def _signed16(self, value: int) -> int:
        """将16位无符号整数转换为有符号整数（-32768..32767）"""
        if value >= 0x8000:
            return value - 0x10000
        return value

    def _encode_motor1(self, theta1_rad: float) -> int:
        """关节1弧度 → 电机1编码值（0～65535）"""
        raw = int(-theta1_rad * self.ENC1_SCALE)
        return raw & 0xFFFF

    def _decode_motor1(self, enc1: int) -> float:
        """电机1编码值 → 关节1弧度（有符号）"""
        signed = self._signed16(enc1)
        return -signed / self.ENC1_SCALE

    def _length_to_enc2(self, length: float) -> int:
        """绳长 → 电机2编码值（0～65535）"""
        raw = int(length * self.ENC2_SCALE)
        return raw & 0xFFFF

    def _length_to_theta2(self, length: float) -> float:
        """绳长 → 关节2角度（度，线性插值）"""
        table = self.theta2_length_table
        if length <= table[0][1]:
            return table[0][0]
        if length >= table[-1][1]:
            return table[-1][0]
        for i in range(len(table) - 1):
            a1, l1 = table[i]
            a2, l2 = table[i + 1]
            if l1 <= length <= l2:
                ratio = (length - l1) / (l2 - l1)
                return a1 + ratio * (a2 - a1)
        raise RuntimeError("Unexpected interpolation error")

    def _length_from_theta2(self, theta2_rad: float) -> float:
        """
        关节2的几何模型：给定 theta2 弧度，返回与theta2对应绳长基线的影响值（长度单位）。
        """
        alpha = math.atan(self.b2 / self.a2)  # pi
        l1 = math.sqrt((self.c2 * math.cos(theta2_rad + alpha) + self.L2) ** 2 + (self.c2 * math.sin(theta2_rad + alpha)) ** 2 - (self.R2 + self.r2) ** 2)
        Alpha = math.atan((self.c2 * math.sin(theta2_rad + alpha)) / (self.c2 * math.cos(theta2_rad + alpha) + self.L2))
        beta = math.atan((self.R2 + self.r2) / l1)
        theta_sec = Alpha - beta
        cisser1 = math.pi / 2 - theta_sec - theta2_rad
        l2 = cisser1 * self.R2
        cisser2 = theta2_rad - theta_sec
        l3 = cisser2 * self.r2
        return l1 + l2 + l3

    def _length_from_theta1(self, theta1_rad: float) -> float:
        """
        关节1的几何模型：给定 theta1 弧度，返回与theta1对应绳长基线的影响值（长度单位）。
        """
        theta = theta1_rad + self.theta1_offset_rad   # 安装偏角修正
        r, R = self.r1, self.R1
        a, b, c, d = self.a1, self.b1, self.c1, self.d1

        l1 = math.sqrt((b * math.cos(theta) + c - a * math.sin(theta)) ** 2 +
                       (b * math.sin(theta) + a * math.cos(theta) - d) ** 2 -
                       (R + r) ** 2)
        beta = math.atan((R + r) / l1)
        theta_sec = math.atan2(b * math.sin(theta) + a * math.cos(theta) - d,
                               b * math.cos(theta) + c - a * math.sin(theta))
        alpha = beta - theta_sec
        c1 = theta + alpha
        c2 = alpha
        l2 = r * c1
        l3 = R * c2
        return l1 + l2 + l3

    # ================== 公共接口 ==================
    def Motor_enc_to_angles(self, enc1: int, enc2: int):
        enc1=int(65536+enc1*10)
        enc2 = int(enc2 * 10)
        # 1. 解码电机1 → theta1
        theta1_rad = self._decode_motor1(enc1)
        theta1_deg = math.degrees(theta1_rad)

        # 2. 计算与 theta1 有关的基线偏移编码值 tmp
        L0 = self._length_from_theta1(0.0)                     # theta1=0 时的基线长度
        L_theta1 = self._length_from_theta1(theta1_rad)       # 当前 theta1 时的基线长度
        delta_length = abs(L0 - L_theta1)                      # 长度差绝对值
        tmp_enc = self._length_to_enc2(delta_length)           # 转换为电机2编码偏移

        # 3. 修正电机2编码，得到纯弯曲对应的编码
        corrected_enc2 = enc2 - tmp_enc
        if corrected_enc2 < 0:
            print("绳子太松")
            theta2_deg = 0          # 边界保护,否则关节2的绳子会多放出来
        elif corrected_enc2 > 8502:
            print("绳子太紧")
            theta2_deg = inf        # 边界保护，否则关节2的绳子会崩得很紧

        # 4. 解码为绳长，再插值得 theta2
        theta2_deg = self._length_to_theta2(corrected_enc2)
        enc2 = math.radians(theta2_deg)
        enc1=math.radians(theta1_deg)
        return enc1,enc2

    def Angles_to_motor_enc(self, theta1_deg: float, theta2_deg: float):
        """
        反向运动学：关节角度 → 电机编码

        :return: (enc1, enc2)
        """
        # 1. 关节1弧度 → 电机1编码
        theta1_rad = theta1_deg
        enc1 = self._encode_motor1(theta1_rad)
        # theta2_rad = math.radians(theta2_deg)
        theta2_rad = theta2_deg

        # 2. 计算与 theta1 有关的基线偏移编码值 tmp
        L1_0 = self._length_from_theta1(0.0)
        L1_theta1 = self._length_from_theta1(theta1_rad)
        L2_0 = self._length_from_theta2(0.0)
        L2_theta2 = self._length_from_theta2(theta2_rad)
        delta_length = L1_0+L2_0 - L1_theta1-L2_theta2
        enc2 = self._length_to_enc2(delta_length)
        if(enc1==0):
            enc1=0.0
        else:
            enc1=int((enc1-65536))/10

        enc2 = int(enc2) / 10
        return enc1, enc2


# ================== 使用示例 ==================
if __name__ == "__main__":
    kin = TendonDriveKinematics()

    # 示例1：给定电机编码 → 角度
    enc1, enc2 =0 ,849.8 #[rad rad]
    theta1, theta2 = kin.Motor_enc_to_angles(enc1, enc2)
    print(f"正向: 电机累积enc1={enc1:.2f}rad, 电机累积enc2={enc2:.2f}rad → theta1={theta1:.4f}rad, theta2={theta2:.4f}rad")


    # 示例3：测试其他角度
    theta1_test, theta2_test = 1.57,1.57
    enc1_test, enc2_test = kin.Angles_to_motor_enc(theta1_test, theta2_test)
    theta1_back, theta2_back = kin.Motor_enc_to_angles(enc1_test, enc2_test)
    print(f"\n测试: 目标角度 ({theta1_test}rad, {theta2_test}rad) → 编码 ({enc1_test}, {enc2_test}) → 解码 ({theta1_back}rad, {theta2_back}rad)")