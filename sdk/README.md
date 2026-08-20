# High_15Dof_Hand（新协议版）

按“协议事实来源（下位机源码）优先、逐层解耦”的原则组织。本版已适配**新版下位机协议**
（`router_common.*` / `router_rec_handle.*` / `router_tran_handle.*`）。

## 目录结构

```
High_15Dof_Hand/
├─ high_dof_gui.py               # 仅界面 + 读表单 + 调 controller + 刷新
└─ sdk/
   ├─ config.py                  # 串口默认参数、Rx Func、曲线数量、刷新率、阈值
   ├─ models.py                  # dataclass：命令、反馈、统计、结果、目标枚举（不依赖协议层）
   ├─ serial_manager.py          # 串口 IO + 接收线程 + 线程安全快照
   ├─ serial_commander.py        # 控制语义 → 量化 → 按电机取 mode → 编码 → 发送
   ├─ hand_controller.py         # 选哪些 MCU / 批量发送 / 汇总结果
   └─ protocol/                  # 协议层，按职责分四块，改动只碰对应的那一个
      ├─ __init__.py             # 包门面：外部一律 `from sdk.protocol import XXX`
      ├─ wire.py                 # 帧格式与功能码：字节怎么排、Func 填什么
      ├─ topology.py             # MCU 拓扑 + CurveLayout：第 N 个 32 位字是谁的哪个量、什么类型
      ├─ info_word.py            # 电机信息字位域：反馈里的状态码与错误怎么拆
      ├─ commands.py             # 控制命令码、定点缩放、状态机码（状态码表两侧共用）
      ├─ rx_command_codec.py     # 上位机→MCU 命令帧编码
      └─ tx_feedback_codec.py    # MCU→上位机 反馈帧拼帧/解析
```

依赖只朝下：`protocol/` 不认识 `models`（`wire → info_word/topology/commands → 两个 codec`），
`models.py` 是纯数据结构不认识 `protocol`，GUI 只和 `controller / manager / models` 打交道。

## 新旧协议差异（本次适配的全部改动点）

| | 旧协议 | 新协议（本版） |
|---|---|---|
| Rx 帧 | `0x3B`…`0x1E`，头 5 字节 | `0xAA`…`0xBB`，头 7 字节（帧头+ID+Func+4 字节时间戳） |
| Rx 命令 | mode 1 字节 + param int16，≤20 条 | **mode uint16** + param int16，≤**14** 条 |
| 电机区分 | 命令列表**前/后半段**，短的一半用 NOP 补齐 | **mode 自带电机后缀**：电机1 = 电机0 + **300**，一帧内任意混排 |
| Tx 帧 | `0x5A` + 曲线数 N + log2max 表 + 帧计数 + 时间戳 + 采样点数 + N×行 int16 + CRC16 + `0x0D` | `0xCC` + ID + Func + **4 字节帧计数** + **N×32 位字** + `0xDD`（**无长度字段、无 CRC**） |
| Tx 数值 | int16 定点，按 log2max 反量化 | 每条曲线一个 32 位原始字，**按注册类型解**（本表：信息字 uint32，其余 float32） |
| Tx 电机状态/错误 | 第 0 条曲线是**状态码**（单值，查表取名），错误也占状态码 | 第 0 条曲线换成 **uint32 信息字**：高字节仍是状态码，低 24 位是**错误位域**，可同时报多个错误，也可一个不报 |
| Tx 采样 | 一帧可含多行采样点 | **一帧一个采样点**（低速 1kHz 采样即发） |
| Func | 收发共用一套 0~5 | 收发各自一套：`TxFunc` 已收敛成**只标速度策略不标总线**的 `NORMAL(0)/HIGH(1)`，`RxFunc`(3~4) |
| 功能码 | 位置 6 / 速度 7 / 力矩 8 / … | 全部重编号：位置 206 / 速度 207 / 力矩 208 / …（见下） |

### 功能码与缩放（逐条取自 `router_rec_handle.c` 的 handler）

电机0 用表中数值，电机1 = 数值 + 300（`MotorFunc.code_for(motor)`）。

| 功能 | mode(M1) | scale | 固件换算 |
|---|---|---|---|
| 位置 / 速度 | 206 / 207 | ×100 | `param/100` |
| 力矩 | 208 | ×1 | `param`（mN·m） |
| Iq / Id | 209 / 210 | ×1000 | `param·0.001` |
| MIT 刚度/阻尼/惯量 | 232~234 | **×10** / ×1000 / ×10000 | 原"阻抗"三系数，固件改名 MIT；刚度定标**已变**（旧 ×100，出端默认约 450 mN·m/rad 装不进 int16） |
| **MIT 位置 / 速度 / 扭矩前馈** | **235 / 236 / 237** | ×100 / ×100 / ×1 | 235 **语义变了**（旧为弹簧原点）；236/237 **新增**。位置/速度与 206/207 走固件同一条出端通道 |
| 电流环、速度环 PID | 252~255 | ×1000 | `param·0.001` |
| 位置环 PID | 256 / 257 | ×1 | `param` |
| 清内环 / 清外环错误 | 270 / 271 | — | 无参数；各清信息字的一段错误位域（bit8~11 / bit12~15） |
| 状态派发 | 272 | ×1 | param = 状态码 |
| 轨迹 init/deinit/vmax/amax/pos | 282~286 | — / — / ×100 / ×100 / ×100 | |
| **清轨迹规划错误** | **287** | — | 新增；清信息字 bit0~bit7 |
| 回零 init/前力矩/**反力矩**/**前位置**/反位置 | 301~305 | — / ×1 / ×1 / ×100 / ×100 | 新固件把回零参数拆成 **4 条**（反向力矩、前向位置为新增） |
| 系统辨识 | 311 | — | 常量已列，GUI 暂未开按钮 |
| 刷参 / 清 Flash 错误 | 321 / 322 | ×1 / — | Flash 是共享设备，M1/M2 两个 mode 同效 |
| **清编码器错误** | **323** | — | 清信息字 bit16~**bit19** 并重新布防后台 DMA 采样，M1/M2 各清自己那路 SPI |

未纳入：`Uq/Ud/Ualpha/Ubeta`(202~205) —— 固件 Dispatch 的 switch 里**没有对应 case**。

## 电机信息字（本次协议改动的核心）

反馈里每个电机的第 0 条曲线，uint32，**一个字同时带状态和错误**
（固件 `MotorAppInstance_Frequency1KhzCallBack_Mx` 每 1ms 拼出来，见 `motor_app_instance.h`）。
两段语义完全不同，所以监控页**分两列显示**：状态段是单值查表取名，错误段是位域逐位拆。

| 段 | 位域 | 来源枚举 | 内容 | 清除命令 |
|---|---|---|---|---|
| 状态 | bit24~bit31 | `MotorStateType` | 状态机当前状态码（单值） | —（每帧实时值，不需要也不能清） |
| App：轨迹规划 | bit0~bit7 | `MotorTrajPlanErrorSystem` | 执行中收到新指令(已丢弃) / 工作状态错误（电机不在位置控制；MIT 态**已不再**被轨迹接受） | **287** |
| FOC：内环 | bit8~bit11 | `InnerOuterErrorSystem` | 控制参数丢失 / 编码器数据丢失 / 电流采样丢失 | 270 |
| FOC：外环 | bit12~bit15 | `InnerOuterErrorSystem` | 控制参数丢失 / 参数存 Flash 失败 | 271 |
| Device：编码器 | bit16~**bit19** | `EncoderErrorSystem`（`encoder.h`） | DMA 传输失败 / DMA 采样周期内未搬完 / SPI 无效帧 / 帧 CRC8 校验失败 | 323 |

（bit20~bit23 是 Device 段里固件尚未用到的位，恒为 0，上位机拿它当定帧校验的一部分。）

> 编码器错误段随固件的 DMA 采样流水线改造**整段重排**（`encoder_common.h` → `encoder.h`）：
> 旧的 bit16「SPI 断连」/ bit17「SPI 数据 CRC 失败」两条已被上表四条替换，段掩码
> `ErrorSeg.ENCODER` 同步从 `0x00030000` 放宽到 `0x000F0000`。旧上位机对着新固件跑，
> bit18/bit19 会被 `is_info_word` 当成越界位而整帧丢掉，看起来像掉线。

- **状态与错误各说各的**：内外环任一错误置位，固件把状态机切到
  `InnerOuterCommonError(164)`，**具体错在哪只能看错误位域** —— 旧固件"每种错误各占一个状态码"
  的 150~155 已删除，`MotorState.CODES` 里同步只留 164/165。
- **错误位是粘滞的**：置位后一直保持，不会因为下一次采样正常而自动消失，必须上位机显式清错。
  这是固件有意为之——避免间歇性断线在上位机读到之前被下一次正常采样悄悄抹掉。
- 定义在 `protocol/info_word.py` 的 `ErrorBit`（成员值即掩码，`.label` 是显示名）。
  固件新增错误码时**在枚举里加一行**即可；若加到了 `ErrorSeg` 四段之外的位，还要同步放宽段掩码，
  否则 `is_info_word` 会把正常帧当成定帧错位丢掉。
- 段内出现未命名的位不算错帧，显示成 `未知位(bitN)`，不会被静默吞掉。
- 状态段则相反：**未知状态码照常显示**成 `Unknown(n)` 而不丢帧 —— 状态机刚上电还没跑到
  `StartupReady` 时状态值就是 0，认死状态码表会把这些帧全丢掉，现象像掉线。

## 数据流

控制：GUI → HandController（选 MCU/电机、批量）→ SerialCommander（量化、按电机取 mode）
→ RxCommandCodec（组帧 `0xAA`…`0xBB` + CAN-FD 补齐）→ SerialManager.write_data。

反馈：SerialManager 接收线程 → TxFeedbackCodec（定长拼帧、校验、按 `CurveLayout` 逐条按类型取值、
信息字拆成状态码 + 错误位）→ 线程安全 McuFeedback 快照 → GUI 刷新 TreeView（状态 / 错误两列）。

## 两个必须知道的实现约束

1. **Tx 帧长与曲线类型都靠“曲线表”推**：新 Tx 帧里既没有长度字段也没有 CRC，
   帧长 = `7 + 4×曲线数 + 1`，每条曲线是 float 还是 uint32 也只能靠约定。
   两者都由 `topology.CurveLayout` 从 `MOTOR_CURVES` 推出，默认 10 条
   （2 电机 × 信息字/θ/ω/a/τ），与固件 `RouterAppFixFreq_Init` 的低速注册表一致。
   **改固件曲线注册表，必须同步改 `MOTOR_CURVES`** 与 `AppConfig.feedback_curve_count`；
   曲线数量对不上时布局退回“全 float32”，电机字段映射不出来但原始值仍在 `curves` 里。
   因为没有 CRC，定帧靠“帧头 + ID 合法(0xB0~0xB7) + Func 合法 + 帧尾就位 + 曲线值合理”
   这一组条件，任一不满足就丢 1 字节重新找帧头（CAN-FD 的补零字节也这样跳过）。
   其中“曲线值合理”按类型分别判：float 曲线要有限（非 NaN/Inf），
   信息字要落在“状态段 + 四段错误位域”之内 —— 后者比纯浮点检查更强，提高了定帧可靠性。
2. **本工程只用低速策略**（`app_config.h: ROUTER_TX_LOW_SPEED`）：1kHz 采样即发。
   新固件的 `RouterTxFuncCode` 只剩 `NORMAL(0)/HIGH(1)`（不再按总线区分），
   低速帧无论走 USB/USART 还是 FDCAN，Func 都是 `0`，解码器只接受它。
   新帧不带时间戳，`hw_time = 帧计数器 / 1kHz`（帧计数器每采样点 +1）。
   若改回高速策略，放开 `wire.TX_ACCEPTED_FUNCS` 并改采样频率即可。

## 改帧头/帧尾时的唯一约束：帧尾不能等于任何 mode 的低字节

`router_rec_handle.c` 定位命令条数的办法是从 `payload[7]` 起**每 4 字节看一次，遇到
`RX_FRAME_TAIL` 就认为帧结束**：

```c
while (cmd_cnt < max_cmd_cnt && payload[RX_FRAME_HEADER_SIZE + cmd_cnt * 4] != RX_FRAME_TAIL) cmd_cnt++;
```

而每条命令的第 1 字节正是 `Control_Mode` 的**低字节**。所以帧尾一旦等于某个 mode 的低字节，
那条命令就会被误判成帧尾——它自己**和它之后的所有命令一起被静默丢弃**，
且上位机侧**无法绕过**（每条命令都落在扫描点上）。

- 现行 `RX_FRAME_TAIL = 0xBB`：全部 62 个 mode（M1/M2）的低字节都不等于 `0xBB`，**无冲突**
  （已遍历核对，不是抽查）。`RouterRxControlMode` **全枚举**
  （含 SDK 未用的 `Uq/Ud/Ualpha/Ubeta` 202~205 / 502~505）的 mode 低字节占用区间为：
  `0x00~0x01 / 0x0E~0x10 / 0x14~0x17 / 0x1A~0x1E / 0x28~0x31 / 0x37 / 0x3A~0x3C /
  0x41~0x42 / 0x46~0x4A / 0x59~0x5D / 0x63 / 0x6D~0x6E / 0xCA~0xD2 / 0xE8~0xEB /
  0xF6~0xFF`——以后挑帧尾避开这些即可（`0xBB` 不在其中）。
- 反例：帧尾若取 `0x1C`，则 `Traj_Vel_Max_M1 = 284 = 0x011C` 会中招，含 vmax 的轨迹命令
  全部发不出去。
- `RxCommandCodec` 保留了组帧前的护栏（`commands.TAIL_CONFLICT_MODES`）：**日后改帧尾值
  或加新 mode 时若又踩坑，会直接抛错**而不是无声失效。

`0xAA / 0xBB / 0xCC / 0xDD` 这组还顺带避开了 `0x0A`(`\n`)、`0x0D`(`\r`)、`0x1A`、`0x00`——
链路中途若有按行/文本处理的环节（ASCII 模式 SLCAN、做 CRLF 转换的转发程序、文本日志管道），
这些字节容易被改写或截断，现在四个定界符都不在其中。

## 已核对无需改动的部分

- `MotorState.CODES` 与新固件 `motor_state_machine.h` 的 `MotorStateType` 逐条一致；
  它同时供**下发**（`DISPATCH_STATE` 的参数）与**解析**（信息字高字节）两侧使用。
  本次随固件删掉了 150~155 那批"一种错误一个状态码"，补上 `InnerOuterCommonError(164)`。
- MCU 节点号：`router_common.h` 的 `RouterCommObject` 已补齐 `MCU0~MCU7 = 0xB0~0xB7`，
  与 `McuConfig.COUNT = 8` 一致（旧版“只定义到 MCU5”的疑问已消除）。
- Rx Func 固件仍不校验，默认沿用 `RxFunc.FDCAN_CMD`，`config.py` 可一键切 USB_USART。
- 命令帧仍在编码末尾恒定补 0 到下一个 CAN-FD 合法长度（8/12/16/20/24/32/48/64）。

## 运行

```bash
python high_dof_gui.py
```
