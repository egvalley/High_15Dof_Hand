# High_15Dof_Hand（新协议版）

按“协议事实来源（下位机源码）优先、逐层解耦”的原则组织。本版已适配**新版下位机协议**
（`router_common.*` / `router_rec_handle.*` / `router_tran_handle.*`）。

## 目录结构

```
High_15Dof_Hand/
├─ high_dof_gui.py               # 仅界面 + 读表单 + 调 controller + 刷新
└─ sdk/
   ├─ config.py                  # 串口默认参数、Rx Func、曲线数量、刷新率、阈值
   ├─ models.py                  # dataclass：命令、反馈、统计、结果、目标枚举
   ├─ serial_manager.py          # 串口 IO + 接收线程 + 线程安全快照
   ├─ serial_commander.py        # 控制语义 → 量化 → 按电机取 mode → 编码 → 发送
   ├─ hand_controller.py         # 选哪些 MCU / 批量发送 / 汇总结果
   └─ protocol/
      ├─ constants.py            # 唯一协议源：帧头尾、Func、MCU、模式、缩放、状态机
      ├─ rx_command_codec.py     # 上位机→MCU 命令帧编码
      └─ tx_feedback_codec.py    # MCU→上位机 反馈帧拼帧/解析
```

## 新旧协议差异（本次适配的全部改动点）

| | 旧协议 | 新协议（本版） |
|---|---|---|
| Rx 帧 | `0x3B`…`0x1E`，头 5 字节 | `0xAA`…`0xBB`，头 7 字节（帧头+ID+Func+4 字节时间戳） |
| Rx 命令 | mode 1 字节 + param int16，≤20 条 | **mode uint16** + param int16，≤**14** 条 |
| 电机区分 | 命令列表**前/后半段**，短的一半用 NOP 补齐 | **mode 自带电机后缀**：电机1 = 电机0 + **300**，一帧内任意混排 |
| Tx 帧 | `0x5A` + 曲线数 N + log2max 表 + 帧计数 + 时间戳 + 采样点数 + N×行 int16 + CRC16 + `0x0D` | `0xCC` + ID + Func + **4 字节帧计数** + **N×float32** + `0xDD`（**无长度字段、无 CRC**） |
| Tx 数值 | int16 定点，按 log2max 反量化 | **裸 float32**，直接取值 |
| Tx 采样 | 一帧可含多行采样点 | **一帧一个采样点**（低速 1kHz 采样即发） |
| Func | 收发共用一套 0~5 | 收发**各自一套**且数值重叠 → 拆成 `TxFunc`(0~3) / `RxFunc`(3~4) |
| 功能码 | 位置 6 / 速度 7 / 力矩 8 / … | 全部重编号：位置 206 / 速度 207 / 力矩 208 / …（见下） |

### 功能码与缩放（逐条取自 `router_rec_handle.c` 的 handler）

电机0 用表中数值，电机1 = 数值 + 300（`MotorFunc.code_for(motor)`）。

| 功能 | mode(M1) | scale | 固件换算 |
|---|---|---|---|
| 位置 / 速度 | 206 / 207 | ×100 | `param/100` |
| 力矩 | 208 | ×1 | `param`（mN·m） |
| Iq / Id | 209 / 210 | ×1000 | `param·0.001` |
| 阻抗 刚度/阻尼/惯量/原点 | 232~235 | ×100 / **×1000** / **×10000** / ×100 | 阻尼、惯量的换算**已变**（旧为 ×100 / ×1000） |
| 电流环、速度环 PID | 252~255 | ×1000 | `param·0.001` |
| 位置环 PID | 256 / 257 | ×1 | `param` |
| 清内环 / 清外环错误 | 270 / 271 | — | 无参数 |
| 状态派发 | 272 | ×1 | param = 状态码 |
| 轨迹 init/deinit/vmax/amax/pos | 282~286 | — / — / ×100 / ×100 / ×100 | |
| 回零 init/前力矩/**反力矩**/**前位置**/反位置 | 301~305 | — / ×1 / ×1 / ×100 / ×100 | 新固件把回零参数拆成 **4 条**（反向力矩、前向位置为新增） |
| 系统辨识 | 311 | — | 常量已列，GUI 暂未开按钮 |
| 刷参 / 清 Flash 错误 | 321 / 322 | ×1 / — | |

未纳入：`Uq/Ud/Ualpha/Ubeta`(202~205) —— `router_rec_handle.c` 的 switch 里**没有对应 case**。

## 数据流

控制：GUI → HandController（选 MCU/电机、批量）→ SerialCommander（量化、按电机取 mode）
→ RxCommandCodec（组帧 `0xAA`…`0xBB` + CAN-FD 补齐）→ SerialManager.write_data。

反馈：SerialManager 接收线程 → TxFeedbackCodec（定长拼帧、校验、float32 取值、状态解析）
→ 线程安全 McuFeedback 快照 → GUI 刷新 TreeView。

## 两个必须知道的实现约束

1. **Tx 帧长靠“曲线数量”推**：新 Tx 帧里既没有长度字段也没有 CRC，帧长 = `7 + 4×曲线数 + 1`。
   上位机默认按 10 条（2 电机 × state/θ/ω/a/τ）解帧，与 `router_tran_handle.c`
   `DeviceRouter_TranInit` 的低速注册表一致。**改固件曲线注册表，必须同步改**
   `AppConfig.feedback_curve_count`（或 `McuConfig.LOW_SPEED_CURVE_COUNT`）。
   因为没有 CRC，定帧靠“帧头 + ID 合法(0xB0~0xB7) + Func 合法(低速 0/2) + 帧尾就位 +
   浮点有限”这一组条件，任一不满足就丢 1 字节重新找帧头（CAN-FD 的补零字节也这样跳过）。
2. **本工程只用低速策略**（`app_config.h: ROUTER_TX_LOW_SPEED`）：1kHz 采样即发，
   Tx Func 只会是 `USB_USART_NORMAL(0)` / `FDCAN_NORMAL(2)`，解码器只接受这两个。
   新帧不带时间戳，`hw_time = 帧计数器 / 1kHz`（帧计数器每采样点 +1）。
   若改回高速策略，放开 `constants.TX_ACCEPTED_FUNCS` 并改采样频率即可。

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
- `RxCommandCodec` 保留了组帧前的护栏（`constants.TAIL_CONFLICT_MODES`）：**日后改帧尾值
  或加新 mode 时若又踩坑，会直接抛错**而不是无声失效。

`0xAA / 0xBB / 0xCC / 0xDD` 这组还顺带避开了 `0x0A`(`\n`)、`0x0D`(`\r`)、`0x1A`、`0x00`——
链路中途若有按行/文本处理的环节（ASCII 模式 SLCAN、做 CRLF 转换的转发程序、文本日志管道），
这些字节容易被改写或截断，现在四个定界符都不在其中。

## 已核对无需改动的部分

- `MotorState.CODES` 与新固件 `motor_state_machine.h` 的 `MotorStateType` 逐条一致
  （状态码本次未变；`DISPATCH_STATE` 的参数与反馈解析共用这套枚举）。
- MCU 节点号：`router_common.h` 的 `RouterCommObject` 已补齐 `MCU0~MCU7 = 0xB0~0xB7`，
  与 `McuConfig.COUNT = 8` 一致（旧版“只定义到 MCU5”的疑问已消除）。
- Rx Func 固件仍不校验，默认沿用 `RxFunc.FDCAN_CMD`，`config.py` 可一键切 USB_USART。
- 命令帧仍在编码末尾恒定补 0 到下一个 CAN-FD 合法长度（8/12/16/20/24/32/48/64）。

## 运行

```bash
python high_dof_gui.py
```
