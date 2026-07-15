# High_15Dof_Hand-work（重构版）

按“协议事实来源（router.c）优先、逐层解耦”的原则重构。分层职责如下。

## 目录结构

```
High_15Dof_Hand-work/
├─ main.py                       # 入口
├─ high_dof_gui.py               # 仅界面 + 读表单 + 调 controller + 刷新
├─ tests_selfcheck.py            # 编解码 / 协议一致性自测（无需真实串口）
└─ sdk/
   ├─ config.py                  # 串口默认参数、Func、刷新率、阈值
   ├─ models.py                  # dataclass：命令、反馈、统计、结果、目标枚举
   ├─ serial_manager.py          # 串口 IO + 接收线程 + 线程安全快照
   ├─ serial_commander.py        # 控制语义 → 量化 → 组命令 → 编码 → 发送
   ├─ hand_controller.py         # 选哪些 MCU / 批量发送 / 汇总结果
   └─ protocol/
      ├─ constants.py            # 唯一协议源：帧头尾、Func、MCU、模式、缩放、状态机
      ├─ rx_command_codec.py     # 上位机→MCU 命令帧编码
      └─ tx_feedback_codec.py    # MCU→上位机 反馈帧拼帧/解析
```

## 数据流

控制：GUI → HandController（选 MCU/电机、批量）→ SerialCommander（量化、组命令）
→ RxCommandCodec（组帧 0x3B…0x1E）→ SerialManager.write_data。

反馈：SerialManager 接收线程 → TxFeedbackCodec（拼帧、CRC、反量化、状态解析）
→ 线程安全 McuFeedback 快照 → GUI 刷新 TreeView。

## 本次修掉的问题

- 协议常量集中到 `protocol/constants.py`，GUI/Commander/Manager 不再各自硬编码。
- `SerialManager` 加 `_data_lock`/`_write_lock`，GUI 用 `get_snapshot()` 取一致快照。
- 修复接收线程异常时 `disconnect()` 里 join 自己的问题。
- 命令帧计数器改为每帧递增（旧代码恒为 0，MCU 无法查丢包）。
- 编码器改为扁平命令流：电机由 mode 的 M0/M1 后缀区分，不再靠前/后半段。
- 业务逻辑（`_run`/`_do_*`）从 GUI 移入 `HandController`。

## 已对照 router.h / router.c 校正

1. **ControlMode 数值**：`MotorFunc` 的每个成员逐值对应 router.h `RouterControlMode`
   枚举（位置 `THETA_GEAR=6`、速度 `OMEGA_GEAR=7`、力矩 `TORQUE_GEAR=8`、阻抗 `32~35`、
   PID `52~57`、状态派发 `72`、轨迹 `82~86`、回零/刷参 `102~103`、清Flash `122`）。
2. **双电机区分**：确认靠命令在帧里的**前/后半段位置**区分——前半段→电机0(M1)、
   后半段→电机1(M2)，两段 switch 共用同一套 mode 值。编码器按此路由，两半用 `NOP_MODE`
   补齐到等长，使固件切分点 `command_num//2` 恰好落在中间。
3. **缩放系数**：取自 router.c handler 换算，物理量按**输出轴**单位（齿轮比 GEAR 抵消）——
   位置/速度/力矩/轨迹 ×100、阻抗弹簧 ×10、阻尼 ×100、惯量 ×1000、Iq/Id 与电流/速度 PID
   ×1000、位置 PID ×1、状态码 ×1。
4. **Func**：默认 `RX_FDCAN_COMMAND`（固件未校验），`config.py` 可一键切 USB_USART。
5. **CAN-FD 帧长补齐**：本项目固定走 FDCAN 发送，命令帧在编码末尾**恒定**补 0 到下一个
   CAN-FD 合法长度（8/12/16/20/24/32/48/64）。因是固定行为，已去掉旧的 `pad_canfd` 开关
   （原 `AppConfig.pad_canfd` 字段、`resolved_pad_canfd()` 及各层传参），补齐逻辑内聚在
   `RxCommandCodec._canfd_pad`。

## 固件功能集

本固件 handler 均已实现：位置 / 速度 / 力矩 · Iq/Id · 阻抗(弹簧·阻尼·惯量·原点) ·
各环 PID · 状态派发 · 轨迹(init/deinit/vmax/amax/pos) · 回零 · 刷参 · 清Flash。
`SerialCommander` 对上述全部提供发送方法（旧版误判为"未实现"的那批现已接通）。
注意：旧版曾有的"换 ID (Switch_ID)"在本固件中**无对应 case**，已从 SDK 移除。

## 仍需你确认的事项

- **MCU 数量**：router.h 只定义到 `MCU5(0xB5)`，但 router.c 告警文案写"0xB0~0xB7"。现暂按
  8 个（同旧 GUI）。若实际只有 6 个，改 `McuConfig.COUNT=6` 即可，其余层不动。

（`MotorState.CODES` 已与固件 `MotorStateType` 逐条核对一致；`DISPATCH_STATE`(mode 72)
的参数与反馈解析共用这套枚举。）

## 运行

```bash
python main.py
python tests_selfcheck.py   # 跑编解码自测
```
