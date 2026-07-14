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

## 已对照 router.h / router.c 校正（原三处“待确认”现已落定）

1. **ControlMode 数值**：`constants.py` 的 `_CONTROL_MODE_TABLE` 逐字节抄自 router.h，
   经 `mode_for(功能, 电机)` 查表。旧版臆测的 `THETA_GEAR=6` 等全部作废。
2. **双电机区分**：确认靠 mode 的 M0/M1 后缀区分，非前/后半段。编码器已改为扁平命令流，
   `command_num` 奇偶皆可（固件两段循环并集覆盖全部命令）。
3. **缩放系数**：取自 router.c handler 的除数——速度 ×1000、力矩 ×1、阻抗三项 ×100
   （旧版分别为 100 / 100 / 10·100·1000，均已改正）。力矩通道实际下发到电流环。
4. **Func**：仍默认 `RX_FDCAN_COMMAND`（固件未校验），`config.py` 可一键切 USB_USART。

## 固件功能集差异（重要）

本固件 handler 仅实现：位置 / 速度 / 力矩(电流) / 阻抗(弹簧·阻尼·惯量) / 换ID / 状态派发。
GUI 里的 **Iq、各环 PID、轨迹、回零、刷参、系统辨识、清Flash、弹簧原点** 在该固件中无对应
case。这些指令现由 `SerialCommander` 显式抛 `UnsupportedCommand`（不再乱发错误 mode），
点击对应按钮会在日志里提示“当前固件未实现”。`FIRMWARE_SUPPORTED` 列出了已支持项。

## 仍需你确认的事项

- **MCU 数量**：router.h 只定义到 `MCU5(0xB5)`，但 router.c 告警文案写“0xB0~0xB7”。现暂按
  8 个（同旧 GUI）。若实际只有 6 个，改 `McuConfig.COUNT=6` 即可，其余层不动。
- **MODE_SELECT 状态码**：状态派发命令的参数是否与反馈解析用的同一套状态枚举
  （`StartupReady=2` 等）？两者共用了 `MOTOR_STATE`，待与固件 `ServiceStateCmd` 核对。
- **未实现功能的去留**：上面那批 GUI 功能是要（a）删掉界面、（b）保留但灰化、还是
  （c）你另有固件 handler 可补 mode——请示下，我据此收尾 GUI。

## 运行

```bash
python main.py
python tests_selfcheck.py   # 跑编解码自测
```
