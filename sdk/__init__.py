"""
高自由度机械手上位机 SDK。

分层 (依赖只朝下，不回头)：
    protocol/           协议层：帧格式、曲线布局、错误字、命令码、两个编解码器
    models.py           数据模型：命令/反馈/统计的纯数据结构，不依赖协议层
    serial_manager.py   链路层：串口收发线程、拼帧、按 MCU 存快照与丢包统计
    serial_commander.py 命令层：高层语义 -> 量化 -> 组帧 -> 发送
    hand_controller.py  业务层：对一批 MCU 批量执行命令并收集结果
上层界面 (high_dof_gui.py) 只和 controller / manager / models 打交道。
"""
