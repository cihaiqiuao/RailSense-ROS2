# trainros_interfaces

自定义消息、服务和 Action 包。

计划接口：

- `msg/Coupler.msg`：车钩检测框、关键点、置信度和状态。
- `msg/TrainState.msg`：速度、加速度、横滚、俯仰、航向、激光距离、车钩状态。
- `msg/Stability.msg`：评分、预警、报警和评估指标。
- `msg/SystemStatus.msg`：CPU、内存、I/O、温度、FPS、队列和 DDS 延迟摘要。
- `action/Record.action`：开始/取消录制，并反馈当前录制时长。
