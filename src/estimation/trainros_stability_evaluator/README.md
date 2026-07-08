# trainros_stability_evaluator

列车运行平稳性评估 ROS2 包。

计划输入：

- `/train_state`

计划输出：

- `/stability`
- 类型：`trainros_interfaces/msg/Stability`
- QoS：Reliable

初始范围：

- RMS 和峰值加速度。
- 稳定数据窗口定义后，再添加 PSD 和 TSI。
- 动态阈值参数。
