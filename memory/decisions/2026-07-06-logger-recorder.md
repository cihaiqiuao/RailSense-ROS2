# 2026-07-06 Logger 业务事件与 Recorder 真实录制

## 背景

Logger 之前只写周期心跳 JSONL，Recorder 只有 Action 框架，没有真实调用 rosbag2。当前阶段需要形成“业务事件日志 + 数据录制”的软件闭环。

## 决策

- Logger 不新增自定义日志消息，直接订阅现有 `/diagnostics` 和 `/stability`。
- Logger 将 diagnostics 状态变化写为业务事件，包括：
  - `serial_opened`
  - `serial_closed`
  - `serial_reconnect`
  - `parser_drop_frame`
  - `gps_fix`
  - `gps_no_fix`
  - `diagnostic_state_changed`
- Logger 将 Stability level 变化写为：
  - `stability_normal`
  - `stability_warning`
  - `stability_alarm`
- Logger 继续发布 `/log_status`，但不再把周期心跳写入业务 JSONL。
- Fusion 发布 `/diagnostics`，让 logger 能记录 Fusion 同步健康状态。
- Recorder 使用 `Record.action` 启动 `ros2 bag record -o <output_uri> <record_topics...>`。
- Action cancel 时给 rosbag2 进程组发送 SIGINT，等待 rosbag2 写完缓存并退出。
- `logging.yaml` 新增 `record_topics` 参数。

## 验证

- 在 WSL `/tmp/trainros_ws` 执行 `colcon build --symlink-install`，12 个包构建通过。
- 执行 `colcon test --packages-select trainros_bringup trainros_fusion trainros_logger trainros_recorder --event-handlers console_direct+`，通过。
- Logger 冒烟：发布模拟 diagnostics/stability 后，JSONL 记录 `serial_opened`、`serial_reconnect`、`parser_drop_frame`、`stability_alarm`。
- Recorder 冒烟：Action 启动 `ros2 bag record`，取消后生成 `metadata.yaml` 和 `.db3` 文件。
