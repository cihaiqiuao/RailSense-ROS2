# TrainROS ROS2 命令与无硬件使用说明

本文说明 TrainROS 当前常用 ROS2 命令、每类命令的作用，以及 rosbag2 录制功能为什么重要。

## 1. 构建环境

每次新开 WSL 终端，先加载 ROS2 Humble：

```bash
source /opt/ros/humble/setup.bash
```

如果已经构建过 TrainROS，还需要加载当前工作区：

```bash
cd /tmp/trainros_ws
source install/setup.bash
```

这一步的作用是把本工作区里的包、可执行节点、消息类型和 Action 类型加入当前 shell 环境。没有执行 `source install/setup.bash` 时，`ros2 run`、`ros2 launch`、`ros2 action` 可能找不到 TrainROS 的包和接口。

## 2. 构建命令

建议在 WSL 原生目录 `/tmp/trainros_ws` 构建，而不是直接在 `/mnt/e` 上构建：

```bash
rm -rf /tmp/trainros_ws
mkdir -p /tmp/trainros_ws/src
cp -a /mnt/e/ros2/TrainROS/src/. /tmp/trainros_ws/src/

cd /tmp/trainros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

命令作用：

- `rm -rf /tmp/trainros_ws`：清理旧的临时构建工作区，避免缓存影响。
- `mkdir -p /tmp/trainros_ws/src`：创建标准 ROS2 工作区目录。
- `cp -a ...`：把项目中的 ROS2 包复制到 Linux 原生文件系统。
- `colcon build --symlink-install`：构建所有 ROS2 包，并用符号链接方式安装，方便改代码后快速验证。
- `source install/setup.bash`：加载构建结果。

## 3. 自动化测试命令

运行传感器 parser 单元测试和 fake serial 集成测试：

```bash
cd /tmp/trainros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon test --packages-select trainros_imu_driver trainros_gps_driver trainros_laser_driver trainros_bringup --event-handlers console_direct+
```

作用：

- 验证 IMU/GPS/Laser 协议解析是否正确。
- 自动创建伪串口 PTY。
- 启动 IMU/GPS/Laser driver 和 Fusion。
- 验证 `/imu/data`、`/gps/fix`、`/laser/scan`、`/train_state`、`/diagnostics` 是否有有效数据。

## 4. 无硬件启动流程

### 4.1 启动伪串口数据源

终端 1：

```bash
python3 /mnt/e/ros2/TrainROS/tools/fake_serial_replay.py
```

该脚本会输出三路伪串口，例如：

```text
imu_port: /dev/pts/3
gps_port: /dev/pts/4
laser_port: /dev/pts/5
```

作用：

- 模拟 IMU 33 字节帧。
- 模拟 GPS RMC 语句。
- 模拟 Laser 12 字节距离帧。
- 在没有真实硬件时验证传感器主链路。

### 4.2 启动 TrainROS fake serial 模式

终端 2：

```bash
cd /tmp/trainros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch trainros_bringup trainros_fake_serial.launch.py \
  imu_port:=/dev/pts/3 \
  gps_port:=/dev/pts/4 \
  laser_port:=/dev/pts/5
```

把 `/dev/pts/3`、`/dev/pts/4`、`/dev/pts/5` 换成你终端 1 实际打印出来的端口。

作用：

- 启动 `trainros_imu_driver`。
- 启动 `trainros_gps_driver`。
- 启动 `trainros_laser_driver`。
- 启动 `trainros_fusion`。
- 启动 `trainros_stability_evaluator`。
- 使用 fake serial 参数覆盖真实硬件串口参数。

## 5. Topic 查看命令

查看当前系统有哪些 Topic：

```bash
ros2 topic list
```

查看某个 Topic 的消息类型：

```bash
ros2 topic type /train_state
```

查看一次 IMU 数据：

```bash
ros2 topic echo --once /imu/data
```

查看一次 GPS 数据：

```bash
ros2 topic echo --once /gps/fix
```

查看一次 Laser 数据：

```bash
ros2 topic echo --once /laser/scan
```

查看一次融合结果：

```bash
ros2 topic echo --once /train_state
```

查看一次平稳性评估：

```bash
ros2 topic echo --once /stability
```

查看一次系统诊断：

```bash
ros2 topic echo --once /diagnostics
```

查看发布频率：

```bash
ros2 topic hz /train_state
```

作用：

- `ros2 topic list` 用来确认节点是否成功创建 Topic。
- `ros2 topic echo` 用来直接看消息内容。
- `ros2 topic hz` 用来检查发布频率是否符合预期。

## 6. Node 查看命令

查看当前有哪些节点：

```bash
ros2 node list
```

查看某个节点的订阅、发布、服务和参数：

```bash
ros2 node info /trainros_fusion
```

作用：

- 检查节点是否启动。
- 检查节点是否发布了预期 Topic。
- 检查节点是否订阅了正确 Topic。

## 7. 参数查看命令

列出某个节点的参数：

```bash
ros2 param list /trainros_fusion
```

查看某个参数：

```bash
ros2 param get /trainros_fusion output_rate_hz
```

作用：

- 检查 launch 和 YAML 参数是否生效。
- 例如 Fusion 的 `output_rate_hz`、`sync_queue_size`、`sync_slop_ms`。

## 8. 本地视频发布到 `/camera/image_raw`

TrainROS 提供了视频文件发布节点，可以把本地 MP4 解码成 ROS2 图像 Topic：

```bash
cd /tmp/trainros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run trainros_camera_driver video_file_publisher --ros-args \
  -p video_path:=/mnt/d/chegou/视频/WeChat_20250616101426.mp4
```

该节点发布：

```text
/camera/image_raw
类型：sensor_msgs/msg/Image
编码：bgr8
```

依赖：

```bash
sudo apt-get update
sudo apt-get install -y ffmpeg
```

如果没有安装 `ffmpeg/ffprobe`，节点会报错提示，但不会影响其它节点构建。

也可以用 launch 同时启动视频发布和 YOLO：

```bash
ros2 launch trainros_bringup trainros_video_yolo.launch.py \
  video_path:=/mnt/d/chegou/视频/WeChat_20250616101426.mp4
```

查看视频帧是否发布：

```bash
ros2 topic echo --once /camera/image_raw
ros2 topic hz /camera/image_raw
```

查看 YOLO 检测输出：

```bash
ros2 topic echo /coupler_detection
```

## 9. Logger 使用命令

启动 Logger：

```bash
cd /tmp/trainros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run trainros_logger logger_node --ros-args -p log_dir:=/tmp/trainros_logs
```

查看业务日志：

```bash
tail -f /tmp/trainros_logs/trainros_business.jsonl
```

Logger 当前会记录：

- 串口打开：`serial_opened`
- 串口关闭：`serial_closed`
- 串口重连：`serial_reconnect`
- parser 丢帧：`parser_drop_frame`
- GPS 有定位：`gps_fix`
- GPS 无定位：`gps_no_fix`
- 诊断状态变化：`diagnostic_state_changed`
- 平稳性 warning：`stability_warning`
- 平稳性 alarm：`stability_alarm`

## 10. Recorder 和 Action 命令

启动 Recorder：

```bash
cd /tmp/trainros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run trainros_recorder recorder_node
```

查看 Action 列表：

```bash
ros2 action list
```

查看 Action 类型：

```bash
ros2 action type /record
```

发送录制请求：

```bash
ros2 action send_goal /record trainros_interfaces/action/Record \
  "{record: true, output_uri: '/tmp/trainros_bags/test'}" \
  --feedback
```

作用：

- `/record` 是 TrainROS 的录制控制 Action。
- `record: true` 表示开始录制。
- `output_uri` 是 bag 输出目录。
- `--feedback` 会显示当前录制时长。

Recorder 内部会启动：

```bash
ros2 bag record -o /tmp/trainros_bags/test <record_topics...>
```

默认录制 topic 来自：

```text
src/trainros_bringup/params/logging.yaml
```

当前默认包括：

- `/imu/data`
- `/gps/fix`
- `/laser/scan`
- `/camera/image_raw`
- `/coupler_detection`
- `/train_state`
- `/stability`
- `/system_status`
- `/diagnostics`
- `/log_status`

## 11. rosbag2 查看和回放命令

查看录制文件：

```bash
ls /tmp/trainros_bags/test
```

正常会看到：

```text
metadata.yaml
test_0.db3
```

查看 bag 信息：

```bash
ros2 bag info /tmp/trainros_bags/test
```

回放 bag：

```bash
ros2 bag play /tmp/trainros_bags/test
```

回放时可以另开终端查看数据：

```bash
ros2 topic echo /train_state
```

## 12. 录制功能有什么作用

rosbag2 录制的作用是把运行时 Topic 数据保存下来，形成可回放的数据包。

对 TrainROS 来说，录制功能主要有五个作用：

1. **无硬件复现问题**

   真实车载环境里出现过的 IMU 抖动、GPS no-fix、激光异常、融合错误，可以先录成 bag。回到开发环境后，不需要再连接硬件，也能用 `ros2 bag play` 复现当时的数据流。

2. **离线算法验证**

   Fusion、Stability、YOLO 等算法改完后，可以反复播放同一个 bag，比较改动前后的 `/train_state`、`/stability` 输出是否变好。

3. **调试多节点通信**

   bag 里保存了多个 Topic 的时间戳和数据内容，可以检查 IMU/GPS/Laser 是否频率正常、是否丢数据、是否和 Fusion 同步窗口匹配。

4. **毕业设计和演示**

   没有硬件或现场环境不稳定时，可以直接播放 bag，展示系统从传感器数据到融合、评估、日志的完整链路。

5. **问题留证**

   当系统出现 alarm、GPS no-fix、串口重连等事件时，业务日志记录“发生了什么”，rosbag2 记录“当时的数据是什么”。两者结合起来，后续定位问题更可靠。

一句话总结：

```text
Logger 负责记录事件，rosbag2 负责记录数据。
事件告诉你什么时候出了问题，bag 告诉你当时每个 Topic 的原始数据是什么。
```

## 13. 延时观测与低延时优化命令

查看融合结果发布频率：

```bash
ros2 topic hz /train_state
```

查看传感器端到端延时：

```bash
ros2 topic delay /imu/data
ros2 topic delay /laser/scan
ros2 topic delay /train_state
```

查看系统内部统计出的延时指标：

```bash
ros2 topic echo /diagnostics
```

重点关注 `trainros_monitor` 里的这些字段：

- `imu_latency_ms`：IMU 消息从发布时间到 Monitor 收到的延时。
- `gps_latency_ms`：GPS 消息从发布时间到 Monitor 收到的延时。
- `laser_latency_ms`：Laser 消息从发布时间到 Monitor 收到的延时。
- `train_state_latency_ms`：融合结果从发布时间到 Monitor 收到的延时。

也可以关注 `trainros_fusion` 里的这些字段：

- `imu_laser_sync_count`：IMU 和 Laser 成功同步的次数。
- `gps_age_ms`：最近一次有效 GPS 距离当前的时间。
- `laser_age_ms`：最近一次 Laser 距离当前的时间。
- `coupler_age_ms`：最近一次车钩检测结果距离当前的时间。
- `state_latency_ms`：融合状态使用的最近传感器时间戳到当前发布时刻的延时。
- `dropped_due_to_stale_sensor`：因为 IMU 或 Laser 数据过旧而被统计为 stale 的次数。
- `kalman_enabled`：是否启用轻量 Kalman Filter。
- `kalman_position`：Kalman 当前估计的距离/位置状态。
- `kalman_speed`：Kalman 当前估计的速度。
- `kalman_acceleration`：Kalman 当前估计的加速度。
- `raw_laser_distance`：Laser 原始测距值，单位为米。
- `kalman_imu_update_count`：IMU 加速度观测更新次数。
- `kalman_gps_update_count`：GPS 速度观测校正次数。
- `kalman_laser_update_count`：Laser 距离观测校正次数。

## 14. 为什么 GPS 和 YOLO 不参与主链路强同步

当前 Fusion 的主链路只对 `/imu/data` 和 `/laser/scan` 做 `ApproximateTime` 同步。原因是：

- IMU 频率高，负责姿态和加速度，是状态更新主来源。
- Laser 频率中等，和 IMU 一起形成高频状态更新。
- GPS 通常只有 10Hz，如果强制参与每次同步，会把融合更新节奏拖慢。
- YOLO 推理延时不稳定，如果参与强同步，会让视觉卡顿影响 `/train_state`。

因此 GPS 作为低频校正源，YOLO 车钩状态作为最近值缓存。这样 `/train_state` 可以稳定按 50Hz 发布，同时仍然保留 GPS 速度估计和车钩状态。

Fusion 内部还维护一个轻量三维 Kalman Filter，状态量为 `position`、`speed` 和 `acceleration`。IMU 加速度用于高频预测和加速度观测更新，GPS 位置差分得到的速度用于低频校正，Laser 距离作为 `position` 观测校正距离状态。这样可以减少单次 GPS 抖动对 `/train_state.speed` 的影响，同时让 `/train_state.laser_distance` 输出经过滤波后的距离；原始激光值仍可通过 `/laser/scan` 或 `/diagnostics` 中的 `raw_laser_distance` 查看。

## 15. rosbag 默认不录制原始相机图像

默认 `logging.yaml` 中 `record_camera: false`，并且 `record_topics` 不包含 `/camera/image_raw`。这样做是为了避免原始图像占用大量磁盘带宽，影响高频传感器链路。

默认录制重点保留：

- `/imu/data`
- `/gps/fix`
- `/laser/scan`
- `/coupler_detection`
- `/train_state`
- `/stability`
- `/system_status`
- `/diagnostics`
- `/log_status`

如果需要录制相机原始图像，可以启动 Recorder 时打开参数：

```bash
ros2 run trainros_recorder recorder_node --ros-args -p record_camera:=true
```

或者在 YAML 的 `record_topics` 中手动加入：

```yaml
- /camera/image_raw
```

判断延时是否变大的基本方法：

- `/train_state` 频率明显低于 50Hz，说明融合发布链路可能被阻塞。
- `imu_latency_ms` 或 `laser_latency_ms` 持续升高，说明传感器数据在 DDS 或回调队列里积压。
- `gps_age_ms` 持续超过 `gps_timeout_ms`，说明 GPS 数据缺失或频率异常。
- 录制 rosbag 后系统明显变慢，优先检查是否录制了 `/camera/image_raw`。
