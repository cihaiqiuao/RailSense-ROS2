# 2026-07-08 测试体系补强决策

## 背景

项目已有 parser gtest、Stability gtest 和 fake serial 集成测试，但还缺少工程质量类测试，无法覆盖 Sanitizer、Fusion 核心算法、Logger 业务事件、Recorder Action、launch/参数回归和视觉降级路径。

## 决策

- 新增 `ENABLE_SANITIZER` CMake 选项，Debug 构建时给 C++ 节点和 gtest 目标启用 `-fsanitize=address,undefined -fno-omit-frame-pointer`。
- 将 Fusion 三维 Kalman 逻辑拆到 `trainros_fusion/motion_kalman_filter.hpp`，节点继续保持原 Topic 和消息接口不变。
- 新增 Fusion gtest，覆盖 IMU 加速度更新、GPS 速度校正、Laser 距离校正、GPS 缺失预测和非法预测步长。
- 新增 Logger pytest，验证 diagnostics/stability 事件能写入 JSONL，且重复状态不会重复刷日志。
- 新增 Recorder pytest，验证 `/record` Action 可启动、反馈 duration，并可取消 rosbag2 子进程。
- 新增 launch/参数 pytest，检查 `trainros_system.launch.py`、`trainros_fake_serial.launch.py` 和关键 YAML 参数。
- 新增 Monitor pytest，验证延时 diagnostics 字段和 Topic 超时 WARN。
- 新增 Camera/YOLO 降级 pytest，验证视频或模型缺失时节点不崩溃。

## 验证

- `/tmp/trainros_ws` 普通构建：12 个包通过。
- `/tmp/trainros_ws` 普通测试：41 tests，0 errors，0 failures，0 skipped。
- `/tmp/trainros_publish_asan_ws` Sanitizer Debug 构建：12 个包通过。
- `/tmp/trainros_publish_asan_ws` Sanitizer 测试：41 tests，0 errors，0 failures，0 skipped。

## 注意

- Sanitizer 测试使用 `ASAN_OPTIONS=detect_leaks=0`，避免 ROS2/DDS 第三方库退出阶段 leak 噪声。
- 暂不启用 TSan，避免 DDS 内部线程噪声影响日常验证。
