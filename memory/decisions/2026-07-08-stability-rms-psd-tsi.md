# 2026-07-08 平稳性评估算法增强

## 背景

原 `trainros_stability_evaluator` 只基于滑动窗口 RMS 和峰值加速度输出 `/stability`，`psd_value` 和 `tsi` 仍为占位值。

## 决策

- 不修改 `Stability.msg`，继续复用 `score`、`rms_acceleration`、`peak_acceleration`、`psd_value`、`tsi`、`warning`、`alarm` 和 `level`。
- 新增工程可用版窗口化评估：
  - RMS：窗口内加速度平方均值开方。
  - Peak：窗口内绝对加速度峰值。
  - PSD：对窗口加速度做简化 DFT，统计 `psd_min_hz` 到 `psd_max_hz` 的频带能量。
  - TSI：由 RMS、Peak、PSD 按权重组合得到的工程指标。
  - Score：由 TSI 映射到 0-100 分。
- 启动初期样本不足 `min_window_samples` 时不直接进入 `alarm`，避免瞬时误报警。
- Stability 节点按 `diagnostics_period_ms` 节流发布 `/diagnostics`，包含窗口样本数、RMS、Peak、PSD、TSI、score 和 level。

## 参数

参数位于 `src/trainros_bringup/params/fusion.yaml` 的 `trainros_stability_evaluator`：

- `sample_rate_hz`
- `min_window_samples`
- `psd_min_hz`
- `psd_max_hz`
- `rms_weight`
- `peak_weight`
- `psd_weight`
- `tsi_alarm_value`
- `diagnostics_period_ms`

## 验证

- 新增 `test_stability_metrics`，覆盖零输入、恒定加速度、脉冲峰值、正弦振动、窗口淘汰和参数更新。
- fake serial 集成测试启动 Stability 节点，验证 `/stability` 和 `trainros_stability_evaluator` diagnostics。
- WSL `/tmp/trainros_ws` 中构建和测试 `trainros_stability_evaluator`、`trainros_bringup` 通过，结果为 `21 tests, 0 errors, 0 failures, 0 skipped`。
