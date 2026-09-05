# 赛场光照诊断与固定参数流程

本流程用于解决前景、背景光照变化导致亮区掩码和识别结果波动的问题。第一轮先保留
自动曝光和自适应阈值，完整记录它们如何变化；分析日志后再把曝光和阈值冻结为赛场
常量。采集阶段不要连接底盘UART。

## 1. 自适应阈值的组成

当前亮区阈值为：

```text
adaptive_raw = max(Otsu, P92 - 1, ROI_mean + 24)
adaptive_final = clamp(adaptive_raw, 105, 250)
```

`P92`、上下限和24灰度级局部对比度均可在 `detector_tuning.conf` 修改。固定模式下，
ROI候选提取和归一化模板检查会共同使用 `fixed_led_threshold`，不再根据当前帧改变。

## 2. 采集命令

确认配置文件保持：

```ini
use_auto_exposure=1
use_fixed_led_threshold=0
```

在赛场运行：

```bash
./maixcam_marker_detector \
  --detector-config detector_tuning.conf \
  --control-config lateral_control.conf \
  --display 2>&1 | tee field_adaptive.log
```

不要附加 `--chassis-uart`，避免采集光照数据时车辆运动。

至少记录以下场景，每个场景保持10–20秒，并记下大致时间顺序：

1. 标志正对相机、位于画面中心且静止。
2. 标志在画面左、中、右位置缓慢移动。
3. 将标志移出画面，只保留实际赛场背景。
4. 相机朝向现场最亮区域和最暗区域。
5. 模拟比赛距离范围的最近和最远位置。
6. 若场灯存在闪烁，额外保持静止记录至少30秒。

把完整 `field_adaptive.log` 发给我，不要只截取识别成功的行。丢失帧正是选择固定
参数的重要依据。

## 3. 日志字段

只有 `--debug`、`--debug-display` 或 `--display` 下，下列诊断字段才有有效值：

| 字段 | 含义 |
|---|---|
| `threshold_mode` | `ADAPTIVE` 或 `FIXED` |
| `led_threshold` | 本帧实际用于提取亮区的阈值 |
| `adaptive_unclamped_threshold` | 三个候选取最大值、尚未上下限裁剪的值 |
| `otsu_threshold` | Otsu候选 |
| `percentile_threshold` | 配置高分位灰度减1 |
| `mean_based_threshold` | ROI均值加局部对比度 |
| `configured_bright_percentile` | 当前高分位配置，例如 `0.92` |
| `configured_min/max_led_threshold` | 自适应阈值裁剪上下限 |
| `configured_local_contrast_threshold` | ROI均值项所加灰度值 |
| `configured_saturation_threshold` | 饱和比例统计使用的灰度门槛 |
| `configured_morphology_kernel` | 闭运算核尺寸 |
| `configured_fixed_led_threshold` | 固定模式配置值；自适应模式下仅供参考 |
| `roi_x/y/w/h` | 当前统计和检测使用的ROI |
| `roi_gray_min/max/mean/stddev` | ROI灰度范围、均值和标准差 |
| `roi_gray_p50/p90/p95/p99` | ROI灰度分位数 |
| `roi_bright_fraction` | 阈值和形态学处理后的亮区比例 |
| `roi_saturation_fraction` | 灰度不低于 `saturation_threshold` 的比例 |
| `camera_auto_exposure` | 相机是否使用自动曝光 |
| `camera_exposure_us` | GC4653当前曝光时间，微秒 |
| `camera_gain` | GC4653当前增益 |
| `raw_contour_count` | 二值图原始轮廓数量 |
| `led_candidate_count` | 轮廓筛选后保留的亮区候选数量 |
| `found/confidence` | 对应阈值下的识别结果和置信度 |

ROI在 `SEARCH_FULL` 和 `TRACK_ROI` 状态下大小不同，分析时必须按 `state` 和ROI尺寸
分组，不能把两类灰度统计直接混在一起取平均。

## 4. 我会如何确定常量

收到日志后会分别统计成功帧、丢失帧、全图搜索和局部跟踪：

1. 检查曝光时间/增益是否频繁变化，并计算它们与阈值、丢失率的关系。
2. 判断每帧最终阈值主要由 Otsu、P92 还是均值对比度项决定。
3. 找出能压制背景亮区、同时保留三大L的阈值稳定区间。
4. 排除启动预热和明显遮挡，再使用稳定区间的中位数而不是极值作为候选常量。
5. 曝光优先选择能避免LED大面积达到255、又能保持三大L连续的最短可靠值。

仅凭一段自适应日志可以得到第一组建议值，但不能保证一次冻结就覆盖全赛场。固定后
必须使用相同路线再录一份日志做A/B验证。

## 5. 冻结参数

分析完成后修改：

```ini
use_auto_exposure=0
exposure_us=<日志分析得到的曝光时间>
use_fixed_led_threshold=1
fixed_led_threshold=<日志分析得到的灰度阈值>
```

重新运行同一命令。画面右侧应显示 `FIX`，日志的 `threshold_mode` 应为 `FIXED`，
`led_threshold` 必须保持不变；`adaptive_unclamped_threshold` 仍会记录，便于观察环境
漂移，但不会再影响二值图。

验收时比较自适应版与固定版的识别率、最长连续丢失帧、中心标准差、原始轮廓数量、
亮区比例和处理时间。若固定阈值在最亮或最暗区域明显失败，不应强行只使用一个全场
常量，需要优先改善遮光、曝光或LED亮度一致性。
