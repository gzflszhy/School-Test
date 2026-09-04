# MaixCAM 横向跟随调参与数据采集手册

本文对应 `maixcam_marker_detector` 的日志级横向控制器。当前版本只计算并显示
`vy`，不会打开 UART，也不会让底盘运动。

## 1. 坐标和符号

- 图像 `x` 向右增加。
- AVC1 `vy > 0` 表示底盘向左，`vy < 0` 表示底盘向右。
- 因此目标在画面左侧时，`lateral_error_m > 0` 且 `vy > 0`；目标在画面右侧时
  两者都应为负。
- 当前不计算 `vx`，不控制旋转。

三大 L 的直角边 CAD 长度均为 50 mm。程序用两条边的像素长度均值 `s` 计算：

```text
distance_m = focal_x_px * 0.050 / s
lateral_error_m = -(refined_center_x_px - principal_x_px)
                  * distance_m / focal_x_px
```

当距离也由同一个 `focal_x_px` 推导时，横向误差中的焦距会代数抵消。因此标称焦距
误差主要影响日志中的 `distance_m`；横向跟随更依赖 50 mm 实物尺寸、中心点和光心
`principal_x_px`。以后接入独立标定距离时，应继续使用标定后的 `focal_x_px`。

## 2. 运行方式

使用内置默认参数：

```bash
./maixcam_marker_detector --debug-display 2>&1 | tee lateral.log
```

使用外部参数文件：

```bash
./maixcam_marker_detector --debug-display \
  --control-config lateral_control.conf 2>&1 | tee lateral.log
```

配置文件每行一个 `key=value`，支持 `#` 注释。未知字段、非法数值或缺少文件都会让
程序明确报错退出，不会静默采用一半新参数、一半旧参数。

## 3. 调试画面

画面底部横线表示 `vy` 全量程：

- 箭头向左：`vy > 0`，要求底盘向左。
- 箭头向右：`vy < 0`，要求底盘向右。
- 箭头越长，速度绝对值越大。
- 圆圈和 `HOLD`：命令处于零速死区。
- 红色 `STOP`：当前控制输出无效。
- 绿色十字：仅由三个大 L 得到的几何中心。
- 青色菱形：可选小部件一致性检查通过后的修正中心。

青色竖线是目标中心，左右两条暗黄色竖线是按当前距离换算出的中心死区。状态行还
显示滤波横向误差 `x`、标称距离 `z`，以及位置比例 `P`、积分 `I`、相对速度前馈
`V` 三项。`SAT` 表示速度限幅，`SLEW` 表示加速度限幅。

## 4. JSON字段

每帧重点记录以下字段：

| 字段 | 含义 |
|---|---|
| `control_valid` | 本帧是否允许使用 `vy` |
| `control_source` | `MEASURED`、`PREDICTED` 或 `INVALID` |
| `vy_mps` | 限速、限加速度后的最终横移速度 |
| `vy_unconstrained_mps` | 限制前的控制器输出 |
| `marker_scale_px` | 两条50 mm直角边的平均像素长度 |
| `distance_m` | 使用标称镜头参数得到的距离 |
| `lateral_error_raw_m` | 当前帧原始横向误差，左正右负 |
| `lateral_error_filtered_m` | α-β滤波后的横向误差 |
| `relative_lateral_velocity_mps` | 目标相对相机的横向速度估计 |
| `vy_position_mps` | 中心位置比例项 P |
| `vy_integral_mps` | 为消除运动中稳态偏差而保留的积分项 I |
| `vy_velocity_mps` | 相对速度前馈项 V |
| `command_saturated` | 限制前命令是否超过速度上限 |
| `acceleration_limited` | 本帧命令是否受到变化率限制 |
| `geometric_center_x_px` | 三大 L 斜边中点 |
| `refined_center_x_px` | 小部件修正后的中心 |
| `optional_refinement_used` | 本帧是否实际应用小部件修正 |
| `optional_refinement_components` | 参与修正的一致小部件数量 |
| `optional_correction_x_px` | 小部件最终施加的水平修正量 |

## 5. 参数分组

### 镜头和几何

- `focal_x_px`：默认 281.5 px，是 GC4653、81°水平视场、480像素宽的理论值。
  距离标定后替换。
- `principal_x_px`：水平光心，默认240。应优先通过多距离居中静态数据校正。
- `marker_leg_m`：两大 L 相邻中心的实物距离，当前 CAD 为0.050 m，不应拿它吸收
  镜头误差。
- `min_marker_scale_px`：低于此尺度不输出控制，防止极远小目标产生巨大噪声。
- `min_distance_m`、`max_distance_m`：合理距离门限，门外数据作废。

### 小部件修正

- `optional_refinement_enabled`：设为0可完整关闭修正，便于A/B对比。
- `optional_refinement_min_components`：默认2。不要在比赛版降为1，单个误匹配缺乏
  一致性验证。
- `optional_residual_gate_scale`：单个小部件相对 CAD 预测位置的最大偏差，按大 L
  尺度归一化。
- `optional_consensus_gate_scale`：多个小部件残差彼此一致的范围。
- `optional_max_correction_scale`：修正量硬上限，防止小部件拖走稳定的大 L 中心。
- `optional_correction_weight`：残差实际施加比例。默认0.35，大 L 始终是主测量。

只有至少两个小部件同时通过“位置合理＋残差一致”才会修正。识别到但不一致的小部件
仍保留在检测日志中，但不影响控制中心。

### α-β滤波器

滤波器维护横向位置和相对速度两个状态：

```text
x_predict = x + v * dt
innovation = measurement - x_predict
x = x_predict + alpha * innovation
v = v + beta / dt * innovation
```

- `filter_alpha` 越大，位置响应越快、噪声越多。
- `filter_beta` 越大，速度跟随越快，但静止时更容易产生虚假速度。
- `innovation_gate_base_m` 与 `innovation_gate_distance_scale` 控制异常测量门限。
- `max_relative_velocity_mps` 限制明显不合理的速度估计。
- `max_prediction_age_s` 默认0.1秒。短暂丢帧使用匀速预测，超过后立即
  `control_valid=false` 和 `vy=0`。
- `filter_reset_gap_s` 是必须重新初始化滤波器的时间间隔。

不要同时修改 `alpha` 和 `beta`。先调位置，再调速度。

### 控制器

```text
P = lateral_kp_per_s * deadbanded_position
I = clamp(I + lateral_ki_per_s2 * deadbanded_position * dt)
V = relative_velocity_gain * relative_velocity
vy_raw = P + I + V
```

- `position_deadband_m`：中心附近不因微小位置误差移动，默认2 mm。现有20 cm静态
  数据的滤波噪声约0.04 mm，因此不再使用过宽的5 mm死区。
- `lateral_kp_per_s`：位置误差反馈。太小会跟随滞后，太大会左右振荡。
- `lateral_ki_per_s2`：位置误差积分增益。它会保留维持目标匀速跟随所需的命令，
  使相对速度回到0后小车仍能运动。太小会长期落后，太大会换向拖尾或振荡。
- `max_integral_vy_mps`：积分项自身上限。控制器在总输出饱和且积分继续推向同一
  方向时暂停积分；反向误差仍可释放积分。
- `relative_velocity_gain`：目标运动前馈/阻尼。太小追不上0.5 m/s目标，太大会放大
  速度估计噪声。
- `max_vy_mps`：视觉命令限速。协议硬上限是0.8 m/s，第一次上车建议改成0.1。
- `max_command_acceleration_mps2`：MaixCAM侧的命令变化率限制。C板仍保留自己的
  加速度安全限制。

## 6. 推荐调参顺序

每次只修改一组参数，并保存参数文件、日志、距离和场景说明。

### 阶段A：静止相机、静止目标

在20、30、50、70、100 cm各录制至少20秒：

1. 目标物理居中，校正 `principal_x_px`，使平均 `lateral_error_raw_m` 接近0。
2. 检查 `marker_scale_px` 标准差和 `distance_m` 单调性。
3. 对比关闭/开启小部件修正。只有修正后标准差更小且均值不漂移才保留。
4. 调整 `filter_alpha`，使静态滤波误差稳定但不出现明显长延迟。
5. 缓慢增加 `filter_beta`，直到静止 `relative_lateral_velocity_mps` 仍接近0。

静态建议验收条件：无误识别跳点；滤波横向标准差不超过3 mm；`vy` 长时间保持0；
`control_valid` 不无故失效。

### 阶段B：相机静止、人工水平移动目标

分别向左、向右移动，先慢速，再接近0.5 m/s：

1. 目标向左时必须输出正 `vy`，向右时必须输出负 `vy`。
2. 检查滤波误差相对原始误差的延迟。
3. 逐步增加 `filter_beta` 改善速度估计。
4. 此阶段先令 `lateral_ki_per_s2=0`，调 `relative_velocity_gain`，观察速度变化能否
   提前反映在 `vy` 中。
5. 恢复积分后，固定相机进行的长时间偏置会让 `I` 持续增长，这是没有车辆闭环时
   的预期结果，不应误判成检测漂移。

这一步没有车辆闭环，不能据此确定最终 `kp`，只能验证符号、滤波和前馈趋势。

### 阶段C：第一次上车

1. 将 `max_vy_mps=0.10`、`max_integral_vy_mps=0.08`，保持较低比例和积分增益。
2. 按电控快速开始逐方向确认，随时松开L1停车。
3. 先把 `lateral_ki_per_s2=0` 和 `relative_velocity_gain=0`，只调位置比例项到不过冲。
4. 从较小值增加积分增益，使匀速目标跟随时的中心偏差逐渐消失；若换向后拖尾，
   先减小积分增益或积分上限。
5. 再从0.1开始增加速度项，直到起步和换向滞后改善且不振荡。
6. 最后逐步提高 `max_vy_mps`，不得一步调到0.6或0.8。

### 判断是否真正居中

只看相机固定、目标移动的日志无法验证最终闭环，因为日志中的命令没有真正改变
相机位置。上车后应同时满足：

1. 目标匀速运动时，`lateral_error_filtered_m` 的长期均值接近0，而不是靠持续偏在
   一侧产生速度命令。
2. 匀速阶段相对速度接近0时，`vy_integral_mps` 应保留主要跟随速度。
3. 目标停止或反向后，积分能被反向误差释放，且中心点不持续穿越死区来回振荡。
4. 正常稳态不应长期出现 `command_saturated=true`；否则车辆速度能力或参数不够。

### 阶段D：比赛场地

分别记录强光、暗处、LED PWM条带、最近/最远距离和0.5 m/s运动数据。只有检测、
距离、滤波与底盘响应都稳定后，才冻结比赛参数。

## 7. 仍需采集的数据

进入最终闭环前需要：

1. 多个已知距离下的 `marker_scale_px`，用于距离标定。
2. 目标位于画面中心及左右已知物理偏移时的数据，用于校正光心和横向尺度。
3. 人工水平移动目标的日志，最好有大致移动距离和用时。
4. 实车固定 `vy` 阶跃（例如±0.05、±0.10、±0.20 m/s）的底盘响应。
5. MaixCAM相对底盘中心的水平安装偏移；最终控制时可作为固定零点补偿。

## 8. 当前默认值的定位

默认参数用于宿舍日志验证，不是比赛最终参数。尤其是 `max_vy_mps=0.60` 只是让
离线箭头和日志保留动态范围；接入UART前必须按阶段C先降到0.10 m/s。
