# MaixCAM-Pro 接入 AVC1 实车指南

本指南对应 `maixcam_marker_detector` 0.5.0。程序只发送 `vx=0` 和横向 `vy`，不会
控制底盘自转。底盘解算、PID、CAN、电机控制和最终安全仲裁仍由 C 板完成。

## 1. 接线

| MaixCAM-Pro | DJI C板 | 说明 |
|---|---|---|
| A19 / UART1_TX | PG9 / USART6_RX | MaixCAM向C板单向发送 |
| GND | GND | 必须共地 |

- 使用3.3 V TTL、115200、8N1、无流控。
- 不要接5 V串口，也不要接RS-232电平。
- 推荐 `/dev/ttyS1`，不要使用输出启动日志且被系统占用的UART0。
- V1不需要连接 MaixCAM RX 和 C板 PG14。

上电后先在 MaixCAM 检查：

```bash
ls -l /dev/ttyS1
```

若设备不存在，不要改程序路径硬试，先检查系统版本和引脚复用。

## 2. 程序的安全行为

不提供 `--chassis-uart` 时，程序完全不打开串口：

```bash
./maixcam_marker_detector --control-config lateral_control.conf
```

显式启用后：

```bash
./maixcam_marker_detector --control-config lateral_control.conf \
  --chassis-uart /dev/ttyS1 --chassis-vy-limit 0.10
```

- 串口开启后先发送一帧 `valid=0` 停车帧。
- 正常检测或100 ms以内的短时预测发送 `valid=1`。
- 此前已锁定目标但预测超时后，以0.08 m/s进入有界横向扫描；重新识别后退出搜索、
  清除旧积分并重新居中。
- 搜索8秒仍未找回，或启动后从未识别到目标时，发送 `valid=0,vx=0,vy=0`。
- 第一次相机空读立即发送停车帧；若一直空读，C板保持在无效状态。
- 正常退出时再发送停车帧。
- 串口写失败会报错退出；C板超过200 ms收不到合法新帧也会停车。
- `seq` 从0开始，每次发送尝试都递增，包括停车帧，并允许自然回绕。
- 协议逐轴硬限幅为0.8 m/s；程序另有默认0.10 m/s的实车发送限幅。

最终发送状态会写入每帧 JSON：

| 字段 | 含义 |
|---|---|
| `chassis_uart_enabled` | 本次是否启用了串口 |
| `chassis_tx_attempted` | 本帧是否尝试发送 |
| `chassis_tx_success` | 是否完整写出24字节 |
| `chassis_tx_seq` | AVC1序号 |
| `chassis_tx_valid` | 发送给C板的valid |
| `chassis_tx_vx_mps` | 实际发送vx，当前固定为0 |
| `chassis_tx_vy_mps` | 经过独立实车限幅后的vy |

## 3. 第一次联调前参数

先在 `lateral_control.conf` 使用保守参数：

```ini
position_deadband_m=0.002
lateral_kp_per_s=2.50
lateral_ki_per_s2=0.50
max_integral_vy_mps=0.08
relative_velocity_gain=0.00
max_vy_mps=0.10
max_command_acceleration_mps2=0.50
search_enabled=1
search_speed_mps=0.08
search_first_leg_s=1.00
search_max_duration_s=8.00
search_velocity_direction_threshold_mps=0.02
```

命令行仍保留 `--chassis-vy-limit 0.10`，形成第二层限速。不要第一次就使用日志验证
阶段的0.6 m/s上限。

## 4. 第一次实车联调顺序

准备足够空旷的场地并将车轮架空或确保随时可以松开L1：

1. 接线后启动视觉程序，但不要按L1。确认程序显示 UART 已启用且没有写错误。
2. 圆圈键解锁，方块键进入自动模式，仍不按L1，确认底盘不动。
3. 启动后尚未识别过目标时遮住标志，确认日志为 `chassis_tx_valid=false`，底盘停止。
4. 让目标位于画面左侧。持续按住L1，车应以不超过0.10 m/s向左移动。
5. 松开L1并确认立即停车；再用画面右侧目标确认车向右移动。
6. 锁定目标后将它移出画面，确认约100 ms后进入 `SEARCHING`，先沿最后方向移动，
   找回后重新居中；测试期间随时准备松开L1。
7. 保持目标不可见，确认搜索达到8秒后变为 `chassis_tx_valid=false` 并停车。
8. 停止视觉程序，确认退出停车帧或200 ms超时能使底盘停止。
9. 方向、搜索和停车全部正确后，再开始调比例项、积分项和速度前馈。

PS2顺序固定为：圆圈解锁 → 方块进入自动模式 → 持续按住L1允许运动。松开L1
立即停车；三角返回手动，叉键锁定。

## 5. 逐步恢复跟随能力

1. `relative_velocity_gain=0`、较小积分增益，先把比例反馈调到能回中且不过冲。
2. 增加 `lateral_ki_per_s2`，使匀速跟随时中心长期偏差逐渐消失。
3. 若换向后仍沿旧方向运动太久，降低积分增益或 `max_integral_vy_mps`。
4. 从0.1开始增加 `relative_velocity_gain`，改善目标起步和换向响应。
5. 最后按0.10、0.20、0.30 m/s逐级提高发送限速，每一级都重新验证松开L1、
   目标丢失和停止程序三种停车路径。

正常实车运行不要开启 `--debug` 或 `--debug-display`，图像传输和绘制会降低频率。
使用标准模式可维持现有约24 FPS，满足AVC1建议的20–50 Hz发送范围。

## 6. 协议实现

程序按照电控文档的固定24字节小端帧实现：

```text
magic | seq | vx | vy | valid | 3字节零填充 | checksum
```

`magic=0x31435641`，checksum初值为 `0xA5A51234`，对前20字节逐字节执行：

```text
c = (c << 5) XOR (c >> 2) XOR byte
```

独立主机测试文件 `tests/avc1_protocol_test.cpp` 收录了电控文档给出的三组完整
字节级校验向量，用于检查浮点格式、字节序、填充、checksum和序号回绕。
