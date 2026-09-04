# MaixCAM-Pro Marker Detector

这是 MaixCAM-Pro（GC4653）上的 C++17 / MaixCDK / OpenCV 端侧 LED Marker 检测与横向跟随程序。生产入口没有 GUI、显示输出或逐帧保存；每处理一帧输出一行 JSONL。默认只计算并记录 `vy`；只有显式提供 `--chassis-uart` 才会按 AVC1 协议向底盘发送命令。

当前主判定只依赖三个 30 mm 大 L：三者分别通过轮廓形状筛选，且其中心组成误差范围内符合 CAD 比例的等腰直角三角形，即可输出 `TRACKABLE`。程序不再搜索或判断黑色板；归一化模板只用于读取三个可选小部件、描述置信度，以及在两个以上小图形命中时输出方向和 `FULL_ID`。

## 运行基线

- 请求 GC4653 `480x270 @ 60 FPS`、`FMT_GRAYSCALE`、`buff_num=1`；底层 ISP
  仍可选择 GC4653 的 `1280x720 @ 60 FPS` 模式并缩放输出；
- 启动后默认丢弃 30 帧，以便自动曝光稳定；手动曝光时，使用配置中的
  `exposure_us`（微秒）后再开始预热；
- 单缓冲模式不调用 `Camera::clear_buff()`；MaixCDK 4.10.3 的 GC4653
  单缓冲通道会报告该操作不受支持；
- 相机返回的灰度缓冲区通过 `cv::Mat` 零拷贝封装，并由 `CameraFrame` 维持其生命周期。检测器必须在该 `CameraFrame` 仍存在时完成 `process()`。

`capture_timestamp_us` 是 `Camera::read()` 成功返回时记录的 host `steady_clock` 单调时间；目前 MaixCDK `Image` API 未提供传感器曝光时间戳，不能把它解释为曝光起点。`output_timestamp_us` 为结果序列化前的同一时钟时间，二者差值是本程序可测的 capture-to-output 时延。

## 固定兼容版本

本项目以 MaixCAM-Pro 系统 `maixcam-pro-2025-03-19-maixpy-v4.10.3`
为当前实机兼容基线。必须使用对应的 MaixCDK commit：

```text
0beaac9d3dd06248bad5259935f5110e261dd447
```

不要直接使用 MaixCDK `main` 分支编译后部署到该固件，否则可能出现
`libmaixcam_lib.so` 符号不匹配。

## MaixCDK 构建与部署

将本目录作为 MaixCDK 项目目录导入/打开，并使用 MaixCDK（或 MaixVision）的 Release 构建流程。`main/CMakeLists.txt` 是官方 component 格式：它声明 `vision` 和 `peripheral` 依赖，分别提供相机/OpenCV集成和官方 UART API。

项目只请求官方 API 已公开的调用：

```cpp
maix::camera::Camera camera(480, 270, maix::image::FMT_GRAYSCALE,
                            nullptr, 60.0, 1, true, false);
camera.exp_mode(0); // MaixCDK 4.10.3: 0=auto, 1=manual
camera.skip_frames(30);
maix::image::Image *image = camera.read(nullptr, 0, true, -1);
// use image->data() while image is alive, then delete image
```

不要使用 `cv::VideoCapture`。不要加 `display` 依赖。实机如发现 60 FPS 不稳定，可先把 `buff_num` 调为 2；这会改变时延，必须重新记录 benchmark。

### 在 WSL 中从零构建

建议使用 WSL2 Ubuntu 20.04 或更高版本。完整流程：

```bash
sudo apt update
sudo apt install -y git cmake build-essential python3 python3-pip autoconf automake libtool

cd ~
git clone https://github.com/Sipeed/MaixCDK.git
cd MaixCDK
git checkout --detach 0beaac9d3dd06248bad5259935f5110e261dd447
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -U pip
python3 -m pip install -U -r requirements.txt
python3 -m pip install --force-reinstall "cmake>=3.13,<4"

cp -a /path/to/repository/code/maixcam_marker_detector projects/
cd projects/maixcam_marker_detector
maixcdk menuconfig
# 选择 maixcam，保存退出
maixcdk build
```

首次部署前将可执行文件和完整依赖目录上传到设备，不能只上传可执行文件：

```bash
ssh root@DEVICE_IP "mkdir -p /root/maixcam_marker_test/dl_lib"
scp build/maixcam_marker_detector root@DEVICE_IP:/root/maixcam_marker_test/
scp -r build/dl_lib/. root@DEVICE_IP:/root/maixcam_marker_test/dl_lib/
scp lateral_control.conf root@DEVICE_IP:/root/maixcam_marker_test/
```

在 MaixCAM-Pro 上运行：

```bash
killall launcher_daemon
cd /root/maixcam_marker_test
chmod +x maixcam_marker_detector
export LD_LIBRARY_PATH="$PWD/dl_lib:/mnt/system/lib:/mnt/system/usr/lib:/mnt/system/usr/lib/3rd:/lib:/usr/lib"
ldd ./maixcam_marker_detector
./maixcam_marker_detector
```

## 输出与控制面约定

主程序对每一个**成功取得且处理的**帧输出 JSONL。`found=false` 仍会输出一条记录，绝不使用上一帧预测结果冒充本帧检测。字段包括检测质量、置信度、bbox/中心、横向误差、方向、SEARCH/TRACK 状态、采集与输出时间戳、处理耗时、有效检测 FPS，以及标称距离、滤波状态与最终 `vy_mps`。

横向控制参数集中在 `lateral_control.conf`。运行时加载无需重新编译：

```bash
./maixcam_marker_detector --control-config lateral_control.conf
```

完整字段解释、数据采集和从静态到实车的调参顺序见仓库根目录的
`LATERAL_TRACKING_TUNING.md`。

## AVC1 实车输出

MaixCAM-Pro 推荐使用 UART1：`A19/UART1_TX` 接 C 板 `PG9/USART6_RX`，两板 GND
共地，均为3.3 V TTL。不要使用5 V或RS-232电平。设备节点通常为 `/dev/ttyS1`。

默认不打开串口。第一次联调使用独立的0.10 m/s发送限幅：

```bash
./maixcam_marker_detector --control-config lateral_control.conf \
  --chassis-uart /dev/ttyS1 --chassis-vy-limit 0.10
```

程序固定发送 `vx=0`，只控制横移。每个视觉周期发送一帧24字节 AVC1 命令；发送序号
包括停车帧在内持续递增。控制结果失效时发送 `valid=0,vx=0,vy=0`；首次打开串口、
相机空读和正常退出时也主动发送停车帧。串口写失败会使程序报错退出，C板另有
200 ms超时停车。`--chassis-vy-limit` 是串口侧独立安全限幅，不会改变日志中的
控制器原始 `vy_mps`；实际发送值见 `chassis_tx_vy_mps`。

目标短失联先预测100 ms；仍未找到且此前锁定过目标时，程序沿最后目标方向进入
低速横移扫描。重新识别后清除旧积分并重新居中；默认搜索8秒仍失败则发送
`valid=0` 停车。搜索速度、扫描周期和超时均在 `lateral_control.conf` 中配置。

完整接线、PS2操作和联调顺序见仓库根目录的 `REAL_CHASSIS_INTEGRATION.md`。

默认运行时无屏幕、无图像保存和无高频诊断日志。可通过信号正常退出；启动、相机打开失败和空读会写入 stderr。

## 调试图像模式

调试模式不会写入图片文件。它把两幅图并排实时发送：左侧为灰度原图，黄色框是当前搜索 ROI，青色框是通过轮廓/形状初筛的 LED 候选，绿色框和十字是最终结果；右侧是检测器实际使用的 LED 二值掩码。顶部同时显示状态、置信度、候选数/原始轮廓数、阈值和调试帧率。

连接 MaixVision 后运行：

```bash
./maixcam_marker_detector --debug
```

需要同时在设备屏幕上显示时运行：

```bash
./maixcam_marker_detector --debug-display
```

只有 `--debug-display` 会额外绘制 L 中心：所有通过大 L 初筛的候选中心是紫色小点；最终参与成功判定的三个中心用红圈标出，并以绿色线段连接。首次捕获时按画面从上到下编号为 `L0`、`L1`、`L2`，跟踪期间以相对上一帧总位移最小的方式保持编号，连续丢失并重新搜索后才重新编号。

输出的跟踪中心是两个非直角顶点的中点，即三 L 图案中心，而不是相对图案偏移 2.5 mm 的实体板中心。它不受等长直角边交换影响，可直接作为后续横向跟随的稳定测量基准。固定编号后，每组三角形只执行一次 128x128 仿射变换。

三个可选小部件仍会独立识别。调试画面以橙色斜十字标出其实际亮区中心：`S` 为小 L，`Q0`、`Q1` 为两个方块。JSON 的 `optional_features` 数组同步输出每个部件的 `found`、`center_x`、`center_y`；未命中时中心为 `-1,-1`。这些结果不会参与 `found` 判定，但会保留给后续坐标/位姿算法使用。

`--debug-display` 还会在画面底部绘制 `vy`：向左箭头表示正速度，向右箭头表示负速度，长度表示相对 `max_vy_mps` 的大小；绿色十字为三 L 几何中心，青色菱形为小部件一致性修正中心。调试绘制和图像传输会降低帧率，只用于排查，不应用于最终比赛模式。

## 实机前必须验证

本仓库没有伪造性能结论。部署后应单独记录 camera read、SEARCH_FULL、TRACK_ROI 和 end-to-end 的 mean/P50/P95/P99；同时完成 LED PWM/曝光扫描。GC4653 为滚动快门，LED PWM、过曝、运动拖影或低频条带无法由单帧传统视觉保证恢复。固定曝光（建议先扫描 1–2 ms）必须依据这些实测结果再写入配置。
