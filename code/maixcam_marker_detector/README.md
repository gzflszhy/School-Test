# MaixCAM-Pro Marker Detector

这是 MaixCAM-Pro（GC4653）上的 C++17 / MaixCDK / OpenCV 端侧 LED Marker 检测程序。生产入口没有 GUI、显示输出或逐帧保存；每处理一帧输出一行 JSONL，适合直接接入下游滤波/控制记录器。

当前主判定只依赖三个 30 mm 大 L：三者分别通过轮廓形状筛选，且其中心组成误差范围内符合 CAD 比例的等腰直角三角形，即可输出 `TRACKABLE`。模板、小 L、两个方块和黑色板不再能够否决这组三角形；它们只用于候选排序、增加报告置信度，以及在两个以上小图形命中时输出方向和 `FULL_ID`。

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

将本目录作为 MaixCDK 项目目录导入/打开，并使用 MaixCDK（或 MaixVision）的 Release 构建流程。`main/CMakeLists.txt` 是官方 component 格式：它声明 `vision` 依赖，后者提供 `maix::camera::Camera`、`maix::image` 和 MaixCDK 的 OpenCV 集成。

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

主程序对每一个**成功取得且处理的**帧输出 JSONL。`found=false` 仍会输出一条记录，绝不使用上一帧预测结果冒充本帧检测。字段包括检测质量、置信度、bbox/中心、横向误差、方向、SEARCH/TRACK 状态、采集与输出时间戳、处理耗时和有效检测 FPS。

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

只有 `--debug-display` 会额外绘制 L 中心：所有通过大 L 初筛的候选中心是紫色小点；最终参与成功判定的三个中心用红圈标出，并以绿色线段连接。`R` 是检测到的直角顶点，`A`、`B` 是两条直角边的另外两个端点。该绘制仅用于人工核对，不进入生产模式的计算输出。

三个可选小部件仍会独立识别。调试画面以橙色斜十字标出其实际亮区中心：`S` 为小 L，`Q0`、`Q1` 为两个方块。JSON 的 `optional_features` 数组同步输出每个部件的 `found`、`center_x`、`center_y`；未命中时中心为 `-1,-1`。这些结果不会参与 `found` 判定，但会保留给后续坐标/位姿算法使用。

调试绘制和图像传输会降低帧率，只用于排查，不应用于最终比赛模式。判断方法：右图没有完整 LED，优先检查曝光/PWM/阈值；右图完整但左图没有青框，说明轮廓条件仍过严；有至少三个位置正确的青框但没有绿框，说明拓扑或模板验证失败。

## 实机前必须验证

本仓库没有伪造性能结论。部署后应单独记录 camera read、SEARCH_FULL、TRACK_ROI 和 end-to-end 的 mean/P50/P95/P99；同时完成 LED PWM/曝光扫描。GC4653 为滚动快门，LED PWM、过曝、运动拖影或低频条带无法由单帧传统视觉保证恢复。固定曝光（建议先扫描 1–2 ms）必须依据这些实测结果再写入配置。`Camera::gain(int)` 的设备刻度尚未在本项目中标定，因此 `analogue_gain`/`digital_gain` 不会被静默映射。
