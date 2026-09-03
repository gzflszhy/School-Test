# MaixCAM-Pro Marker Detector

这是 MaixCAM-Pro（GC4653）上的 C++17 / MaixCDK / OpenCV 端侧 LED Marker 检测程序。生产入口没有 GUI、显示输出或逐帧保存；每处理一帧输出一行 JSONL，适合直接接入下游滤波/控制记录器。

## 运行基线

- 请求 GC4653 `640x360 @ 60 FPS`、`FMT_GRAYSCALE`、`buff_num=1`；
- 启动后默认丢弃 30 帧，以便自动曝光稳定；手动曝光时，使用配置中的
  `exposure_us`（微秒）后再开始预热；
- 每次读取前调用 `Camera::clear_buff()`，优先新鲜帧而非处理积压帧；
- 相机返回的灰度缓冲区通过 `cv::Mat` 零拷贝封装，并由 `CameraFrame` 维持其生命周期。检测器必须在该 `CameraFrame` 仍存在时完成 `process()`。

`capture_timestamp_us` 是 `Camera::read()` 成功返回时记录的 host `steady_clock` 单调时间；目前 MaixCDK `Image` API 未提供传感器曝光时间戳，不能把它解释为曝光起点。`output_timestamp_us` 为结果序列化前的同一时钟时间，二者差值是本程序可测的 capture-to-output 时延。

## MaixCDK 构建与部署

将本目录作为 MaixCDK 项目目录导入/打开，并使用 MaixCDK（或 MaixVision）的 Release 构建流程。`main/CMakeLists.txt` 是官方 component 格式：它声明 `vision` 依赖，后者提供 `maix::camera::Camera`、`maix::image` 和 MaixCDK 的 OpenCV 集成。

项目只请求官方 API 已公开的调用：

```cpp
maix::camera::Camera camera(640, 360, maix::image::FMT_GRAYSCALE,
                            nullptr, 60.0, 1, true, false);
camera.skip_frames(30);
maix::image::Image *image = camera.read(true, -1);
// use image->data() while image is alive, then delete image
```

不要使用 `cv::VideoCapture`。不要加 `display` 依赖。实机如发现 60 FPS 不稳定，可先把 `buff_num` 调为 2；这会改变时延，必须重新记录 benchmark。

## 输出与控制面约定

主程序对每一个**成功取得且处理的**帧输出 JSONL。`found=false` 仍会输出一条记录，绝不使用上一帧预测结果冒充本帧检测。字段包括检测质量、置信度、bbox/中心、横向误差、方向、SEARCH/TRACK 状态、采集与输出时间戳、处理耗时和有效检测 FPS。

运行时无屏幕、无图像保存和无高频诊断日志。可通过信号正常退出；启动、相机打开失败和空读会写入 stderr。

## 实机前必须验证

本仓库没有伪造性能结论。部署后应单独记录 camera read、SEARCH_FULL、TRACK_ROI 和 end-to-end 的 mean/P50/P95/P99；同时完成 LED PWM/曝光扫描。GC4653 为滚动快门，LED PWM、过曝、运动拖影或低频条带无法由单帧传统视觉保证恢复。固定曝光（建议先扫描 1–2 ms）必须依据这些实测结果再写入配置。`Camera::gain(int)` 的设备刻度尚未在本项目中标定，因此 `analogue_gain`/`digital_gain` 不会被静默映射。
