# MaixCAM Pro AVC1 UART Test

这是一个独立的 MaixCDK UART 发送测试工程，不依赖视觉检测主程序。

## UART 参数

- TX: `A19` -> `UART1_TX`
- Device: `/dev/ttyS1`
- Baud: `115200`
- Format: `8N1`
- Flow control: none
- Frequency: `20 Hz`
- Frame size: `24 bytes`

发送的 `valid` 固定为 `1`。`vx` 每 15 秒循环：前 5 秒 `0.00`，中间 5 秒 `0.05`，最后 5 秒 `0.00`。`vy` 固定为 `0.00`。

## 帧格式

| Offset | Size | Field |
| --- | ---: | --- |
| 0 | 4 | magic `0x31435641` (little-endian) |
| 4 | 4 | sequence |
| 8 | 4 | `vx` float32 |
| 12 | 4 | `vy` float32 |
| 16 | 1 | valid = 1 |
| 17 | 3 | zero padding |
| 20 | 4 | AVC1 checksum |

## 编译

先进入本目录：

```bash
cd code/maixcam_marker_detector/uart_test
```

确保 MaixCDK 路径已经设置：

```bash
export MAIXCDK_PATH=/path/to/MaixCDK
```

然后按当前 MaixCDK 环境使用：

```bash
maixcdk build
```

如果环境使用 MaixCDK Python 入口，则使用对应的 `maixcdk.py build` 命令。

## 运行

把编译得到的 MaixCAM Pro 可执行文件复制到设备后：

```bash
chmod +x ./maixcam_avc1_uart_test
./maixcam_avc1_uart_test
```

正常输出类似：

```text
AVC1 UART test started: A19 UART1_TX, /dev/ttyS1, 115200 8N1, 20 Hz
sent seq=0 vx=0.00 valid=1 (24 bytes)
sent seq=1 vx=0.00 valid=1 (24 bytes)
```

USB-TTL 验证时连接 `MaixCAM Pro A19 (TX) -> USB-TTL RX`，并连接双方 GND。
