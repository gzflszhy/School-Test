# MaixCAM Pro AVC1 UART Send / Listen Test

这个目录现在包含两个独立的 MaixCDK UART 测试工程，用于验证 MaixCAM Pro 串口发送与接收，不依赖视觉检测主程序。

```text
uart_test/
├── send/
│   ├── app.yaml
│   └── main/
│       ├── CMakeLists.txt
│       └── src/main.cpp
└── listen/
    ├── app.yaml
    └── main/
        ├── CMakeLists.txt
        └── src/main.cpp
```

## UART 参数

- Device: `/dev/ttyS1`
- Baud: `115200`
- Format: `8N1`
- Flow control: none
- TX: `A19 -> UART1_TX`
- RX: `A18 -> UART1_RX`

## send

`send` 每 50 ms 发送一个 24-byte AVC1 frame，即 20 Hz。

- `valid = 1`
- `vy = 0.00`
- `vx` 每 15 秒循环：`0.00 -> 0.05 -> 0.00`

帧格式：

| Offset | Size | Field |
| --- | ---: | --- |
| 0 | 4 | magic `0x31435641` little-endian |
| 4 | 4 | sequence |
| 8 | 4 | `vx` float32 |
| 12 | 4 | `vy` float32 |
| 16 | 1 | valid |
| 17 | 3 | zero padding |
| 20 | 4 | AVC1 checksum |

编译：

```bash
cd code/maixcam_marker_detector/uart_test/send
export MAIXCDK_PATH=/path/to/MaixCDK
maixcdk build
```

运行编译得到的程序后，正常输出类似：

```text
AVC1 SEND: A19 UART1_TX, /dev/ttyS1, 115200 8N1, 20 Hz
sent seq=0 vx=0.00 vy=0.00 valid=1 (24 bytes)
```

## listen

`listen` 在 `A18 / UART1_RX` 上持续接收数据。它会从字节流中寻找 AVC1 magic，按 24-byte 定长帧解析，并检查 checksum。

编译：

```bash
cd code/maixcam_marker_detector/uart_test/listen
export MAIXCDK_PATH=/path/to/MaixCDK
maixcdk build
```

收到正确 frame 时输出类似：

```text
AVC1 LISTEN: A18 UART1_RX, /dev/ttyS1, 115200 8N1
Waiting for 24-byte AVC1 frames...
recv seq=123 vx=0.050 vy=0.000 valid=1 checksum=OK
```

checksum 错误时会打印 `BAD checksum`。

## 接线测试

### MaixCAM Pro -> USB-TTL

测试 `send`：

```text
MaixCAM A19 (TX) -> USB-TTL RX
MaixCAM GND      -> USB-TTL GND
```

### USB-TTL -> MaixCAM Pro

测试 `listen`：

```text
USB-TTL TX       -> MaixCAM A18 (RX)
USB-TTL GND      -> MaixCAM GND
```

### 两块设备直接 UART

```text
Device A TX -> Device B RX
Device A RX <- Device B TX
GND         <-> GND
```

MaixCAM Pro UART 为 3.3 V TTL，请确认对端电平兼容。
