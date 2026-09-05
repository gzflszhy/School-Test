# MaixCAM Pro AVC1 constant vy sender

Standalone MaixCDK C++ UART test project.

It continuously sends the existing 24-byte AVC1 chassis command frame at 20 Hz:

- TX pin: A16
- pinmux: UART0_TX
- device: /dev/ttyS0
- baud: 115200
- format: 8N1, no flow control
- vx: 0.0 m/s
- vy: +0.2 m/s
- valid: 1

## Build

Make sure `MAIXCDK_PATH` points to your MaixCDK checkout, then from this directory run the same MaixCDK build flow used by the parent project, for example:

```bash
export MAIXCDK_PATH=/path/to/MaixCDK
maixcdk build
```

If your setup uses the MaixCDK Python entry script instead of the installed `maixcdk` command, use that equivalent build command.

## Run

Copy the generated executable to the MaixCAM Pro and run it directly. The process does not stop on its own; use Ctrl+C to terminate it.

Expected log lines look like:

```text
AVC1 constant sender: A16 UART0_TX, /dev/ttyS0, 115200 8N1, 20 Hz, vx=0.00, vy=0.20, valid=1
sent seq=0 vx=0.00 vy=0.20 valid=1 (24 bytes)
```
