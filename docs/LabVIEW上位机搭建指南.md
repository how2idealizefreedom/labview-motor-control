# LabVIEW 上位机搭建指南

LabVIEW 通过 VISA 打开 STM32 的虚拟串口，发送 `S` / `K` 命令，接收 `R` 返回的状态。

## 1. 清单

| 类别 | 内容 |
|---|---|
| 软件 | LabVIEW 2018+、NI-VISA |
| 硬件 | STM32F103C8T6 板、双路 H 桥（TB6612 等）、两个直流电机 |
| 连接 | USB 线（PC↔板）、电机驱动独立供电 |

STM32 USB CDC 在 Windows 下一般免驱识别为 COM 口。

## 2. 找串口

1. 烧录固件，USB 线接板子的 USB 口。
2. 设备管理器 → 端口，记下 `USB Serial Device (COMx)`。
3. 没出现就装 ST VCP 驱动，或检查 `usbd_desc.c` 的 VID/PID。

先用 `python host/motor_host.py --port COMx` 确认能通，再开 LabVIEW。

## 3. 协议

| 命令 | 含义 | 例子 |
|---|---|---|
| `S<速度>` | 设电机 A，范围 -1000~1000‰ | `S+500\n` |
| `K<百分比>` | 设 k，范围 1~100 | `K50\n` |
| `R\n` | 查状态 | `A=+500 B=+972 K=0.50` |
| `X\n` | 急停 | — |

电机 B 由下位机解算，满足 `sin(v1) = k·cos(v2)`。

## 4. VI 搭建

前面板控件：

| 控件 | 类型 | 名字 | 范围 |
|---|---|---|---|
| 速度滑块 | 数值输入 | `Speed A` | -1000~1000 |
| 比例输入 | 数值输入 | `Ratio K` | 1~100，默认 100 |
| 停止按钮 | 布尔 | `STOP` | — |
| 状态显示 | 字符串显示 | `Status` | — |
| 实际值 | 两个数值显示 | `A act` / `B act` | — |
| 串口 | VISA 资源名 | `VISA Resource` | `COMx` |

程序框图：

```
① VISA Configure Serial Port
     Baud Rate = 115200, 8 data, 1 stop, no parity

② 速度变化时发送
     Concatenate Strings: "S" + Format("%+d", Speed A) + "\n"
     -> VISA Write
     %+d 保证正数带 + 号，与固件解析兼容

③ 比例变化时发送
     "K" + Format("%d", Ratio K) + "\n" -> VISA Write

④ 读取状态
     VISA Read (Byte Count = 100) -> Status
     Scan from String: "A=%d B=%d K=%f" -> A act, B act

⑤ 急停
     STOP -> VISA Write "X\n" -> VISA Close

⑥ While Loop，周期 50 ms
```

建议用 Event Structure 分三个分支：`Speed A: Value Change`、`Ratio K: Value Change`、`STOP: Value Change`。只在操作时发命令，串口不会拥堵。

## 5. 验证

拖动 `Speed A`，读 `A act` / `B act`：

| k | A | B |
|---:|---:|---:|
| 1.00 | +600 | +600 |
| 1.00 | -400 | -400 |
| 0.50 | +166 | +344 |
| 0.50 | +333 | +972 |
| 0.25 | +80 | +334 |

k=1 时 A=B。k<1 时 A 被限制在 `|a| ≤ (2000/π)·asin(k)` 内，B 变化更大。把多组 (A, B) 做成 XY 图可以看到关系曲线。

## 6. 排查

| 现象 | 原因 | 处理 |
|---|---|---|
| VISA 打不开 | 端口错或被占用 | 确认 COMx，关闭其他串口工具 |
| 发命令无返回 | 没加 `\n` | 命令必须以换行结尾 |
| 返回乱码 | 两端波特率不一致 | LabVIEW 设 115200 |
| 电机不转 | 驱动没供电 / 方向脚接错 | 驱动独立供电，检查 PB12/13、PB14/15 |
| 两电机转向相反 | 线序或方向定义 | 对调一路电机线，或改 `motor.c` 方向 GPIO |

## 7. 没有 LabVIEW 时

`host/motor_host.py` 是同协议的命令行上位机。`--mock` 不需要硬件，`--sweep` 会扫一遍并打印 `sin(v1) - k·cos(v2)` 的残差。
