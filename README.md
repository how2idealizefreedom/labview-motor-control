# labview-motor-control

STM32F103C8T6 双直流电机控制。上位机经 USB CDC 发一个速度指令，下位机解算另一个电机，使两者满足：

```
sin(v1) = k · cos(v2)

v1 =  a      · π / 2000
v2 = (1000-b) · π / 2000
```

`a`、`b` 是两电机速度指令（‰，-1000~1000）。k=1 时 b=a；k<1 时 A 的幅值受限，B 的变化更大。

## 结构

```
firmware/
  Core/Src/coupling.c    三角解算、限幅、斜坡（纯算法）
  Core/Src/pid.c         PI + 积分限幅（纯算法）
  Core/Src/encoder.c     编码器测速（TIM2 / TIM4）
  Core/Src/motor.c       状态机、看门狗、开环/闭环切换、H 桥驱动
  Core/Src/protocol.c    命令解析、状态与遥测
  Core/Src/main.c        5 ms 调度
  MDK-ARM/               Keil 工程
host/motor_host.py       Python 上位机
tests/                   离线测试（Python + C）
docs/                    协议、硬件、LabVIEW、控制说明
```

三层分开：算法层无硬件依赖，可在 PC 上单测；驱动层接 STM32 外设；应用层是协议与调度。

## 命令

以换行结尾。

| 命令 | 说明 |
|---|---|
| `S+500` | 设电机 A 速度 +500‰ |
| `K50` | k = 0.50（1~100） |
| `L1` / `L0` | 开 / 关编码器速度闭环（默认开环） |
| `T1` / `T0` | 开 / 关遥测（每 50 ms 一帧） |
| `R` | 基本状态：`A=.. B=.. K=..` |
| `V` | 扩展状态：输出、目标、实测、状态、闭环标志 |
| `E` | 编码器计数与实测速度 |
| `X` | 急停 |
| `?` | 帮助 |

## 运行

```bash
cd tests
python3 test_firmware.py        # 26 项离线测试
make test-c                     # 若本机有 gcc，跑 C 算法单测

cd ../host
python motor_host.py --mock --sweep       # 三角约束残差
python motor_host.py --mock --loop-sweep  # 闭环跟踪误差
python motor_host.py --port COM3          # 接板子
```

遥测开启后设备每 50 ms 主动发一行，以 `>` 开头，格式与 `V` 相同。上位机的 `parse_extended()` 可直接解析。

## 控制

开环：斜坡生成 A 的目标，解算 B 的目标，直接映射到 PWM。

闭环（`L1`）：斜坡生成目标后，两路 PI 用编码器实测速度做反馈，输出 PWM。PI 参数 kp=0.4、ki=1.5，积分限幅按 `out_max/ki` 处理，避免饱和绕卷。

```
|a| ≤ (2000/π) · asin(k)
```

k=1 → |a|≤1000；k=0.5 → |a|≤333；k=0.25 → |a|≤160。固件按此截断 A，避免 `acos` 无解。

## 安全

- 500 ms 内没有收到任何命令，两电机自动刹车，状态置 `T`（timeout）。任何命令都会刷新看门狗。
- 超长行（>63 字节）整帧丢弃，不会截断后误解析。
- 上电先不接电机，确认串口和 `S0` 正常后再接；电机驱动独立供电，与 STM32 共地。

## 硬件

| 信号 | 引脚 |
|---|---|
| 电机 A PWM / 方向 | PA6 (TIM3_CH1) / PB12、PB13 |
| 电机 B PWM / 方向 | PB1 (TIM3_CH4) / PB14、PB15 |
| 编码器 A / B | PA0、PA1 (TIM2) / PB6、PB7 (TIM4) |
| USB CDC | PA11、PA12 |
| 心跳 LED | PA5 |

闭环需要 `ENCODER_FULL_CPS`（满速计数率）按实板标定；方向反了就改 `ENCODER_A_DIR` / `ENCODER_B_DIR`。

## 验证状态

Python 镜像逐行对应固件逻辑，26/26 项通过：三角约束、定义域、斜坡、PI、协议解析与异常、看门狗、闭环收敛。Keil 编译与实板运行未在本机完成。

## License

MIT
