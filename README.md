# stm32-dual-motor-control

STM32F103C8T6 双直流电机控制。上位机经 USB CDC 发一个速度指令，下位机解算另一个电机，使两者满足：

```
sin(v1) = k · cos(v2)

v1 =  a      · π / 2000
v2 = (1000-b) · π / 2000
```

`a`、`b` 是两电机的速度指令（‰，-1000~1000）。k=1 时 b=a，两电机同速同向；k<1 时 A 的幅值受限，B 的变化更大。

## 目录

```
firmware/    STM32F103 HAL 固件（Keil MDK 工程）
  Core/Src/motor.c      电机驱动、三角解算、5 ms 斜坡
  Core/Src/protocol.c   ASCII 命令解析
  Core/Src/main.c       5 ms 调度
host/
  motor_host.py         Python 上位机，--mock 可离线运行
docs/
  LabVIEW上位机搭建指南.md
  通信协议.md
  硬件接线.md
```

## 命令

以换行结尾，按行解析。

| 命令 | 说明 |
|---|---|
| `S+500` | 设电机 A 速度 +500‰ |
| `K50` | k = 0.50（范围 1~100） |
| `R` | 查询，返回 `A=+500 B=+250 K=0.50` |
| `X` | 急停 |
| `?` | 帮助 |

## 运行

```bash
cd host
python motor_host.py --mock --sweep    # 离线验证 sin(v1)=k*cos(v2)

python motor_host.py --mock            # 离线交互
python motor_host.py --port COM3       # 接板子
```

离线验证结果：k=1 时残差 0；k=0.5 时最大残差 0.00059；k=0.25 时 0.00011。

## 定义域

`|sin(v1)| ≤ k`，所以电机 A 的指令被限制在：

```
|a| ≤ (2000/π) · asin(k)
```

k=1 → |a|≤1000；k=0.5 → |a|≤333；k=0.25 → |a|≤160。超出部分固件会截断，避免 `acos` 无解。

## 硬件

| 信号 | 引脚 |
|---|---|
| 电机 A PWM / 方向 | PA6 (TIM3_CH1) / PB12、PB13 |
| 电机 B PWM / 方向 | PB1 (TIM3_CH4) / PB14、PB15 |
| USB CDC | PA11、PA12 |
| 心跳 LED | PA5 |

电机驱动需要独立供电。

## 验证状态

Python 等价模型已验证三角约束、斜坡与协议解析。Keil 编译和实板运行未在本机完成，需在 Keil5 中打开 `firmware/MDK-ARM/Motor_Control_LabVIEW.uvprojx` 编译。用到 `sinf` / `acosf` / `asinf`，需 C99 数学库。

## License

MIT
