# 双电机线性联动控制系统 · LabVIEW 上位机

基于 STM32F103C8T6 的双直流电机控制系统：上位机（LabVIEW / Python）通过 USB 虚拟串口下发一个速度指令，下位机按固定比例 `vB = k · vA` 联动两个电机，实现线性速度关系。

```
   LabVIEW / Python 上位机
          │  USB CDC (ASCII 命令: S+500 K50 R X)
          ▼
   STM32F103C8T6  (5 ms 调度)
     ├─ 命令解析 (protocol.c)
     └─ 线性联动 + 斜坡 (motor.c)
          │  PWM + 方向 GPIO
          ▼
   TB6612 双路 H 桥 ──► 电机A / 电机B
```

## 核心特性

- **固定比例线性联动**：主机只发一个速度 `vA`，从机自动算出 `vB = k·vA`（k 运行时可调，0~100%）。整条速度范围内 `vB / vA` 恒定。
- **斜坡平滑**：每 5 ms 一拍、每拍最多变化 25‰，速度从 -1000 到 +1000 平滑过渡，无突变。
- **清晰的命令协议**：`S` 设速度 / `K` 设比例 / `R` 查状态 / `X` 急停，行缓冲解析，抗粘包。
- **双上位机方案**：LabVIEW VI（图形化，可画 XY 图演示线性度）+ Python 脚本（带离线 mock，无硬件也能验证）。

## 目录结构

```
firmware/                    STM32F103C8T6 固件（Keil MDK / HAL）
  Core/Src/motor.c           双电机驱动 + 线性联动 + 斜坡
  Core/Src/protocol.c        ASCII 命令解析
  Core/Src/main.c            5 ms 非阻塞调度
  MDK-ARM/*.uvprojx          Keil 工程（已含新增源文件）
host/
  motor_host.py              Python 上位机（--mock 离线仿真 / --port 真板）
docs/
  LabVIEW上位机搭建指南.md     VI 分步搭建 + 协议 + 排查
  通信协议.md                 完整协议说明与时序
  硬件接线.md                 引脚映射与驱动供电
```

## 快速开始

```bash
# 1. 无硬件，先验证线性联动逻辑（离线仿真）
cd host
python motor_host.py --mock --sweep
#   预期：整条范围内 B = k*A，最大线性误差 = 0

# 2. 接上板子后（COMx 换成实际端口）
python motor_host.py --port COM3
> S+500        # A=+500, B=+250 (默认 k=0.5)
> K25          # 改比例，B 变为 A 的 25%
> R            # 查状态 A/B/K
> X            # 急停

# 3. LabVIEW：按 docs/LabVIEW上位机搭建指南.md 搭建 VI
```

## 验证状态

| 项目 | 状态 | 证据 |
|---|---|---|
| 线性联动算法（B=k·A） | ✅ 逻辑验证通过 | Python 等价仿真：9 个速度点 + 4 种比例，线性误差 0 |
| 斜坡平滑性 | ✅ 逻辑验证通过 | 单拍变化严格 ≤25‰ |
| 通信协议解析 | ✅ 逻辑验证通过 | mock 下 S/K/R/X 全部正确 |
| Keil 工程编译 | ⚠️ 未在本环境编译 | 本机无 ARM 工具链；源码已做语法与工程配置检查，需在 Keil5 中 `Build` 确认 |
| 实板/实车运行 | ⚠️ 未验证 | 需在实物上电测试，注意先接电机驱动独立供电 |

> 说明：本环境为 Windows，无 ARM 交叉编译器与实物硬件。算法层与协议层已用与固件逐行等价的 Python 模型完整验证；Keil 编译与实板运行留待有硬件时完成。

## 硬件映射

| 信号 | 引脚 | 说明 |
|---|---|---|
| 电机 A PWM | PA6 (TIM3_CH1) | 10 kHz |
| 电机 A 方向 | PB12 / PB13 | H 桥 IN1 / IN2 |
| 电机 B PWM | PB1 (TIM3_CH4) | 10 kHz |
| 电机 B 方向 | PB14 / PB15 | H 桥 IN1 / IN2 |
| USB CDC | PA11 / PA12 | 虚拟串口 |
| 心跳 LED | PA5 | 5 ms 翻转 |

## License

MIT
