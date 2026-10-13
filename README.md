# 2026 电赛 H 题 — 车载平衡滚球运动控制系统

[![Platform](https://img.shields.io/badge/MCU-MSPM0G3507-blue)](https://www.ti.com/microcontrollers-mcus-processors/arm-based-microcontrollers/arm-cortex-m0-mcus/mspm0g3507.html)
[![Toolchain](https://img.shields.io/badge/CCS-TI%20Arm%20Clang%204.0-orange)](https://www.ti.com/tool/CCSTUDIO)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

2026 年全国大学生电子设计竞赛（TI 杯）**H 题：车载平衡滚球运动控制系统** 完整工程仓库。
循线小车沿黑色环形赛道行驶，车载摆杆通过闭环步进电机将钢球稳定在指定位置，
摄像头实时检测钢球坐标并通过串口回传主控，实现"行车 + 滚球平衡"一体化控制。

```
┌──────────────────────────────────────────────────────────────┐
│   环形黑线赛道 (1.5m 直道 + R0.5m 半圆 ×2, 总长约 6.14m)        │
│                                                              │
│   小车 (MSPM0G3507)                                          │
│   ├─ 8 路灰度循迹 + 双轮编码器里程计                          │
│   ├─ 摆杆角度闭环 (Emm_V5 步进电机, 串口位置模式)              │
│   └─ MaixCAM2 摄像头 → YOLO11 检测钢球 → UART 回传坐标 (cm)   │
└──────────────────────────────────────────────────────────────┘
```

---

## 仓库结构

```
.
├── firmware/                 # MSPM0G3507 主控固件 (CCS 工程, 可直接导入)
│   ├── main.c                #   主程序: 模式调度 / 循迹 / 遥控 / OLED
│   ├── board.syscfg          #   SysConfig 外设配置 (生成 ti_msp_dl_config)
│   ├── app/                  #   应用层: 系统模式 + 各赛题任务模式
│   ├── ball_balance/         #   滚球平衡控制 (视觉接口 / 串级PID / 高频PD)
│   ├── drivers/              #   驱动层 (电机 / 循迹 / 编码器 / OLED / 串口…)
│   ├── legacy/               #   遗留模块 (陀螺仪 / MPU6050, 保留未接入)
│   ├── Debug/                #   CCS 构建目录 (makefile + SysConfig 产物)
│   └── targetConfigs/        #   J-Link 调试配置
├── vision/                   # MaixCAM2 相机端: YOLO11 钢球检测 + UART 回传
│   ├── main.py               #   检测 / 像素→厘米标定 / 串口协议
│   ├── app.yaml              #   MaixCAM 应用打包清单
│   ├── models/               #   导出的 YOLO11 模型 (mud / axmodel)
│   └── dist/                 #   打包好的 MaixCAM 应用 zip
├── docs/                     # 设计文档 (接线 / 算法 / 任务映射)
├── tools/                    # MATLAB 辅助调参脚本
└── scripts/build.sh          # 命令行一键编译脚本 (TI Arm Clang)
```

## 快速开始

### 方式一: CCS (推荐, 与竞赛环境一致)

1. 安装 **CCS 20.5.1** 及产品 **MSPM0-SDK 2.10.00.04**、**SysConfig 1.26.2+**；
2. CCS → `Project → Import → CCS Projects`，选择 `firmware/` 目录，工程名 `BallBalanceCar`；
3. 确认 `Debug/` 下 makefile 中的 `CG_TOOL_ROOT` / `MSPM0_SDK_ROOT` 指向本机路径；
4. 右键工程 → `Build Project`，产物为 `Debug/BallBalanceCar.out`；
5. J-Link 连接目标板 (targetConfigs/MSPM0G3507.ccxml)，下载运行。

### 方式二: 命令行编译 (无 IDE)

```bash
# 默认使用本机 C:/ti 工具链, 可用环境变量覆盖
export CG_TOOL_ROOT=/c/ti/ti_cgt_arm_llvm_4.0.2.LTS
export MSPM0_SDK_ROOT=/c/ti/mspm0_sdk_2_10_00_04
bash scripts/build.sh          # 编译 + 链接, 输出 firmware/Debug/BallBalanceCar.out
```

## 硬件平台

| 项目 | 参数 |
| --- | --- |
| 主控 | TI MSPM0G3507 (Cortex-M0+, 128 KB Flash, 32 KB SRAM, LQFP-64) |
| 电机驱动 | TB6612 ×2 路 (PWM 20 kHz, 速度单位 %) |
| 编码器 | 13 PPR × 1:28 减速箱, A/B 相上升沿 2 倍频, 728 计数/轮 |
| 轮径 | 65 mm (周长 ≈ 204.2 mm, 分辨率 ≈ 0.28 mm/计数) |
| 摆杆执行 | 张大头 42 步进电机 (Emm_V5 串口闭环, 位置模式 300 RPM) |
| 视觉 | MaixCAM2 + YOLO11, UART 115200 回传钢球坐标 (cm) |
| 显示 | 0.96" OLED (I2C, 软件模拟) |
| 调参 | MATLAB 继电反馈自整定 (`tools/pid_relay_tuner.m`) |

完整引脚分配见 [docs/hardware-wiring.md](docs/hardware-wiring.md)。

## 赛题任务 ↔ 程序模式

S2 按键循环切换模式，S1 启动/停止，S3/S4 在线调参。OLED 首行显示当前模式。

| 赛题要求 | 分值 | 模式 (OLED) | 源文件 |
| --- | --- | --- | --- |
| ① 图传装置 | 6 | ESP32 图传常开 | `drivers/esp32_camera_link.c` |
| ② A 点出发顺时针一圈回 A, ≤20 s, 停车偏差 ≤2 cm | 16 | `MODE:007` | `app/mode_loop_arc_stop.c` |
| ③ 静止: 球 O → +5 cm → -5 cm, ≤5 s | 13 | `MODE:+5>-5` / `MODE:5.NEW` | `ball_balance/ball_route.c`, `ball_route_ramp.c` |
| ④ A → B, ≤8 s, 球稳定在 O (±1 cm) | 20 | `MODE:AB20` | `app/mode_ab20.c` |
| ⑤ 顺时针一圈, ≤30 s, 球稳定在 O | 20 | `MODE:006` / `MODE:ABCD` | `app/mode_loop_arc.c`, `mode_full_loop.c` |
| ⑥ 顺时针一圈, ≤30 s, 球稳定任意指定位置 | 20 | `MODE:006(1)` / `MODE:HOLD6` | `app/mode_loop_offset.c`, `mode_arc_trajectory.c` |

模式循环顺序: `TRACK → HFZERO → ZERO → +5>-5 → DRIVE1 → HOLD6 → 006 → 007 → 5.NEW → 006(1) → AB → AB20 → ABCD → TRACK50 → RC → TRACK`

## 控制算法

- **循迹**: 8 路灰度加权误差 → 一阶滤波 → P 校正 + 丢线回拐；轮速闭环 (PI + 前馈) 保证直道速度对称
- **滚球平衡**: 视觉坐标低通 → PD (位置 + 速度) → 摆杆目标角度 → Emm_V5 位置模式执行，5 ms 控制周期
- **整圈轨迹**: 编码器里程计按"直道-圆弧-直道-圆弧"四段规划速度，坡度前馈补偿加减速
- **串级 PID 回零**: 位置外环输出目标球速，速度内环输出平台角度

详见 [docs/control-algorithms.md](docs/control-algorithms.md)。

## 视觉端通信协议

MaixCAM2 → MSPM0 (UART3, PB6=TX / PB7=RX, 115200 8N1):

```text
#-12.5!      钢球坐标 -12.5 cm (范围 ±12.5 cm, 自动 ×10 转 mm)
#+3.25!      钢球坐标 +3.25 cm
#N!          本帧未检测到钢球
```

兼容旧双坐标协议 `#320.00,300.00!` (像素横坐标)。

## 整理说明 (v1.0, 2026-10)

本仓库由竞赛期间的散乱工程整理而成:

- 所有文件按 **驱动层 / 应用层 / 平衡控制层** 分层命名 (`motor_tb6612.c`、`mode_loop_arc.c`、`ball_hf_zero.c` 等)，废弃 `006.c` / `007.c` / `-5new.c` / `0.c` / `empty.c` 等临时命名；
- `AB.c` → `app/sys_mode.c` (系统模式管理 + AB 直行), `modeAB.c` → `app/mode_ab20.c` 等, 一一对应;
- 合并重复的 `timebase` 模块: 原工程根目录 `timebase.c` 与 `MPU6050/timebase.c` 定义相同符号, 整理后保留根目录版本 (SysTick 优先级 1, 相机中断优先级 0, 与设计文档一致), `MPU6050/` 仅剩遗留头文件移入 `legacy/`;
- 全部 32 个编译单元经 TI Arm Clang 4.0.2 + MSPM0 SDK 2.10.00.04 编译链接验证通过 (与原工程基线对比: `.text` 37,248 B vs 37,208 B, 差异来自 timebase 合并);
- 控制逻辑零改动, 仅重命名/移动文件并同步修正 `#include` 与构建文件。

## License

MIT License, 见 [LICENSE](LICENSE)。竞赛原始代码版权归参赛队所有。
