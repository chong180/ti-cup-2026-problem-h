# 赛题任务 ↔ 程序模式映射

H 题要求 (满分 100 + 设计报告 20) 与固件模式的对应关系。
S2 循环切换模式, S1 启动/停止, OLED 首行显示模式名。

## 模式循环顺序

```
TRACK → HFZERO → ZERO → +5>-5 → DRIVE1 → HOLD6 → 006 → 007
      → 5.NEW → 006(1) → AB → AB20 → ABCD → TRACK50 → RC → (TRACK)
```

## 任务对照表

| # | 赛题要求 | 分值 | 使用模式 | 实现要点 |
| --- | --- | --- | --- | --- |
| 1 | 图传装置: 发送模块装车, 实时显示 + 完整记录钢球滚动视频 | 6 | ESP32 常开 | `drivers/esp32_camera_link.c` 上电自动配网 (AP/STA), 手机或 PC 连热点看图传; 摄像头固定于摆杆凹槽上方覆盖整杆 |
| 2 | A 点出发顺时针一圈回 A, ≤20 s, 停车偏差 ≤2 cm | 16 | `MODE:007` | `app/mode_loop_arc_stop.c`: 编码器四段圆弧轨迹 (5600 mm), 到 A 点直接停车; 高占空比 34% 保证 20 s 内完成 |
| 3 | 静止: 球 O → +5 cm → 折返 −5 cm 并稳定, ≤5 s, 误差 ≤1 cm | 13 | `MODE:+5>-5` / `MODE:5.NEW` | `ball_balance/ball_route.c` (阶跃快速轨迹) / `ball_route_ramp.c` (线性扫描平滑轨迹) |
| 4 | A → B, ≤8 s, 球稳定在 O (±1 cm) | 20 | `MODE:AB20` | `app/mode_ab20.c`: 1.5 m 定距 + 球中心 PD 稳定, 加减速前馈角度 |
| 5 | 顺时针一圈, ≤30 s, 球稳定在 O | 20 | `MODE:006` / `MODE:ABCD` | `app/mode_loop_arc.c`: 圆弧回 A + 过线后追加直线循迹; `mode_full_loop.c` 为等价的编码器整圈模式 |
| 6 | 顺时针一圈, ≤30 s, 球稳定任意指定位置 (±1 cm) | 20 | `MODE:006(1)` / `MODE:HOLD6` | `app/mode_loop_offset.c`: 整圈中球稳定在 +70 mm 固定偏置; `mode_arc_trajectory.c` 支持按键时刻任意位置为目标的静止保持 |
| 7 | 其他 (创新) | 5 | — | 8 路循迹 + 轮速闭环 + 在线调参 + MATLAB 自整定 + 模块化工程 |
| 8 | 设计报告 | 20 | — | 本仓库 `docs/` 提供接线表、算法说明、任务映射, 可直接作为报告素材 |

## 各模式状态机速查

### `app/mode_loop_arc.c` (MODE:006, 任务⑤主模式)

```
IDLE → HOLD(球稳定, 300 ms) → DRIVE(圆弧轨迹 0..5600 mm)
     → TRACK(追加直线循迹 500/1300 mm) → FINAL(球沉降 800 ms) → DONE
```

### `app/mode_loop_arc_stop.c` (MODE:007, 任务②)

```
IDLE → HOLD → DRIVE(圆弧轨迹 0..5600 mm, 到 A 点) → FINAL → DONE
```

### `app/mode_ab20.c` (MODE:AB20, 任务④)

```
IDLE → WAIT_BALL(等球可见) → ACCEL(1.5 s 加速) → CRUISE(定距 1.5 m)
     → DECEL(700 mm 内线性减速) → FINAL_BALANCE(球稳定 1.2 s) → DONE
```

### `ball_balance/ball_route.c` (MODE:+5>-5, 任务③)

```
IDLE → WAIT_ZERO(球回中) → TO_POSITIVE(+5 cm) → TO_NEGATIVE(−5 cm) → DONE
```

### `ball_balance/ball_hf_zero.c` (MODE:HFZERO)

```
IDLE → ACQUIRE → HOLD → FAST(大误差快速模式) → LOST(丢球保护回零) → DONE
```

## 按键功能矩阵

| 模式 | S1 | S2 | S3 | S4 |
| --- | --- | --- | --- | --- |
| TRACK | 启/停循迹 | 切换模式 | 距离 +0.5 cm | 距离 −0.5 cm |
| ZERO / HOLD6 | 启/停闭环 | 切换模式 | 位置环 Kp +0.05 | 位置环 Kp −0.05 |
| HFZERO | 启/停 | 切换模式 | 位置增益 +0.02 | 位置增益 −0.02 |
| 其他任务模式 | 启/停任务 | 切换模式 | 距离 +0.5 cm | 距离 −0.5 cm |
| RC (遥控) | — | 切换模式 | — | — |

> 遥控命令经 UART 调试口 (当前版本遥测解析已禁用, 保留接口):
> `FWD/BWD/CW/CCW/REL` 按住式, `TL90/TL180/TR90/TR180` 原地转指定角度。

## 计时规则 (`main.c` updateCarTimer)

- 模式启动且轮速 > 20 mm/s 时开始计时
- 停止且轮速 ≤ 20 mm/s 持续 200 ms 后冻结计时
- OLED 以 0.1 s 精度显示 (非滚球模式在 y=48, 滚球模式在 y=56)
