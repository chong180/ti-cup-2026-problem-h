# 控制算法说明

## 1. 系统时序架构

```
SysTick (1 ms, 优先级 1)
   └─ 每 5 ms 发布 g_ballControlTick 标志 (只置位, 不做计算)

主循环 while(1):
   ├─ Timebase_TakeBallControlTick() → BallBalance_Update()      (最高优先级任务)
   │     └─ 视觉坐标低通 → PD → 摆杆目标角度 → Emm_V5 串口发送
   ├─ Key_GetEvent()            → 模式切换 / 启停 / 调参
   ├─ EncoderDistance_Update()  → 10 ms 窗口测速 + 里程累计
   ├─ LineTracker_Read()        → 8 路灰度采样
   ├─ 按模式执行控制 (循迹 / AB / 圆弧轨迹 / 遥控 …)
   ├─ updateCarTimer()          → 行驶计时 (启动到停稳)
   ├─ updateDisplay()           → OLED 刷新 (100 ms 周期)
   └─ Delay_ms(1)
```

相机 UART RX 中断优先级 0 (高于 SysTick), 只负责收字节入环形缓冲,
不进行控制计算, 避免中断阻塞。

## 2. 循迹控制 (`main.c` 内联 + `drivers/encoder_*.c`)

**误差模型**: 8 路灰度按权重 (−35 … +35) 加权, 去掉中心传感器 ±5 的固有偏移,
进入 ±5 死区。

```
controlError = error − sign(error) × 5        (死区补偿)
```

**一阶自适应滤波**: 大误差 (|e| ≥ 10) 用时常数 8 ms, 小误差用 25 ms,
兼顾弯道响应与直道平稳。

**P 校正 + 丢线回拐**:

```
correction = Kp × filteredError              (Kp 默认 0.60, 可 S3/S4 或串口调节)
线偏右 → correction > 0 → 左轮加速 + 右轮减速 → 车右转
丢线   → 按上次误差方向施加固定校正量 (35%) 回拐找线
```

**轮速闭环** (PI + 前馈, 10 ms 周期):

```
target    = desiredPWM × 500 mm/s / 42            (PWM→速度前馈标定)
feedforward = desiredPWM × 500 / (左 450 / 右 560)  (左右轮速度差异补偿)
output    = feedforward + Kp×err + Ki×∫err         (Kp 0.010, Ki 0.030)
限幅: 24% ~ 60% (内轮不低于死区, 外轮保留差速余量)
```

**定距停车**: 编码器平均里程达 5.63 m (可 S3/S4 ±0.5 cm 微调) 提前断电,
利用约 14 cm 滑行距离停在基准线, 末段 20 cm 线性降速至 25%。

## 3. 滚球平衡控制 (`ball_balance/`)

### 3.1 视觉接口 (`ball_balance.c`)

- 协议 `#±xx.xx!` (cm) → ×10 转 mm; `#N!` 表示丢球
- 位置一阶低通滤波; 球速由连续 3 个有效帧差分求得
- 超过 300 ms 无有效数据 → 目标角度回 0° (保护)

### 3.2 PD 中心稳定 (第三问 / AB 模式底盘闭环)

```
angleCmd = Kp × (target − position) + Kd × (0 − velocity)
默认 Kp = 0.480, Kd = 0.003, 角度安全限幅 ±30°
电机位置模式 300 RPM; 5 ms 周期最大步进 0.2° (换向 0.35°)
```

### 3.3 串级 PID 回零 (`ball_zero_control.c`, MODE:ZERO)

```
位置外环: targetVelocity = Kp_pos × posErr + Kd_pos × d(posErr)/dt   (Kp 0.80, Kd 0.10)
速度内环: platformAngle  = Kp_vel × velErr + Kd_vel × d(velErr)/dt   (Kp 0.040, Kd 0.0005)
角度限幅 ±4°, 每 5 ms 最多变化 0.20°
```

### 3.4 高频预测 PD (`ball_hf_zero.c`, MODE:HFZERO)

在 PD 基础上增加速度预测项与静止振荡模式, 用于圆弧低速段抑制球的前后晃动;
位置增益可 S3/S4 在线调节 (步进 0.02)。

### 3.5 轨迹模式 (`ball_route.c` / `ball_route_ramp.c`, 第三问)

- `ball_route.c`: 开环时序 O → +5 cm → −5 cm (阶跃目标, 到位快速换向)
- `ball_route_ramp.c`: 目标位置按线性三角波 0 → +5 → −5 cm 连续扫描, 避免过冲
- 均带角度斜坡限速与末端保持

## 4. 整圈轨迹模式 (`app/`)

以 `mode_loop_arc.c` (MODE:006) 为例, 编码器里程四段规划:

| 段 | 里程 (mm) | 占空比 | 说明 |
| --- | --- | --- | --- |
| 起步加速 | 0 → 260 | 12% → 34% | 3 s 渐进加速, 防球前冲 |
| 直道 S1 | → 1373 | 34% | 循迹 Kp 0.45 |
| 圆弧 A1 | → 2812 | 27% | 差速 +2.6 偏置 |
| 直道 S2 | → 4186 | 34% | |
| 圆弧 A2 | → 5600 | 27% | 回到 A 点 |
| 直线循迹 | → 6900 | 27% | 过 A 点后追加 1.3 m 直线循迹 |
| 末端平衡 | — | 16% | 球稳定 1.2 s |

- 总里程按 5600/6140 标定系数缩放 (编码器实测与理论弧长差异)
- 起步/制动时给摆杆叠加前馈角度 (+2.2° / −1.2°), 抑制球惯性滑动
- `mode_loop_offset.c` (006(1)) 将球固定稳定在 +70 mm 偏置 (第六问任意位置)

## 5. MATLAB 辅助调参 (`tools/`)

- `pid_relay_tuner.m`: Åström-Hägglund 继电反馈自整定。用 bang-bang 替代 PID
  产生极限环, 测振荡周期 Tu 与振幅 a, 计算 Ku = 4d/(πa),
  Ziegler-Nichols 整定 Kp = 0.6Ku, Ki = 1.2Ku/Tu, Kd = 0.075Ku·Tu
- `ab20_tune.m`: AB20 模式 (A→B 段) 速度曲线与球稳定参数离线仿真调参
