# vision/ — MaixCAM2 相机端 (钢球检测与坐标回传)

摆杆凹槽正上方的摄像头实时检测钢球, 将坐标 (cm) 通过 UART 回传主控 MSPM0G3507。

## 硬件

- **MaixCAM2** (Sg2002 + 自带 NPU), 摄像头朝向摆杆凹槽, 画面覆盖整根 25 cm 摆杆
- UART4: A21 = TX → MSPM0 PB7 (UART3_RX), 115200 8N1
- 补光灯: B3 (高有效), 环境光不足时自动补光
- 所有设备与主控共地

## 文件

| 文件 | 说明 |
| --- | --- |
| `main.py` | MaixCAM 应用主程序: YOLO11 推理 → 像素→厘米标定 → 串口回传 |
| `app.yaml` | MaixCAM 应用打包清单 (`maix-steel-v1.0.1.zip`) |
| `models/best_yolo11.mud` | YOLO11 模型 (MaixCAM 格式) |
| `models/best_yolo11_npu.axmodel` | NPU 部署模型 |
| `models/best_yolo11_vnpu.axmodel` | 虚拟 NPU 部署模型 |
| `dist/maix-steel-v1.0.1.zip` | 打包好的应用, 可直接刷入 MaixCAM |

## 串口协议 (相机 → MCU)

```text
#-12.5!      钢球坐标 −12.5 cm (有效范围 ±12.5 cm)
#+3.25!      钢球坐标 +3.25 cm
#N!          本帧未检测到钢球 (MCU 超过 300 ms 未收到数据会回零保护)
```

发送周期 50 ms。旧版双坐标协议 `#320.00,300.00!` (像素横/纵坐标) 仍兼容。

## 像素 → 厘米标定

摆杆两端贴有标识点, 两点间距已知 (25 cm), 据此建立像素→厘米线性映射:

```python
ROD_LENGTH_CM = 25.0
MANUAL_CALIBRATION = False      # True: 触摸屏点选两点标定
FIXED_POINT_1 = (55, 252)       # 摆杆左端 (画面坐标)
FIXED_POINT_2 = (639, 172)      # 摆杆右端 (画面坐标)
```

## 部署

1. MaixCAM2 通过 MaixPy 烧录固件 (≥ 4.3.x)；
2. 将 `dist/maix-steel-v1.0.1.zip` 拖入 MaixCAM 磁盘, 或在 MaixPy IDE 中
   直接打开 `main.py` 运行;
3. 上电后 OLED/串口打印 `YOLO11 model loaded` 与标定信息即就绪;
4. 坐标系约定: 零点下方为正, 上方为负 (与主控 `ball_balance.c` 一致)。
