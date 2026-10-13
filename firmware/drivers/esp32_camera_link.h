/**
 * ============================================================================
 * @file    esp32_camera_link.h
 * @brief   ESP32-WiFi 图传模块驱动 — UART1 (PB6=TX, PB7=RX), 115200 8N1
 *
 * 移植自官方 05_UART 例程 (Yahboom ESP32-WiFi 图传模块),
 * 按当前工程风格改为自包含模块:
 *   - UART0 + PA10/PA11 手动配置 (不依赖 SysConfig)
 *   - 延时使用工程 Delay.h 的 Delay_ms
 *   - 去掉官方代码中对 uart0 调试串口的依赖, 查询结果存入内部缓冲供读取
 *
 * 模块功能:
 *   上电后通过串口指令配置模块的 WiFi 模式 (AP/STA/AP+STA) 与 AI 模式,
 *   之后手机/PC 连上模块热点 (或同一路由), 打开上位机 APP 即可看到
 *   摄像头画面 (图传)。MCU 侧还可解析模块回传的 AI 检测数据
 *   ($lx,ly,rx,ry,# 目标框 / $...# 二维码 / $...! 人脸识别)。
 *
 * 使用方法:
 *   1. SYSCFG_DL_init() 之后调用 ESP32Wifi_Init()  (仅初始化UART1);
 *   2. 调用 ESP32Wifi_QuickStart() 完成模块配置 (约2s, 含AP热点建立);
 *   3. OLED/串口读取 ESP32Wifi_GetApIp() 获得热点IP, 手机连接后打开APP;
 *   4. 如需AI识别, ESP32Wifi_SetAiMode() 切换模式, 轮询
 *      ESP32Wifi_HasNewData() 后读取对应数据结构体。
 * ============================================================================
 */
#ifndef __ESP32_CAMERA_LINK_H
#define __ESP32_CAMERA_LINK_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ======================== 硬件参数 (UART0: PA10=TX, PA11=RX) ======================== */
#define ESP_UART_INST            UART0
#define ESP_UART_IRQN            UART0_INT_IRQn
#define ESP_TX_IOMUX             IOMUX_PINCM21              /* PA10 = UART0_TX */
#define ESP_TX_IOMUX_FUNC        IOMUX_PINCM21_PF_UART0_TX
#define ESP_RX_IOMUX             IOMUX_PINCM22              /* PA11 = UART0_RX */
#define ESP_RX_IOMUX_FUNC        IOMUX_PINCM22_PF_UART0_RX

/* 波特率: 115200 @BUSCLK 32MHz */
#define ESP_BAUD_IBRD            17U
#define ESP_BAUD_FBRD            23U

/* ======================== WiFi 配置 (按需修改) ======================== */
/* WiFi模式: 0=AP, 1=STA, 2=AP+STA共存 (模块断电复位后生效) */
#define ESP_WIFI_MODE            '2'
/* STA模式: 连接已有路由器 */
#define ESP_STA_SSID             "Yahboom2"
#define ESP_STA_PD               "yahboom890729"
/* AP模式: 模块自身热点 (手机直连看图传) */
#define ESP_AP_SSID              "ESP32_WIFI_TEST"
#define ESP_AP_PD                ""        /* 空=无密码 */

/* 上电默认AI模式: Nornal_AI=纯图传不检测 */
#define ESP_AI_MODE_DEFAULT      Nornal_AI

/* 每条指令后的配置生效等待 (官方值300ms) */
#define ESP_COMMAND_SETTLE_MS    (300U)

/* 模块上电启动等待: 官方例程为2000ms!
 * 等待不足时模块尚未就绪, 所有串口配置指令全部丢失,
 * 热点保持出厂状态 (搜不到ESP32_WIFI_TEST的最常见原因) */
#define ESP_BOOT_WAIT_MS         (2000U)

/* ======================== AI 数据协议 ======================== */
typedef struct {
    int16_t  lx;    /* 左上角 */
    int16_t  ly;
    int16_t  rx;    /* 右下角 */
    int16_t  ry;
    int16_t  cx;    /* 中心点 */
    int16_t  cy;
    uint16_t area;  /* 面积 (宽/10 × 高) */
    int16_t  id;    /* 人脸id (仅人脸识别模式有效) */
} ESP32_AI_Msg;

typedef enum {
    Nornal_AI = 0,  /* 不检测 (纯图传) */
    Cat_Dog_AI,     /* 猫狗检测 */
    FACE_AI,        /* 人脸检测 */
    COLOR_AI,       /* 颜色检测 */
    REFACE_AI,      /* 人脸识别 */
    QR_AI     = 5,  /* 二维码识别 */
    AI_MAX
} AI_mode;

typedef struct {
    char QR_msg[50];    /* 二维码文本 */
} QR_AI_Msg;

/* ======================== API 函数 ======================== */

/**
 * @brief 初始化 UART1 (PB6/PB7, 115200) 并使能接收中断
 * @note  只初始化串口, 不触碰模块; SYSCFG_DL_init() 之后调用
 */
void ESP32Wifi_Init(void);

/**
 * @brief 模块快速配置: WiFi模式 + STA + AP + AI模式 + 查询AP/STA IP
 * @note  阻塞约2秒 (指令间需等模块生效), 建议上电初始化阶段调用一次;
 *        修改WiFi模式需模块断电复位后重新配置才生效
 */
void ESP32Wifi_QuickStart(void);

/** @brief 设置WiFi模式 (wifi_mode:0/1/2, 需模块断电复位后生效) */
void ESP32Wifi_SetMode(void);

/** @brief 配置STA (连接路由器的ssid/密码) */
void ESP32Wifi_SetSta(void);

/** @brief 配置AP (模块自身热点的ssid/密码) */
void ESP32Wifi_SetAp(void);

/** @brief 设置AI模式, 自动切换接收解析状态机 */
void ESP32Wifi_SetAiMode(AI_mode mode);

/** @brief 查询STA的IP (结果经ISR收齐后由 ESP32Wifi_GetStaIp 读取) */
void ESP32Wifi_QueryStaIp(void);

/** @brief 查询AP热点的IP */
void ESP32Wifi_QueryApIp(void);

/** @brief 查询模块固件版本 */
void ESP32Wifi_QueryVersion(void);

/* ---- 结果读取 ---- */

/** @brief 是否有新的AI/二维码数据 (读后可用 Clear 清零) */
uint8_t ESP32Wifi_HasNewData(void);

/** @brief 清除新数据标志 */
void ESP32Wifi_ClearNewData(void);

/** @brief 获取当前AI模式 */
AI_mode ESP32Wifi_GetAiMode(void);

/** @brief 获取AI检测数据 (目标框/中心点/面积/人脸id) */
const ESP32_AI_Msg *ESP32Wifi_GetAiMsg(void);

/** @brief 获取二维码文本 */
const QR_AI_Msg *ESP32Wifi_GetQrMsg(void);

/** @brief 获取AP热点IP字符串 (如 "ap_ip:192.168.4.1", 未查询到为空串) */
const char *ESP32Wifi_GetApIp(void);

/** @brief 获取STA的IP字符串 (未查询到为空串) */
const char *ESP32Wifi_GetStaIp(void);
#endif /* __ESP32_WIFI_H */
