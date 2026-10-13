/* UART1/PB6/PB7 is now assigned to UART_Camera. */

 /* UART1/PB6/PB7 is now assigned to UART_Camera. */
/**
 * ============================================================================
 * @file    esp32_wifi.c
 * @brief   ESP32-WiFi 图传模块驱动实现 — UART1 (PB6=TX, PB7=RX)
 *
 * 指令协议 (MCU -> 模块, ASCII, 无回车换行):
 *   wifi_mode:<0/1/2>   WiFi模式 AP/STA/AP+STA (模块断电复位后生效)
 *   sta_ssid:<ssid>     STA名称        sta_pd:<pwd>    STA密码
 *   ap_ssid:<ssid>      AP热点名称     ap_pd:<pwd>     AP热点密码
 *   ai_mode:<0..5>      AI模式         sta_ip/ap_ip    查询IP
 *   wifi_ver            查询固件版本
 *
 * 回传协议 (模块 -> MCU):
 *   查询应答: 以 \r\n 结尾的文本 (如 "ap_ip:192.168.4.1\r\n")
 *   AI检测:   $lx,ly,rx,ry,#   (各3位十进制, 共16字符+帧头帧尾)
 *   二维码:   $<文本>#
 *   人脸识别: $lx,ly,rx,ry,#@ID:<n>!
 *   透传数据: $<文本>#         (Nornal_AI模式下APP下发的TCP数据)
 * ============================================================================
 */

#include "drivers/esp32_camera_link.h"
#include "drivers/delay.h"

#include <stdio.h>
#include <string.h>

/* ======================== 模块状态 ======================== */
static AI_mode      s_runmode    = Nornal_AI;
static QR_AI_Msg    s_qrMsg;
static ESP32_AI_Msg s_aiMsg;
static uint8_t      s_newData    = 0U;      /* 1: 收到完整AI/QR数据帧 */

static char         s_apIp[24]  = "";       /* 最近一次查询到的AP IP */
static char         s_staIp[24] = "";       /* 最近一次查询到的STA IP */

/* 接收状态机 (官方逻辑): 0=AI解析 1=查询应答 2=透传 3=人脸 4=二维码 */
static uint8_t      s_cmdFlag   = 0U;
static uint8_t      s_recvBuf[50];
static uint8_t      s_dataBuf[50];
static uint8_t      s_index     = 0U;
static uint8_t      s_endFlag   = 0U;
static uint8_t      s_frameFlag = 0U;

/* ======================== 串口底层发送 (短指令, 阻塞可接受) ======================== */
static void espSendChar(char ch)
{
    while (DL_UART_isBusy(ESP_UART_INST) == true) {
    }
    DL_UART_Main_transmitData(ESP_UART_INST, (uint8_t)ch);
}

static void espSendString(const char *str)
{
    while (*str != '\0') {
        espSendChar(*str++);
    }
}

/* 发送一条格式化指令并等待生效 */
static void espSendCmd(const char *cmd)
{
    espSendString(cmd);
    Delay_ms(ESP_COMMAND_SETTLE_MS);
}

/* ======================== 初始化 (仅UART1, 手动配置不依赖SysConfig) ======================== */
void ESP32Wifi_Init(void)
{
    /* ---- 1. 使能UART1时钟 ---- */
    DL_UART_Main_reset(ESP_UART_INST);
    DL_UART_Main_enablePower(ESP_UART_INST);

    /* ---- 2. 配置引脚复用: PB6=UART1_TX, PB7=UART1_RX ---- */
    DL_GPIO_initPeripheralOutputFunction(ESP_TX_IOMUX, ESP_TX_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(ESP_RX_IOMUX, ESP_RX_IOMUX_FUNC);

    /* ---- 3. UART1时钟: BUSCLK ---- */
    DL_UART_Main_ClockConfig clkCfg = {
        .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
    };
    DL_UART_Main_setClockConfig(ESP_UART_INST, &clkCfg);

    /* ---- 4. 115200, 8N1, 无流控 ---- */
    DL_UART_Main_Config cfg = {
        .mode        = DL_UART_MAIN_MODE_NORMAL,
        .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity      = DL_UART_MAIN_PARITY_NONE,
        .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
    };
    DL_UART_Main_init(ESP_UART_INST, &cfg);
    DL_UART_Main_setOversampling(ESP_UART_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(ESP_UART_INST, ESP_BAUD_IBRD, ESP_BAUD_FBRD);

    /* ---- 5. 使能RX中断 (优先级3, 低于编码器1/无线2) ---- */
    DL_UART_Main_enableInterrupt(ESP_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_SetPriority(ESP_UART_IRQN, 3U);
    NVIC_ClearPendingIRQ(ESP_UART_IRQN);
    NVIC_EnableIRQ(ESP_UART_IRQN);

    /* ---- 6. 使能UART1 ---- */
    DL_UART_Main_enable(ESP_UART_INST);
}

/* ======================== 模块配置指令 ======================== */
void ESP32Wifi_SetMode(void)
{
    char buf[24];
    (void)sprintf(buf, "wifi_mode:%c", ESP_WIFI_MODE);
    espSendCmd(buf);
}

void ESP32Wifi_SetSta(void)
{
    char buf[40];
    (void)sprintf(buf, "sta_ssid:%s", ESP_STA_SSID);
    espSendCmd(buf);
    (void)sprintf(buf, "sta_pd:%s", ESP_STA_PD);
    espSendCmd(buf);
}

void ESP32Wifi_SetAp(void)
{
    char buf[40];
    (void)sprintf(buf, "ap_ssid:%s", ESP_AP_SSID);
    espSendCmd(buf);
    (void)sprintf(buf, "ap_pd:%s", ESP_AP_PD);
    espSendCmd(buf);
}

void ESP32Wifi_SetAiMode(AI_mode mode)
{
    char buf[24];

    s_runmode = mode;
    (void)sprintf(buf, "ai_mode:%d", (int)mode);
    espSendCmd(buf);

    /* 切换接收解析状态机 (官方 mode_chage 逻辑) */
    if (mode == Nornal_AI) {
        s_cmdFlag = 2U;             /* 网络透传数据 */
    } else if (mode == REFACE_AI) {
        s_cmdFlag = 3U;             /* 人脸识别 */
    } else if (mode == QR_AI) {
        s_cmdFlag = 4U;             /* 二维码 */
    } else {
        s_cmdFlag = 0U;             /* 其它AI模式 (目标框) */
    }
}

void ESP32Wifi_QueryStaIp(void)
{
    espSendString("sta_ip");
    s_cmdFlag = 1U;
}

void ESP32Wifi_QueryApIp(void)
{
    espSendString("ap_ip");
    s_cmdFlag = 1U;
}

void ESP32Wifi_QueryVersion(void)
{
    espSendString("wifi_ver");
    s_cmdFlag = 1U;
}

void ESP32Wifi_QuickStart(void)
{
    /* 关键时序: 模块上电后约需2s才能接收串口配置 (官方例程 delay_ms(2000)),
     * 等待不足会导致所有配置指令丢失, 热点保持出厂状态 */
    Delay_ms(ESP_BOOT_WAIT_MS);

    ESP32Wifi_SetMode();
    ESP32Wifi_SetSta();
    ESP32Wifi_SetAp();
    ESP32Wifi_SetAiMode(ESP_AI_MODE_DEFAULT);

    /* 查询AP IP验证通信; 无应答则重发AP配置再查, 最多3轮
     * (覆盖模块启动比预期更慢的情况) */
    for (uint8_t attempt = 0U; attempt < 3U; attempt++) {
        ESP32Wifi_QueryApIp();
        Delay_ms(150U);
        if (s_apIp[0] != '\0') {
            break;
        }
        ESP32Wifi_SetAp();
        Delay_ms(500U);
    }

    ESP32Wifi_QueryStaIp();
    Delay_ms(120U);

    /* 查询结束后恢复AI模式对应的数据解析状态 */
    ESP32Wifi_SetAiMode(s_runmode);
}

/* ======================== 结果读取 ======================== */
uint8_t ESP32Wifi_HasNewData(void)          { return s_newData; }
void    ESP32Wifi_ClearNewData(void)        { s_newData = 0U; }
AI_mode ESP32Wifi_GetAiMode(void)           { return s_runmode; }
const ESP32_AI_Msg *ESP32Wifi_GetAiMsg(void){ return &s_aiMsg; }
const QR_AI_Msg    *ESP32Wifi_GetQrMsg(void){ return &s_qrMsg; }
const char *ESP32Wifi_GetApIp(void)         { return s_apIp; }
const char *ESP32Wifi_GetStaIp(void)        { return s_staIp; }

/* ======================== 接收解析 (官方状态机, 去掉uart0调试输出) ======================== */

/* 查询应答处理: 识别 sta_ip/ap_ip/版本 文本并保存 */
static void ipIdentify(uint8_t *data)
{
    if (memcmp(data, "ap_ip", 5U) == 0) {
        (void)snprintf(s_apIp, sizeof(s_apIp), "%s", (char *)data);
    } else if (memcmp(data, "sta_ip", 6U) == 0) {
        (void)snprintf(s_staIp, sizeof(s_staIp), "%s", (char *)data);
    } else {
        /* 版本信息等, 不需要保存 */
    }
    memset(s_dataBuf, 0, sizeof(s_dataBuf));
}

/* 解析 $lx,ly,rx,ry,# 目标框数据 (16字符, 4个逗号) */
static void parseAiMsg(char *buf)
{
    uint8_t commas = 0U;

    if (strlen(buf) != 16U) {
        return;
    }
    if ((buf[3] != ',') || (buf[7] != ',') || (buf[11] != ',') ||
        (buf[15] != ',')) {
        return;
    }
    for (uint8_t i = 0U; i < 16U; i++) {
        if (buf[i] == ',') {
            commas++;
        }
    }
    if (commas != 4U) {
        return;
    }

    s_aiMsg.lx = (int16_t)((buf[0] - '0') * 100 + (buf[1] - '0') * 10 +
                           (buf[2] - '0'));
    s_aiMsg.ly = (int16_t)((buf[4] - '0') * 100 + (buf[5] - '0') * 10 +
                           (buf[6] - '0'));
    s_aiMsg.rx = (int16_t)((buf[8] - '0') * 100 + (buf[9] - '0') * 10 +
                           (buf[10] - '0'));
    s_aiMsg.ry = (int16_t)((buf[12] - '0') * 100 + (buf[13] - '0') * 10 +
                           (buf[14] - '0'));

    if ((s_aiMsg.lx > s_aiMsg.rx) || (s_aiMsg.ly > s_aiMsg.ry)) {
        return;                     /* 坐标非法 */
    }

    s_aiMsg.cx   = (int16_t)((s_aiMsg.rx - s_aiMsg.lx) / 2 + s_aiMsg.lx);
    s_aiMsg.cy   = (int16_t)((s_aiMsg.ry - s_aiMsg.ly) / 2 + s_aiMsg.ly);
    s_aiMsg.area = (uint16_t)((s_aiMsg.rx - s_aiMsg.lx) / 10 *
                              (s_aiMsg.ry - s_aiMsg.ly));
}

/* 人脸识别: $lx,ly,rx,ry,#@ID:<n>! -> buf 已去掉帧界符 */
static void parseFaceMsg(char *buf)
{
    parseAiMsg(buf);                /* 前16字符与目标框格式一致 */

    if (buf[19] == '-') {           /* id 为负 */
        s_aiMsg.id = (int16_t)(-(buf[20] - '0'));
    } else {
        s_aiMsg.id = (int16_t)(buf[19] - '0');
    }
}

/* 通用 $...# 帧收集: 收到完整帧返回1并把内容拷入s_dataBuf */
static uint8_t collectFrame(char data, char endCh)
{
    if ((data == '$') && (s_frameFlag == 0U)) {
        s_frameFlag = 1U;
        s_index     = 0U;
        memset(s_recvBuf, 0, sizeof(s_recvBuf));
        return 0U;
    }

    if (s_frameFlag == 0U) {
        return 0U;
    }

    if (data == endCh) {            /* 帧尾: 完整一帧 */
        s_frameFlag = 0U;
        memcpy(s_dataBuf, s_recvBuf, sizeof(s_recvBuf));
        memset(s_recvBuf, 0, sizeof(s_recvBuf));
        s_index = 0U;
        return 1U;
    }
    if (data == '$') {              /* 中途丢包重来 */
        s_index = 0U;
        memset(s_recvBuf, 0, sizeof(s_recvBuf));
        return 0U;
    }
    if ((endCh == '!') && ((data == '#') || (data == '@'))) {
        return 0U;                  /* 人脸帧: 去掉 #/@ 只要有效数据 */
    }

    if (s_index < (uint8_t)sizeof(s_recvBuf)) {
        s_recvBuf[s_index++] = (uint8_t)data;
    } else {                        /* 溢出: 丢弃本帧 */
        s_frameFlag = 0U;
        s_index     = 0U;
        memset(s_recvBuf, 0, sizeof(s_recvBuf));
    }
    return 0U;
}

/* 字节分派 (官方 Data_Deal 逻辑) */
static void dataDeal(uint8_t rxData)
{
    if (s_cmdFlag == 1U) {
        /* 查询应答: 以 \r\n 结束 */
        s_recvBuf[s_index] = rxData;
        if (rxData == 0x0DU) {
            s_endFlag = 1U;
        }
        if ((s_endFlag == 1U) && (rxData == 0x0AU)) {
            s_cmdFlag = 0U;
            s_endFlag = 0U;
            memcpy(s_dataBuf, s_recvBuf, s_index);
            s_dataBuf[s_index] = '\0';
            memset(s_recvBuf, 0, sizeof(s_recvBuf));
            s_index = 0U;
            ipIdentify(s_dataBuf);
        } else if (s_index < (uint8_t)(sizeof(s_recvBuf) - 1U)) {
            s_index++;
        } else {
            s_index = 0U;           /* 溢出保护 */
        }
    } else if (s_cmdFlag == 2U) {
        /* 网络透传 $...#: 本工程暂不处理内容, 仅维持帧同步 */
        (void)collectFrame((char)rxData, '#');
    } else if (s_cmdFlag == 3U) {
        /* 人脸识别 $...! */
        if (collectFrame((char)rxData, '!') != 0U) {
            parseFaceMsg((char *)s_dataBuf);
            memset(s_dataBuf, 0, sizeof(s_dataBuf));
            s_newData = 1U;
        }
    } else if (s_cmdFlag == 4U) {
        /* 二维码 $...# */
        if (collectFrame((char)rxData, '#') != 0U) {
            memcpy(s_qrMsg.QR_msg, s_dataBuf, sizeof(s_qrMsg.QR_msg));
            s_qrMsg.QR_msg[sizeof(s_qrMsg.QR_msg) - 1U] = '\0';
            memset(s_dataBuf, 0, sizeof(s_dataBuf));
            s_newData = 1U;
        }
    } else {
        /* 其它AI模式 $lx,ly,rx,ry,# */
        if (collectFrame((char)rxData, '#') != 0U) {
            parseAiMsg((char *)s_dataBuf);
            memset(s_dataBuf, 0, sizeof(s_dataBuf));
            s_newData = 1U;
        }
    }
}

/* ======================== UART1 中断服务函数 ======================== */
void UART0_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(ESP_UART_INST)) {
    case DL_UART_IIDX_RX:
        while (DL_UART_Main_isRXFIFOEmpty(ESP_UART_INST) == false) {
            uint8_t data = DL_UART_Main_receiveData(ESP_UART_INST);
            dataDeal(data);
        }
        break;

    default:
        break;
    }
}

