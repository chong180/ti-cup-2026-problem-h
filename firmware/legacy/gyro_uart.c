/**
 * @file    gyro.c
 * @brief   单轴Z轴陀螺仪驱动 — 0x5A / 5字节帧协议实现
 *
 * 帧结构: [0x5A][TYPE][DATAL][DATAH][SUMCRC]
 *   SUMCRC = (0x5A + TYPE + DATAL + DATAH) & 0xFF
 *   raw    = (int16_t)((DATAH << 8) | DATAL)  // 小端
 *   Yaw    = raw / 32768.0 * 180.0            // 度
 */
#include "ti_msp_dl_config.h"
#include "legacy/gyro_uart.h"

/* ============================================================
 *  全局变量
 * ============================================================ */
volatile float    gyro_yaw         = 0.0f;
volatile uint8_t  gyro_rx_done     = 0;
volatile uint32_t gyro_frame_count = 0;
volatile uint32_t gyro_crc_error   = 0;

/* ============================================================
 *  帧解析状态机
 * ============================================================ */
typedef enum {
    GYRO_STATE_WAIT_HEADER,   /* 等待 0x5A          */
    GYRO_STATE_RECV_TYPE,     /* 接收 TYPE           */
    GYRO_STATE_RECV_DATAL,    /* 接收 DATAL (低字节) */
    GYRO_STATE_RECV_DATAH,    /* 接收 DATAH (高字节) */
    GYRO_STATE_RECV_SUMCRC    /* 接收 SUMCRC         */
} GyroRxState;

static volatile GyroRxState s_rxState = GYRO_STATE_WAIT_HEADER;

/* 当前帧缓存 (5字节) */
static uint8_t s_frame[GYRO_FRAME_LEN];

/**
 * @brief  校验并解析一帧 5 字节数据
 *         s_frame[0]=0x5A [1]=TYPE [2]=DATAL [3]=DATAH [4]=SUMCRC
 */
static void Gyro_ParseFrame(void)
{
    uint8_t sum;

    /* 和校验: byte[0]+byte[1]+byte[2]+byte[3], 取低8位 */
    sum = (uint8_t)((s_frame[0] + s_frame[1] + s_frame[2] + s_frame[3]) & 0xFF);

    if (sum != s_frame[4]) {
        /* 校验失败 */
        gyro_crc_error++;
        return;
    }

    /* 仅解析 Yaw 航向角帧 (TYPE=0xBB) */
    if (s_frame[1] == GYRO_TYPE_YAW) {
        /* 小端拼接: DATAL 在前(低字节), DATAH 在后(高字节) */
        int16_t raw = (int16_t)(((uint16_t)s_frame[3] << 8) | s_frame[2]);

        /* 角度换算: raw / 32768.0 * 180.0 */
        gyro_yaw = (float)raw / 32768.0f * 180.0f;

        gyro_rx_done = 1;
        gyro_frame_count++;
    }
}

/* ============================================================
 *  UART 接收中断服务函数
 *  使用逐字节状态机实现帧同步, 5字节一帧
 * ============================================================ */
void UART_GYRO_INST_IRQHandler(void)
{
    uint8_t data = DL_UART_Main_receiveData(UART_GYRO_INST);

    switch (s_rxState) {
    case GYRO_STATE_WAIT_HEADER:
        if (data == GYRO_FRAME_HEADER) {
            s_frame[0] = data;
            s_rxState  = GYRO_STATE_RECV_TYPE;
        }
        break;

    case GYRO_STATE_RECV_TYPE:
        s_frame[1] = data;
        s_rxState  = GYRO_STATE_RECV_DATAL;
        break;

    case GYRO_STATE_RECV_DATAL:
        s_frame[2] = data;   /* 低字节 */
        s_rxState  = GYRO_STATE_RECV_DATAH;
        break;

    case GYRO_STATE_RECV_DATAH:
        s_frame[3] = data;   /* 高字节 */
        s_rxState  = GYRO_STATE_RECV_SUMCRC;
        break;

    case GYRO_STATE_RECV_SUMCRC:
        s_frame[4] = data;   /* SUMCRC */
        Gyro_ParseFrame();
        s_rxState = GYRO_STATE_WAIT_HEADER;
        break;

    default:
        s_rxState = GYRO_STATE_WAIT_HEADER;
        break;
    }
}

/* ============================================================
 *  内部工具函数
 * ============================================================ */

/** 通过 UART 阻塞发送命令帧 */
static void Gyro_SendCmd(const uint8_t *buf, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++) {
        DL_UART_Main_transmitDataBlocking(UART_GYRO_INST, buf[i]);
    }
}

/* ============================================================
 *  初始化
 * ============================================================ */
void Gyro_Init(void)
{
    /* 使能 UART RX 中断 (外设级 + NVIC 级) */
    DL_UART_Main_enableInterrupt(UART_GYRO_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_SetPriority(UART_GYRO_INST_INT_IRQN, 2U);
    NVIC_EnableIRQ(UART_GYRO_INST_INT_IRQN);
}

/* ============================================================
 *  数据获取接口
 * ============================================================ */
float Gyro_GetYaw(void)
{
    return gyro_yaw;
}

float Gyro_GetAngleDeg(void)
{
    return gyro_yaw;
}

/* ============================================================
 *  命令发送函数
 * ============================================================ */

/**
 * 解锁指令: [0x5A][0x50][SUMCRC]
 * SUMCRC = (0x5A + 0x50) & 0xFF = 0xAA
 */
void Gyro_Unlock(void)
{
    uint8_t cmd[3] = { GYRO_FRAME_HEADER, GYRO_CMD_UNLOCK, 0 };
    cmd[2] = (uint8_t)((cmd[0] + cmd[1]) & 0xFF);
    Gyro_SendCmd(cmd, 3);
}

/**
 * Yaw 航向归零: [0x5A][0x51][SUMCRC]
 * SUMCRC = (0x5A + 0x51) & 0xFF = 0xAB
 */
void Gyro_HeadingZero(void)
{
    uint8_t cmd[3] = { GYRO_FRAME_HEADER, GYRO_CMD_ZERO, 0 };
    cmd[2] = (uint8_t)((cmd[0] + cmd[1]) & 0xFF);
    Gyro_SendCmd(cmd, 3);
}

/**
 * 自动零偏校准: [0x5A][0x52][SUMCRC]
 * SUMCRC = (0x5A + 0x52) & 0xFF = 0xAC
 * @note 校准期间需保持模块静止
 */
void Gyro_AutoCalib(void)
{
    uint8_t cmd[3] = { GYRO_FRAME_HEADER, GYRO_CMD_CALIB, 0 };
    cmd[2] = (uint8_t)((cmd[0] + cmd[1]) & 0xFF);
    Gyro_SendCmd(cmd, 3);
}

/**
 * 修改输出速率: [0x5A][0x53][RATE][SUMCRC]
 * SUMCRC = (0x5A + 0x53 + RATE) & 0xFF
 * @param rateHz 输出频率 (Hz)
 */
void Gyro_SetRate(uint8_t rateHz)
{
    uint8_t cmd[4];
    cmd[0] = GYRO_FRAME_HEADER;
    cmd[1] = GYRO_CMD_SET_RATE;
    cmd[2] = rateHz;
    cmd[3] = (uint8_t)((cmd[0] + cmd[1] + cmd[2]) & 0xFF);
    Gyro_SendCmd(cmd, 4);
}
