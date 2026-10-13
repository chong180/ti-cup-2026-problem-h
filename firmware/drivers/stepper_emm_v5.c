#include "drivers/stepper_emm_v5.h"

#include "ti_msp_dl_config.h"

static void stepperSend(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    for (i = 0U; i < length; i++) {
        DL_UART_Main_transmitDataBlocking(UART_dianji_INST, data[i]);
    }

    /* Do not return while the last byte is still being shifted out. */
    while (DL_UART_Main_isBusy(UART_dianji_INST)) {
    }
}

/**
 * @brief  将电机的当前位置清零
 * @param  addr 电机地址，通常为 1
 * @note   协议帧：地址 + 0x0A + 0x6D + 校验字节 0x6B
 */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
    const uint8_t cmd[] = {
        addr,   /* 电机地址 */
        0x0AU,  /* 功能码：当前位置清零 */
        0x6DU,  /* 命令数据 */
        0x6BU   /* 校验字节 */
    };
    stepperSend(cmd, (uint16_t) sizeof(cmd));
}

/**
 * @brief  电机使能控制
 * @param  addr  电机地址，通常为 1
 * @param  state true=使能，false=失能
 * @param  snF   多机同步标志；单电机控制时填 false
 * @note   snF=true 时命令先进入等待状态，需要再发送同步运动命令
 */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
    const uint8_t cmd[] = {
        addr,             /* 电机地址 */
        0xF3U,            /* 功能码：使能控制 */
        0xABU,            /* 命令头 */
        (uint8_t) state,  /* 使能状态 */
        (uint8_t) snF,    /* 多机同步标志 */
        0x6BU             /* 校验字节 */
    };
    stepperSend(cmd, (uint16_t) sizeof(cmd));
}

/**
 * @brief  速度模式控制
 * @param  addr 电机地址，通常为 1
 * @param  dir  旋转方向，0/1；机构方向相反时交换这两个值
 * @param  vel  转速，单位 RPM，按高字节在前发送
 * @param  acc  加速度档位；0 表示不使用加减速曲线
 * @param  snF  多机同步标志；单电机控制时填 false
 */
void Emm_V5_Vel_Control(
    uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    const uint8_t cmd[] = {
        addr,                  /* 电机地址 */
        0xF6U,                 /* 功能码：速度模式 */
        dir,                   /* 旋转方向 */
        (uint8_t) (vel >> 8),  /* 转速高字节 */
        (uint8_t) vel,         /* 转速低字节 */
        acc,                   /* 加速度 */
        (uint8_t) snF,         /* 多机同步标志 */
        0x6BU                  /* 校验字节 */
    };
    stepperSend(cmd, (uint16_t) sizeof(cmd));
}

/**
 * @brief  位置模式控制
 * @param  addr 电机地址，通常为 1
 * @param  dir  旋转方向，0/1；可分别对应杆子的上升和下降
 * @param  vel  运行转速，单位 RPM
 * @param  acc  加速度档位；0 表示不使用加减速曲线
 * @param  clk  目标脉冲数，按最高字节在前发送
 * @param  raF  位置模式：0=相对位置，1=绝对位置
 * @param  snF  多机同步标志；单电机控制时填 false
 * @note   杆子的实际位移由电机步距角、细分数、齿轮和齿条参数共同决定。
 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel,
    uint8_t acc, uint32_t clk, uint8_t raF, bool snF)
{
    const uint8_t cmd[] = {
        addr,                   /* 电机地址 */
        0xFDU,                  /* 功能码：位置模式 */
        dir,                    /* 旋转方向 */
        (uint8_t) (vel >> 8),   /* 转速高字节 */
        (uint8_t) vel,          /* 转速低字节 */
        acc,                    /* 加速度 */
        (uint8_t) (clk >> 24),  /* 脉冲数第 4 字节 */
        (uint8_t) (clk >> 16),  /* 脉冲数第 3 字节 */
        (uint8_t) (clk >> 8),   /* 脉冲数第 2 字节 */
        (uint8_t) clk,          /* 脉冲数第 1 字节 */
        raF,                    /* 相对/绝对位置模式 */
        (uint8_t) snF,          /* 多机同步标志 */
        0x6BU                   /* 校验字节 */
    };
    stepperSend(cmd, (uint16_t) sizeof(cmd));
}

/**
 * @brief  立即停止电机
 * @param  addr 电机地址，通常为 1
 * @param  snF  多机同步标志；单电机停止时填 false
 */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
    const uint8_t cmd[] = {
        addr,           /* 电机地址 */
        0xFEU,          /* 功能码：立即停止 */
        0x98U,          /* 命令数据 */
        (uint8_t) snF,  /* 多机同步标志 */
        0x6BU           /* 校验字节 */
    };
    stepperSend(cmd, (uint16_t) sizeof(cmd));
}

/**
 * @brief  触发多电机同步运动
 * @param  addr 电机地址；广播同步时按驱动器协议填写广播地址
 * @note   仅在之前的控制命令中将 snF 设置为 true 时需要调用
 */
void Emm_V5_Synchronous_Motion(uint8_t addr)
{
    const uint8_t cmd[] = {
        addr,   /* 电机地址 */
        0xFFU,  /* 功能码：同步运动 */
        0x66U,  /* 命令数据 */
        0x6BU   /* 校验字节 */
    };
    stepperSend(cmd, (uint16_t) sizeof(cmd));
}
