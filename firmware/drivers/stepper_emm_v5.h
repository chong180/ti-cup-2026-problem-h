#ifndef STEPPER_EMM_V5_H
#define STEPPER_EMM_V5_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Emm_V5 串口闭环步进电机驱动。
 *
 * 硬件由 SysConfig 配置：
 *   UART_dianji / UART2 / PB15(TX) / PB16(RX) / 115200 8N1.
 *
 * addr：驱动器中配置的电机地址，通常为 1。
 * dir ：0/1 表示两个旋转方向，可分别用于齿杆上升和下降。
 * snF ：多电机同步启动标志；单电机控制时使用 false。
 * raF ：0=相对位置模式，1=绝对位置模式。
 */

void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);
void Emm_V5_Vel_Control(
    uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel,
    uint8_t acc, uint32_t clk, uint8_t raF, bool snF);
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
void Emm_V5_Synchronous_Motion(uint8_t addr);

#endif
