#ifndef __BALL_ROUTE_RAMP_H
#define __BALL_ROUTE_RAMP_H

#include <stdbool.h>

/*
 * BallRouteNew: 线性目标位置扫描 0→+5cm→-5cm
 *
 * 与 ball_route.c 的阶跃式目标不同, 本驱动让目标位置按线性三角波
 * 连续变化, 球跟随平滑轨迹运动, 避免突变导致过冲.
 *
 * 状态流: IDLE → RAMP_UP(0→+50mm) → RAMP_DOWN(+50→-50mm) → HOLD(-50mm)
 */

typedef enum {
    ROUTE5_NEW_IDLE = 0,
    ROUTE5_NEW_RAMP_UP,     /* 目标 0 → +50mm 线性上升 */
    ROUTE5_NEW_RAMP_DOWN,  /* 目标 +50mm → -50mm 线性下降 */
    ROUTE5_NEW_HOLD,        /* 保持 -50mm */
    ROUTE5_NEW_DONE
} Route5New_State;

void Route5New_Init(void);
void Route5New_Start(void);
void Route5New_Stop(void);
void Route5New_Toggle(void);
void Route5New_Update(void);

bool Route5New_IsRunning(void);
Route5New_State Route5New_GetState(void);
float Route5New_GetTargetMm(void);
float Route5New_GetPositionMm(void);

#endif
