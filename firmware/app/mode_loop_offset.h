#ifndef __MODE_LOOP_OFFSET_H
#define __MODE_LOOP_OFFSET_H

#include <stdint.h>
#include "drivers/line_tracker.h"

/*
 * Mode 006(1): 基于006, 小球固定目标+70mm(7cm齿轮侧)
 *
 * State flow:
 *   IDLE -> HOLD -> DRIVE (arc trajectory 0..5600mm)
 *        -> TRACK (extra 1300mm line tracking)
 *        -> FINAL (ball settle) -> DONE
 *
 * 与006区别: 目标位置固定+70mm, 启动前馈向齿轮侧倾斜
 */

typedef enum {
    MODE0061_IDLE = 0,
    MODE0061_HOLD,
    MODE0061_DRIVE,
    MODE0061_TRACK,
    MODE0061_FINAL,
    MODE0061_DONE
} Mode0061_State;

void Mode0061_Init(void);
void Mode0061_OnStartStop(void);
void Mode0061_Stop(void);
void Mode0061_Update(const LineTracker_State *line, int8_t *left, int8_t *right);

uint8_t Mode0061_IsRunning(void);
Mode0061_State Mode0061_GetState(void);
float Mode0061_GetTargetMm(void);
float Mode0061_GetDistanceMm(void);
float Mode0061_GetFinishMm(void);
int8_t Mode0061_GetDrivePercent(void);

#endif
