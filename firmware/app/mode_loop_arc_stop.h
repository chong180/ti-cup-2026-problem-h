#ifndef __MODE_LOOP_ARC_STOP_H
#define __MODE_LOOP_ARC_STOP_H

#include <stdint.h>
#include "drivers/line_tracker.h"

/*
 * Mode 007: mode6 arc trajectory, stop at A point (5600mm).
 * Same as 006 but without the extra track segment.
 */

typedef enum {
    MODE007_IDLE = 0,
    MODE007_HOLD,
    MODE007_DRIVE,
    MODE007_FINAL,
    MODE007_DONE
} Mode007_State;

void Mode007_Init(void);
void Mode007_OnStartStop(void);
void Mode007_Stop(void);
void Mode007_Update(const LineTracker_State *line, int8_t *left, int8_t *right);

uint8_t Mode007_IsRunning(void);
Mode007_State Mode007_GetState(void);
float Mode007_GetTargetMm(void);
float Mode007_GetDistanceMm(void);
float Mode007_GetFinishMm(void);
int8_t Mode007_GetDrivePercent(void);

#endif
