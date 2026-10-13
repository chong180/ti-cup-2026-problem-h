#ifndef __MODE_ARC_TRAJECTORY_H
#define __MODE_ARC_TRAJECTORY_H

#include <stdint.h>
#include "drivers/line_tracker.h"

typedef enum {
    MODE6_IDLE = 0,
    MODE6_HOLD,
    MODE6_DRIVE,
    MODE6_FINAL,
    MODE6_DONE
} Mode6_State;

void Mode6_Init(void);
void Mode6_OnStartStop(void);
void Mode6_Stop(void);
void Mode6_Update(const LineTracker_State *line, int8_t *left, int8_t *right);

uint8_t Mode6_IsRunning(void);
Mode6_State Mode6_GetState(void);
float Mode6_GetTargetMm(void);

#endif
