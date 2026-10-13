#ifndef __MODE_AB20_H
#define __MODE_AB20_H

#include <stdint.h>
#include "drivers/line_tracker.h"

typedef enum {
    MODEAB_IDLE = 0,
    MODEAB_WAIT_BALL,
    MODEAB_ACCEL,
    MODEAB_CRUISE,
    MODEAB_DECEL,
    MODEAB_FINAL_BALANCE,
    MODEAB_DONE
} ModeAB_State;

void ModeAB_Init(void);
void ModeAB_OnStartStop(void);
void ModeAB_Stop(void);
void ModeAB_Update(const LineTracker_State *line, int8_t *left, int8_t *right);

uint8_t ModeAB_IsRunning(void);
ModeAB_State ModeAB_GetState(void);
float ModeAB_GetDistanceMm(void);
int8_t ModeAB_GetDrivePercent(void);

#endif
