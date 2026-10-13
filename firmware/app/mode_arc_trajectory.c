#include "app/mode_arc_trajectory.h"


#include "app/sys_mode.h"
#include "ball_balance/ball_hf_zero.h"
#include "ball_balance/ball_balance.h"
#include "drivers/encoder_odometry.h"
#include "drivers/line_tracker.h"
#include "drivers/motor_tb6612.h"
#include "drivers/timebase.h"


#define MODE6_DISTANCE_SCALE  (5600.0f / 6140.0f)
#define MODE6_S1_END_MM  (1500.0f * MODE6_DISTANCE_SCALE)
#define MODE6_A1_END_MM  (3070.0f * MODE6_DISTANCE_SCALE)
#define MODE6_S2_END_MM  (4570.0f * MODE6_DISTANCE_SCALE)
#define MODE6_FINISH_MM  (5600.0f)
#define MODE6_MAX_DUTY  (26)
#define MODE6_START_DUTY (16)
#define MODE6_CURVE_DUTY (21)
#define MODE6_FINAL_DUTY (16)
#define MODE6_ACCEL_MS  (800U)
#define MODE6_TRANS_MM  (260.0f * MODE6_DISTANCE_SCALE)
#define MODE6_DECEL_MM  (520.0f * MODE6_DISTANCE_SCALE)
#define MODE6_S_KP  (0.45f)
#define MODE6_S_LIM  (10.0f)
#define MODE6_C_KP  (0.55f)
#define MODE6_C_LIM  (13.0f)
#define MODE6_C_BIAS (2.6f)
#define MODE6_LOST  (13.0f)
#define MODE6_SIGN_TH (10)
#define MODE6_SIGN_CNT (20U)
#define MODE6_PRETILT_MS (200U)
#define MODE6_FF_START (1.3f)
#define MODE6_FF_BRAKE (-1.0f)
#define MODE6_HOLD_MS (300U)
#define MODE6_REL_MS (400U)
#define MODE6_SETTLE_MS (800U)
#define MODE6_SLEW_START (0.60f)
#define MODE6_SLEW_FAST (0.80f)
#define MODE6_SLEW_NORM (0.20f)
#define MODE6_FAST_ENTER (3.0f)
#define MODE6_FAST_EXIT (2.0f)
#define MODE6_SPD_ENTER (40.0f)
#define MODE6_SPD_EXIT (20.0f)

static Mode6_State s_state;
static uint32_t s_phaseMs;
static int8_t s_drive;
static int8_t s_lastErr;
static int8_t s_curveDir;
static int8_t s_curveCand;
static uint8_t s_curveCnt;
static uint8_t s_fast;
static float s_targetMm;

static float absF(float v) { return (v>=0)?v:-v; }
static int8_t clmp(int16_t v) { if(v>MODE6_MAX_DUTY) return MODE6_MAX_DUTY; if(v<0) return 0; return (int8_t)v; }
static float clmpF(float v,float lo,float hi) { if(v<lo) return lo; if(v>hi) return hi; return v; }
static void diffCmd(int16_t base,int16_t corr,int8_t *l,int8_t *r) {
    int16_t lc=base+corr, rc=base-corr;
    int16_t mx=(lc>rc)?lc:rc;
    if(mx>MODE6_MAX_DUTY){int16_t e=mx-MODE6_MAX_DUTY; lc-=e; rc-=e;}
    *l=clmp(lc); *r=clmp(rc);
}

static void ballSlew(void) {
    float pa=absF(HFZero_GetPositionMm()-s_targetMm);
    float va=absF(HFZero_GetVelocityMmS());
    if(s_fast){ if(pa<=MODE6_FAST_EXIT && va<=MODE6_SPD_EXIT) s_fast=0; }
    else if(pa>=MODE6_FAST_ENTER || va>=MODE6_SPD_ENTER) s_fast=1;
}

static void learnCurve(int8_t err) {
    int8_t cand;
    if(s_curveDir) return;
    if(err>=MODE6_SIGN_TH) cand=1; else if(err<=-MODE6_SIGN_TH) cand=-1;
    else { s_curveCand=0; s_curveCnt=0; return; }
    if(cand==s_curveCand){ if(s_curveCnt<MODE6_SIGN_CNT) s_curveCnt++; }
    else { s_curveCand=cand; s_curveCnt=1; }
    if(s_curveCnt>=MODE6_SIGN_CNT) s_curveDir=cand;
}

void Mode6_Init(void) { s_state=MODE6_IDLE; s_targetMm=0; }

void Mode6_Stop(void) {
    TB6612_Stop();
    HFZero_SetFeedforwardAngleDeg(0.0f);
    HFZero_SetStandaloneOscillationMode(false);
    HFZero_SetEnabled(false);
    HFZero_SetTargetMm(0.0f);
    s_state=MODE6_IDLE; s_drive=0; s_lastErr=0;
    s_curveDir=0; s_curveCand=0; s_curveCnt=0; s_fast=0;
}

void Mode6_OnStartStop(void) {
    if (AB_GetMode() != MODE_HOLD6) return;
    if ((s_state!=MODE6_IDLE) && (s_state!=MODE6_DONE)) { Mode6_Stop(); return; }
    if (BallBalance_IsBallVisible()) s_targetMm=BallBalance_GetVisionPositionMm();
    else s_targetMm=0.0f;
    TB6612_Stop(); EncoderDistance_Reset();
    s_drive=0; s_lastErr=0; s_curveDir=0; s_curveCand=0; s_curveCnt=0; s_fast=1;
    HFZero_SetTargetMm(s_targetMm);
    HFZero_SetFeedforwardAngleDeg(MODE6_FF_START);
    HFZero_SetEnabled(true);
    s_phaseMs=Timebase_GetMillis();
    s_state=MODE6_HOLD;
}

void Mode6_Update(const LineTracker_State *line, int8_t *left, int8_t *right) {
    float dist, rem, corr, kp, lim;
    int16_t base; uint32_t el; uint8_t onCurve;
    Mode6_State prev;
    *left=0; *right=0;
    if ((s_state==MODE6_IDLE)||(s_state==MODE6_DONE)) return;
    if (s_state==MODE6_HOLD) {
        TB6612_Stop();
        if ((Timebase_GetMillis()-s_phaseMs)>=MODE6_PRETILT_MS) { s_phaseMs=Timebase_GetMillis(); s_state=MODE6_DRIVE; }
        return;
    }
    if (s_state==MODE6_FINAL) {
        uint32_t fms=Timebase_GetMillis()-s_phaseMs;
        TB6612_Stop();
        if(fms<MODE6_HOLD_MS) { HFZero_SetFeedforwardAngleDeg(MODE6_FF_BRAKE); }
        else if(fms<(MODE6_HOLD_MS+MODE6_REL_MS)) {
            float r=(float)(fms-MODE6_HOLD_MS)/(float)MODE6_REL_MS;
            HFZero_SetFeedforwardAngleDeg(MODE6_FF_BRAKE*(1.0f-r));
        } else { HFZero_SetFeedforwardAngleDeg(0.0f); ballSlew(); }
        if(fms>=MODE6_SETTLE_MS) s_state=MODE6_DONE;
        return;
    }
    dist=EncoderDistance_GetAverageMm();
    if(dist>=MODE6_FINISH_MM) {
        TB6612_Stop(); HFZero_SetFeedforwardAngleDeg(MODE6_FF_BRAKE);
        s_drive=0; s_phaseMs=Timebase_GetMillis(); s_state=MODE6_FINAL; return;
    }
    prev=s_state;
    if(dist<MODE6_S1_END_MM) s_state=MODE6_DRIVE;
    else if(dist<MODE6_A1_END_MM) s_state=MODE6_DRIVE;
    else if(dist<MODE6_S2_END_MM) s_state=MODE6_DRIVE;
    else s_state=MODE6_DRIVE;
    onCurve=(uint8_t)((s_state==MODE6_DRIVE) && (dist>=MODE6_S1_END_MM && dist<MODE6_A1_END_MM) || (dist>=MODE6_S2_END_MM));
    base = onCurve ? MODE6_CURVE_DUTY : MODE6_MAX_DUTY;
    el=Timebase_GetMillis()-s_phaseMs;
    if((s_state==MODE6_DRIVE) && (el<MODE6_ACCEL_MS)) {
        base=MODE6_START_DUTY+(int16_t)((uint32_t)(MODE6_MAX_DUTY-MODE6_START_DUTY)*el/MODE6_ACCEL_MS);
    } else {
        HFZero_SetFeedforwardAngleDeg(0.0f); ballSlew();
    }
    rem=MODE6_FINISH_MM-dist;
    if(onCurve && (rem<=MODE6_DECEL_MM)) {
        float r=clmpF(rem/MODE6_DECEL_MM,0.0f,1.0f);
        base=MODE6_FINAL_DUTY+(int16_t)((float)(MODE6_CURVE_DUTY-MODE6_FINAL_DUTY)*r);
        HFZero_SetFeedforwardAngleDeg(MODE6_FF_BRAKE*(1.0f-r));
    }
    kp = onCurve ? MODE6_C_KP : MODE6_S_KP;
    lim = onCurve ? MODE6_C_LIM : MODE6_S_LIM;
    if(line->lineFound!=0U) {
        s_lastErr=line->error;
        if(onCurve) learnCurve(line->error);
        corr=kp*(float)line->error;
        if(onCurve && s_curveDir) corr+=(float)s_curveDir*MODE6_C_BIAS;
    } else {
        int8_t rd=s_curveDir?s_curveDir:((s_lastErr>0)?1:((s_lastErr<0)?-1:0));
        if(!rd) { s_drive=0; TB6612_Stop(); return; }
        corr=(float)rd*MODE6_LOST;
    }
    corr=clmpF(corr,-lim,lim);
    diffCmd(base,(int16_t)corr,left,right);
    s_drive=(int8_t)base;
    TB6612_SetSpeed(*left,*right);
}

uint8_t Mode6_IsRunning(void) { return (uint8_t)((s_state!=MODE6_IDLE)&&(s_state!=MODE6_DONE)); }
Mode6_State Mode6_GetState(void) { return s_state; }
float Mode6_GetTargetMm(void) { return s_targetMm; }
