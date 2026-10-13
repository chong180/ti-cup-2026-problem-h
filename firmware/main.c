#include <stdint.h>
#include <string.h>

#include "drivers/oled.h"
#include "drivers/delay.h"
#include "drivers/key.h"
#include "drivers/line_tracker.h"
#include "drivers/motor_tb6612.h"
#include "drivers/pid.h"
#include "app/sys_mode.h"
#include "app/mode_fixed_distance.h"
#include "app/mode_ab20.h"
#include "app/mode_full_loop.h"
#include "app/mode_arc_trajectory.h"
#include "app/mode_loop_arc.h"
#include "app/mode_loop_offset.h"
#include "app/mode_loop_arc_stop.h"
#include "drivers/esp32_camera_link.h"
#include "ti_msp_dl_config.h"
#include "drivers/timebase.h"
#include "drivers/encoder_odometry.h"
#include "drivers/encoder_speed.h"
#include "drivers/stepper_emm_v5.h"
#include "ball_balance/ball_balance.h"
#include "ball_balance/ball_zero_control.h"
#include "ball_balance/ball_route.h"
#include "ball_balance/ball_hf_zero.h"
#include "ball_balance/ball_route_ramp.h"

/* ===== Yaw角度数据 (全局, 供OLED显示) ===== */
static float gMpuYaw   = 0.0f;

/* ===== 循迹模式参数 =====
 * 循迹控制逻辑:
 *   error > 0 (线在右侧) -> correction > 0 -> left加速 + right减速 -> 右转
 *   error < 0 (线在左侧) -> correction < 0 -> left减速 + right加速 -> 左转
 *   丢线时按上次误差方向施加固定校正量回拐找线 */
#define DISPLAY_PERIOD_MS        (100U)
#define CONTROL_PERIOD_SECONDS   (0.001f)
#define PID_CORRECTION_LIMIT     (35.0f)

#define PID_KP_DEFAULT           (0.60f)
#define PID_KI_DEFAULT           (0.0f)
#define PID_KD_DEFAULT           (0.0f)
#define PID_KP_STEP              (0.05f)
#define PID_LOST_LINE_CORRECTION (35.0f)
#define LINE_CENTER_DEADBAND     (5)
#define LINE_LEFT_TRIM_PERCENT   (0)
#define LINE_RIGHT_TRIM_PERCENT  (0)
#define LINE_TRACK_DISPLAY_TARGET_MM (5900.0f)
#define LINE_TRACK_STOP_TRIGGER_MM   (5630.0f)
#define LINE_TRACK_SLOW_REMAIN_MM    (2000.0f)
#define LINE_TRACK_MIN_SPEED         (25)
#define LINE_TRACK_MOTOR_FLOOR       (24)
#define LINE_TRACK_MOTOR_CEILING     (55)
#define LINE_TRACK_RECOVERY_CEILING  (60)
#define LINE_ERROR_FILTER_CENTER_MS  (25.0f)
#define LINE_ERROR_FILTER_TURN_MS    (8.0f)
#define LINE_ERROR_TURN_THRESHOLD    (10)
#define WHEEL_SPEED_PI_PERIOD_MS     (10U)
#define WHEEL_SPEED_AT_42_PWM_MMPS   (500.0f)
#define LEFT_SPEED_AT_42_PWM_MMPS    (450.0f)
#define RIGHT_SPEED_AT_42_PWM_MMPS   (560.0f)
#define WHEEL_SPEED_PI_KP            (0.010f)
#define WHEEL_SPEED_PI_KI            (0.030f)
#define WHEEL_SPEED_PI_LIMIT         (8.0f)
#define WHEEL_SPEED_INTEGRAL_LIMIT   (200.0f)

/* ===== 继电反馈自整定 =====
 * Åström-Hägglund 继电反馈法:
 *   1. 用 bang-bang 控制替代 PID, 修正量 = ±RELAY_AMP
 *   2. 系统产生极限环振荡, MATLAB 端测量振荡周期 Tu 和振幅 a
 *   3. 临界增益 Ku = 4*RELAY_AMP / (pi*a)
 *   4. ZN 整定: Kp=0.6*Ku, Ki=1.2*Ku/Tu, Kd=0.075*Ku*Tu */
static uint8_t  gRunning = 0U;
static int8_t   gLastLineError = 0;
static int8_t   gLineSpeed     = 30;   /* 编码器长距离巡线基速(%), 不影响AB模式 */
static int8_t   gLineDriveSpeed = 30;  /* 当前速度，接近终点时自动降到25% */
static float    gFilteredLineError = 0.0f;
static uint32_t gLineFilterLastMs = 0U;
static float    gLeftSpeedIntegral = 0.0f;
static float    gRightSpeedIntegral = 0.0f;
static uint32_t gWheelSpeedPiLastMs = 0U;
static int8_t   gClosedLoopLeftPwm = 0;
static int8_t   gClosedLoopRightPwm = 0;
static PID_Controller gLinePid;

/* ===== 编码器距离微调 (S3加/S4减, 步距0.5cm) =====
 * 各模式启动时把此值加到总目标距离; 冲突距离加到末段 */
static float gEncoderAdjustMm = 0.0f;
#define ENCODER_ADJUST_STEP_MM  (5.0f)  /* 0.5cm = 5mm */
static float gLineTrackStopMm = LINE_TRACK_STOP_TRIGGER_MM;

/* Getter for mode modules to read the encoder adjust value */
float EncoderDistance_GetAdjustMm(void) { return gEncoderAdjustMm; }

static int8_t   gLeftSpeed     = 0;       /* 左轮速度(%), 供串口回调访问 */
static int8_t   gRightSpeed    = 0;       /* 右轮速度(%), 供串口回调访问 */

/* ===== 遥控模式 =====
 * 命令: FWD/BWD/CW/CCW(按住) REL(释放)
 *       TL90/TL180/TR90/TR180(原地转指定角度)
 * 右转(yaw增加): 左轮正+右轮负
 * 左转(yaw减少): 左轮负+右轮正 */
typedef enum {
    RC_IDLE = 0,  RC_FWD,  RC_BWD,  RC_CW,  RC_CCW,  RC_TURN,
    RC_TEST_L,    /* 单轮编码器测试: 仅左轮转 */
    RC_TEST_R,    /* 单轮编码器测试: 仅右轮转 */
} RC_State;
#define RC_TEST_SPEED   (30)   /* 单轮测试占空比(%) */
static RC_State gRcState     = RC_IDLE;
static float    gRcTargetYaw = 0.0f;
/* Car run timer: reset on start, freeze on manual or automatic stop. */
static uint8_t  gCarTimerRunning = 0U;
static uint32_t gCarTimerStartMs = 0U;
static uint32_t gCarElapsedMs    = 0U;
static uint32_t gCarStopCandidateMs = 0U;
#define CAR_TIMER_MOTION_THRESHOLD_MMPS (20.0f)
#define CAR_TIMER_STOP_CONFIRM_MS       (200U)

static uint8_t carIsRunning(void)
{
    switch (AB_GetMode()) {
    case MODE_LINE_TRACK:
        return gRunning;
    case MODE_AB:
        return (uint8_t)((AB_GetState() == AB_LEAVE) ||
                         (AB_GetState() == AB_RUN));
    case MODE_BALL_AB20:
        return ModeAB_IsRunning();
    case MODE_ABCD:
        return ABCD_IsRunning();
    case MODE_HOLD6:
        return Mode6_IsRunning();
    case MODE_006:
        return Mode006_IsRunning();
    case MODE_007:
        return Mode007_IsRunning();
    case MODE_5NEW:
        return Route5New_IsRunning();
    case MODE_006_1:
        return Mode0061_IsRunning();
    case MODE_DRIVE1:
        return Drive1_IsRunning();
    case MODE_REMOTE:
        return (uint8_t)(gRcState != RC_IDLE);
    default:
        return 0U;
    }
}

static void updateCarTimer(void)
{
    uint8_t running = carIsRunning();
    uint32_t nowMs = Timebase_GetMillis();
    float wheelSpeedMmPs = EncoderDistance_GetAverageMmPs();

    if ((running != 0U) && (gCarTimerRunning == 0U) &&
        (wheelSpeedMmPs > CAR_TIMER_MOTION_THRESHOLD_MMPS)) {
        gCarTimerStartMs = nowMs;
        gCarElapsedMs = 0U;
        gCarTimerRunning = 1U;
        gCarStopCandidateMs = 0U;
    }

    if (gCarTimerRunning != 0U) {
        gCarElapsedMs = nowMs - gCarTimerStartMs;
        if ((running == 0U) &&
            (wheelSpeedMmPs <= CAR_TIMER_MOTION_THRESHOLD_MMPS)) {
            if (gCarStopCandidateMs == 0U) {
                gCarStopCandidateMs = nowMs;
            } else if ((nowMs - gCarStopCandidateMs) >=
                       CAR_TIMER_STOP_CONFIRM_MS) {
                gCarTimerRunning = 0U;
                gCarStopCandidateMs = 0U;
            }
        } else {
            gCarStopCandidateMs = 0U;
        }
    }
}
#define RC_TURN_TOL     (3.0f)   /* 转弯到位容差(度) */

static float rc_wrapAngle(float deg)
{
    while (deg >= 180.0f)  deg -= 360.0f;
    while (deg < -180.0f)  deg += 360.0f;
    return deg;
}

/* ===== 简易浮点解析 (替代atof, 节省Flash) ===== */
static float my_atof(const char *s);

/* ===== 无线串口命令回调 =====
 * PC通过MATLAB发送命令: $C,<cmd>[,<param>]\r\n
 * 支持的命令:
 *   KP,<val>  设置循迹Kp     KI,<val>  设置循迹Ki
 *   KD,<val>  设置循迹Kd     RST       航向归零
 *   RUN       启动           STOP      停车
 *   TKP,<val> 设置Track50 Kp  TKD,<val> 设置Track50 Kd */
#if 0 /* Legacy wireless command parser; UART3 is now used by the stepper. */
static void onWirelessCmd(const char *cmd, const char *param)
{
    if (strcmp(cmd, "KP") == 0 && param != NULL) {
        gLinePid.kp = my_atof(param);
    } else if (strcmp(cmd, "KI") == 0 && param != NULL) {
        gLinePid.ki = my_atof(param);
    } else if (strcmp(cmd, "KD") == 0 && param != NULL) {
        gLinePid.kd = my_atof(param);
    } else if (strcmp(cmd, "RST") == 0) {
        Angle_ResetYaw();
    } else if (strcmp(cmd, "DCLR") == 0) {
        /* 编码器里程清零 (MATLAB轨迹同步重置原点) */
        EncoderDistance_Reset();
    } else if (strcmp(cmd, "ET") == 0 && param != NULL) {
        /* 单轮编码器测试: ET,L=仅左轮 ET,R=仅右轮 ET,S=停止
         * 自动切到遥控模式, 用于验证编码器方向与计数 */
        AB_SetMode(MODE_REMOTE);
        gRunning = 0U;
        if (strcmp(param, "L") == 0) {
            gRcState = RC_TEST_L;
        } else if (strcmp(param, "R") == 0) {
            gRcState = RC_TEST_R;
        } else {
            gRcState = RC_IDLE;
            TB6612_Stop();
        }
    } else if (strcmp(cmd, "RUN") == 0) {
        /* 模拟S1启动 */
        if (AB_GetMode() == MODE_LINE_TRACK) {
            if (gRunning == 0U) {
                EncoderDistance_Reset();
                PID_Reset(&gLinePid);
                gLineDriveSpeed = gLineSpeed;
            }
            gRunning = 1U;
        } else if (AB_GetMode() == MODE_AB) {
            AB_OnStartStop();
        } else if (AB_GetMode() == MODE_TRACK50) {
            Track50_OnStartStop();
        } else if (AB_GetMode() == MODE_DRIVE1) {
            Drive1_OnStartStop();
        }
    } else if (strcmp(cmd, "STOP") == 0) {
        gRunning = 0U;
        gRcState = RC_IDLE;
        TB6612_Stop();
        gLeftSpeed = 0;
        gRightSpeed = 0;
    } else if (strcmp(cmd, "MODE") == 0 && param != NULL) {
        int16_t m = (int16_t)my_atof(param);
        if (m >= 0 && m <= (int16_t)MODE_DRIVE1) {
            AB_SetMode((SysMode)m);
            gRunning = 0U;  gRcState = RC_IDLE;
            TB6612_Stop();
        }
    } else if (strcmp(cmd, "FWD") == 0) {
        gRcState = RC_FWD;
    } else if (strcmp(cmd, "BWD") == 0) {
        gRcState = RC_BWD;
    } else if (strcmp(cmd, "CW") == 0) {
        gRcState = RC_CW;
    } else if (strcmp(cmd, "CCW") == 0) {
        gRcState = RC_CCW;
    } else if (strcmp(cmd, "REL") == 0) {
        gRcState = RC_IDLE;
        TB6612_Stop();
    } else if (strcmp(cmd, "TL90") == 0) {
        gRcTargetYaw = rc_wrapAngle(gMpuYaw - 90.0f);
        gRcState = RC_TURN;
    } else if (strcmp(cmd, "TL180") == 0) {
        gRcTargetYaw = rc_wrapAngle(gMpuYaw - 180.0f);
        gRcState = RC_TURN;
    } else if (strcmp(cmd, "TR90") == 0) {
        gRcTargetYaw = rc_wrapAngle(gMpuYaw + 90.0f);
        gRcState = RC_TURN;
    } else if (strcmp(cmd, "TR180") == 0) {
        gRcTargetYaw = rc_wrapAngle(gMpuYaw + 180.0f);
        gRcState = RC_TURN;
    } else if (strcmp(cmd, "TKP") == 0 && param != NULL) {
        Track50_AdjustKp(my_atof(param) - Track50_GetKp());
    } else if (strcmp(cmd, "SPD") == 0 && param != NULL) {
        int16_t spd = (int16_t)my_atof(param);
        if (spd < 5)  spd = 5;
        if (spd > 80) spd = 80;
        gLineSpeed = (int8_t)spd;
        if (gRunning == 0U) {
            gLineDriveSpeed = gLineSpeed;
        }
    } else if (strcmp(cmd, "TUNE") == 0 && param != NULL) {
        /* 继电反馈自整定: TUNE,ON 启动 / TUNE,OFF 停止 */
        if (strcmp(param, "ON") == 0) {
            AB_SetMode(MODE_LINE_TRACK);
            gTuneMode = 1U;
            gRunning  = 1U;
            PID_Reset(&gLinePid);
        } else {
            gTuneMode = 0U;
            gRunning  = 0U;
            TB6612_Stop();
        }
    } else if (strcmp(cmd, "TKD") == 0 && param != NULL) {
        /* Track50没有单独设Kd的接口, 此处预留 */
    }
}

/* ===== 循迹模式: 速度限幅 ===== */
#endif

static int8_t clampSpeed(int16_t speed)
{
    if (speed >  gLineSpeed) return  gLineSpeed;
    if (speed < -gLineSpeed) return -gLineSpeed;
    return (int8_t)speed;
}

/* Final safety clamp only. The correction limiter, rather than this helper,
 * keeps normal tracking commands above the motor dead zone. */
static int8_t clampForwardSpeed(int16_t speed)
{
    if (speed > LINE_TRACK_MOTOR_CEILING) return LINE_TRACK_MOTOR_CEILING;
    if (speed < 0) return 0;
    return (int8_t)speed;
}

static int8_t clampRecoverySpeed(int16_t speed)
{
    if (speed > LINE_TRACK_RECOVERY_CEILING) {
        return LINE_TRACK_RECOVERY_CEILING;
    }
    if (speed < 0) return 0;
    return (int8_t)speed;
}

static float limitCorrectionForSpeed(float correction)
{
    int16_t maximumTrim = LINE_LEFT_TRIM_PERCENT;
    if (LINE_RIGHT_TRIM_PERCENT > maximumTrim) {
        maximumTrim = LINE_RIGHT_TRIM_PERCENT;
    }
    /*
     * Keep headroom for the outside wheel. The inside wheel is clamped to
     * MOTOR_FLOOR later, so it never falls into the motor dead zone while
     * the outside wheel can still create enough speed difference on curves.
     */
    float limit = (float)(LINE_TRACK_MOTOR_CEILING -
                          gLineDriveSpeed - maximumTrim);
    if (limit < 0.0f) {
        limit = 0.0f;
    }
    if (correction > limit) return limit;
    if (correction < -limit) return -limit;
    return correction;
}

/* ===== 简易浮点解析 (替代atof, 节省Flash) ===== */
static float clampWheelPi(float value)
{
    if (value > WHEEL_SPEED_PI_LIMIT) return WHEEL_SPEED_PI_LIMIT;
    if (value < -WHEEL_SPEED_PI_LIMIT) return -WHEEL_SPEED_PI_LIMIT;
    return value;
}

static void resetWheelSpeedPi(void)
{
    gLeftSpeedIntegral = 0.0f;
    gRightSpeedIntegral = 0.0f;
    gWheelSpeedPiLastMs = 0U;
    gClosedLoopLeftPwm = 0;
    gClosedLoopRightPwm = 0;
}

static void setTrackSpeedClosedLoop(int8_t desiredLeftPwm,
                                    int8_t desiredRightPwm)
{
    uint32_t nowMs = Timebase_GetMillis();
    uint32_t dtMs = nowMs - gWheelSpeedPiLastMs;

    if ((gWheelSpeedPiLastMs == 0U) ||
        (dtMs >= WHEEL_SPEED_PI_PERIOD_MS)) {
        float dtSeconds;
        float targetLeft;
        float targetRight;
        float errorLeft;
        float errorRight;
        float correctionLeft;
        float correctionRight;
        float feedforwardLeft;
        float feedforwardRight;
        int16_t outputLeft;
        int16_t outputRight;

        if ((gWheelSpeedPiLastMs == 0U) || (dtMs > 100U)) {
            dtMs = WHEEL_SPEED_PI_PERIOD_MS;
        }
        dtSeconds = (float)dtMs * 0.001f;
        gWheelSpeedPiLastMs = nowMs;

        targetLeft = (float)desiredLeftPwm *
                     WHEEL_SPEED_AT_42_PWM_MMPS / 42.0f;
        targetRight = (float)desiredRightPwm *
                      WHEEL_SPEED_AT_42_PWM_MMPS / 42.0f;
        errorLeft = targetLeft - Encode_GetLeftSpeedMmPs();
        errorRight = targetRight - Encode_GetRightSpeedMmPs();

        gLeftSpeedIntegral += errorLeft * dtSeconds;
        gRightSpeedIntegral += errorRight * dtSeconds;
        if (gLeftSpeedIntegral > WHEEL_SPEED_INTEGRAL_LIMIT) {
            gLeftSpeedIntegral = WHEEL_SPEED_INTEGRAL_LIMIT;
        } else if (gLeftSpeedIntegral < -WHEEL_SPEED_INTEGRAL_LIMIT) {
            gLeftSpeedIntegral = -WHEEL_SPEED_INTEGRAL_LIMIT;
        }
        if (gRightSpeedIntegral > WHEEL_SPEED_INTEGRAL_LIMIT) {
            gRightSpeedIntegral = WHEEL_SPEED_INTEGRAL_LIMIT;
        } else if (gRightSpeedIntegral < -WHEEL_SPEED_INTEGRAL_LIMIT) {
            gRightSpeedIntegral = -WHEEL_SPEED_INTEGRAL_LIMIT;
        }
        correctionLeft = clampWheelPi(
            WHEEL_SPEED_PI_KP * errorLeft +
            WHEEL_SPEED_PI_KI * gLeftSpeedIntegral);
        correctionRight = clampWheelPi(
            WHEEL_SPEED_PI_KP * errorRight +
            WHEEL_SPEED_PI_KI * gRightSpeedIntegral);

        feedforwardLeft = (float)desiredLeftPwm *
                          WHEEL_SPEED_AT_42_PWM_MMPS /
                          LEFT_SPEED_AT_42_PWM_MMPS;
        feedforwardRight = (float)desiredRightPwm *
                           WHEEL_SPEED_AT_42_PWM_MMPS /
                           RIGHT_SPEED_AT_42_PWM_MMPS;
        outputLeft = (int16_t)(feedforwardLeft + correctionLeft);
        outputRight = (int16_t)(feedforwardRight + correctionRight);
        if (outputLeft > LINE_TRACK_RECOVERY_CEILING) {
            outputLeft = LINE_TRACK_RECOVERY_CEILING;
        }
        if (outputRight > LINE_TRACK_RECOVERY_CEILING) {
            outputRight = LINE_TRACK_RECOVERY_CEILING;
        }
        if (outputLeft < LINE_TRACK_MOTOR_FLOOR) {
            outputLeft = LINE_TRACK_MOTOR_FLOOR;
        }
        if (outputRight < LINE_TRACK_MOTOR_FLOOR) {
            outputRight = LINE_TRACK_MOTOR_FLOOR;
        }
        gClosedLoopLeftPwm = (int8_t)outputLeft;
        gClosedLoopRightPwm = (int8_t)outputRight;
    }

    TB6612_SetSpeed(gClosedLoopLeftPwm, gClosedLoopRightPwm);
}

static float my_atof(const char *s)
{
    float val = 0.0f;
    float sign = 1.0f;
    float frac = 0.1f;

    if (*s == '-') { sign = -1.0f; s++; }
    else if (*s == '+') { s++; }

    while (*s >= '0' && *s <= '9') {
        val = val * 10.0f + (float)(*s - '0');
        s++;
    }

    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            val += (float)(*s - '0') * frac;
            frac *= 0.1f;
            s++;
        }
    }

    return val * sign;
}

/* ===== 循迹模式: 电机更新 =====
 * correction > 0 (线偏右) -> left加速 + right减速 -> 车右转
 * correction < 0 (线偏左) -> left减速 + right加速 -> 车左转 */
static void updateMotor(const LineTracker_State *line, int8_t *left, int8_t *right)
{
    float correction;
    int8_t controlError;
    int16_t leftBase  = (int16_t)gLineDriveSpeed + LINE_LEFT_TRIM_PERCENT;
    int16_t rightBase = (int16_t)gLineDriveSpeed + LINE_RIGHT_TRIM_PERCENT;

    if (line->lineFound != 0U) {
        gLastLineError = line->error;
    } else if (gLastLineError == 0) {
        /* A centered car can briefly see all white at a line gap. Continue
         * straight at low speed instead of stopping before the distance
         * target. */
        int16_t bridgeSpeed = gLineDriveSpeed;
        if (bridgeSpeed > LINE_TRACK_MIN_SPEED) {
            bridgeSpeed = LINE_TRACK_MIN_SPEED;
        }
        *left = clampForwardSpeed(bridgeSpeed + LINE_LEFT_TRIM_PERCENT);
        *right = clampForwardSpeed(bridgeSpeed + LINE_RIGHT_TRIM_PERCENT);
        setTrackSpeedClosedLoop(*left, *right);
        return;
    } else {
        PID_Reset(&gLinePid);
        correction = (gLastLineError < 0) ? -PID_LOST_LINE_CORRECTION :
                     PID_LOST_LINE_CORRECTION;
        correction = limitCorrectionForSpeed(correction);
        *left  = clampRecoverySpeed(leftBase + (int16_t)correction);
        *right = clampRecoverySpeed(rightBase - (int16_t)correction);
        setTrackSpeedClosedLoop(*left, *right);
        return;
    }

    /* The two center sensors have weights -5/+5. Remove that unavoidable
     * center offset, but preserve the remaining error for smooth steering. */
    if (line->error > LINE_CENTER_DEADBAND) {
        controlError = (int8_t)(line->error - LINE_CENTER_DEADBAND);
    } else if (line->error < -LINE_CENTER_DEADBAND) {
        controlError = (int8_t)(line->error + LINE_CENTER_DEADBAND);
    } else {
        controlError = 0;
    }

    {
        uint32_t nowMs = Timebase_GetMillis();
        uint32_t dtMs = nowMs - gLineFilterLastMs;

        if ((gLineFilterLastMs == 0U) || (dtMs > 100U)) {
            gFilteredLineError = (float)controlError;
        } else if (dtMs != 0U) {
            int16_t absError = (controlError < 0) ?
                               -(int16_t)controlError :
                               (int16_t)controlError;
            float absFilteredError = (gFilteredLineError < 0.0f) ?
                                     -gFilteredLineError :
                                     gFilteredLineError;
            float filterTauMs =
                ((absError >= LINE_ERROR_TURN_THRESHOLD) ||
                 (absFilteredError >= 2.0f)) ?
                LINE_ERROR_FILTER_TURN_MS :
                LINE_ERROR_FILTER_CENTER_MS;
            float alpha = (float)dtMs /
                          (filterTauMs + (float)dtMs);
            gFilteredLineError +=
                alpha * ((float)controlError - gFilteredLineError);
        }
        gLineFilterLastMs = nowMs;
    }

    correction = gLinePid.kp * gFilteredLineError;
    correction = limitCorrectionForSpeed(correction);
    *left  = clampForwardSpeed(leftBase + (int16_t)correction);
    *right = clampForwardSpeed(rightBase - (int16_t)correction);
    setTrackSpeedClosedLoop(*left, *right);
}

/* ===== OLED显示 (全程显示模式类型 + 陀螺仪角度) =====
 * 显示布局 (128x64 OLED):
 *   y=0:  8x16大字 模式名
 *   y=16: 6x8 Yaw角度
 *   y=24: 6x8 模式信息1
 *   y=32: 6x8 模式信息2
 *   y=40: 6x8 模式信息3
 *   y=48: 6x8 模式信息4
 *   y=56: 6x8 模式信息5
 */
static void updateDisplay(const LineTracker_State *line, int8_t left, int8_t right)
{
    SysMode mode = AB_GetMode();

    OLED_Clear();

    /* 第一行(8x16大字): 显示模式类型 */
    if (mode == MODE_LINE_TRACK) {
        OLED_ShowString(0, 0, "MODE:TRACK", OLED_8X16);
    } else if (mode == MODE_AB) {
        OLED_ShowString(0, 0, "MODE:A->B", OLED_8X16);
    } else if (mode == MODE_BALL_LINE) {
        OLED_ShowString(0, 0, "MODE:ZERO", OLED_8X16);
    } else if (mode == MODE_BALL_ROUTE) {
        OLED_ShowString(0, 0, "MODE:+5>-5", OLED_8X16);
    } else if (mode == MODE_BALL_AB20) {
        OLED_ShowString(0, 0, "MODE:AB20", OLED_8X16);
    } else if (mode == MODE_ABCD) {
        OLED_ShowString(0, 0, "MODE:ABCD", OLED_8X16);
    } else if (mode == MODE_HOLD6) {
        OLED_ShowString(0, 0, "MODE:HOLD6", OLED_8X16);
    } else if (mode == MODE_006) {
        OLED_ShowString(0, 0, "MODE:006", OLED_8X16);
    } else if (mode == MODE_007) {
        OLED_ShowString(0, 0, "MODE:007", OLED_8X16);
    } else if (mode == MODE_5NEW) {
        OLED_ShowString(0, 0, "MODE:5.NEW", OLED_8X16);
    } else if (mode == MODE_006_1) {
        OLED_ShowString(0, 0, "MODE:006(1)", OLED_8X16);
    } else if (mode == MODE_HF_ZERO) {
        OLED_ShowString(0, 0, "MODE:HFZERO", OLED_8X16);
    } else if (mode == MODE_DRIVE1) {
        OLED_ShowString(0, 0, "MODE:DRV1", OLED_8X16);
    } else if (mode == MODE_REMOTE) {
        OLED_ShowString(0, 0, "MODE:RC", OLED_8X16);
    }

    /* 第二行(6x8): Yaw角度 */
    OLED_Printf(0, 16, OLED_6X8, "YAW:%+.1f ADJ:%.1f", gMpuYaw,
                (double)(gEncoderAdjustMm * 0.1f));

    if (mode == MODE_LINE_TRACK) {
        /* ---- 循迹模式信息 ---- */
        OLED_Printf(0, 24, OLED_6X8, "ST:%s Kp%.2f SP:%d",
                    (gRunning != 0U) ? "RUN" : "STOP", gLinePid.kp,
                    gLineDriveSpeed);
        OLED_Printf(0, 32, OLED_6X8, "ERR:%d MASK:%02X", line->error, line->activeMask);
        OLED_Printf(0, 40, OLED_6X8, "VL:%d VR:%d",
                    (int)Encode_GetLeftSpeedMmPs(),
                    (int)Encode_GetRightSpeedMmPs());
    } else if (mode == MODE_AB) {
        /* ---- AB模式信息 ---- */
        const char *abStr;
        switch (AB_GetState()) {
        case AB_IDLE:  abStr = "IDLE";  break;
        case AB_LEAVE: abStr = "LEAVE"; break;
        case AB_RUN:   abStr = "RUN";   break;
        case AB_DONE:  abStr = "DONE";  break;
        default:       abStr = "????";  break;
        }
        OLED_Printf(0, 24, OLED_6X8, "AB:%s RX:%s", abStr,
                    BallBalance_IsBallVisible() ? "OK" : "NO");
        OLED_Printf(0, 32, OLED_6X8, "BALL:%+.1fcm A:%+.1f",
                    (double)(HFZero_GetPositionMm() * 0.1f),
                    (double)HFZero_GetAngleDeg());
        OLED_Printf(0, 40, OLED_6X8, "D:%d/150 L:%d R:%d",
                    (int)(EncoderDistance_GetAverageMm() * 0.1f),
                    left, right);
    } else if (mode == MODE_BALL_LINE) {
        OLED_Printf(0, 24, OLED_6X8, "ST:%s RX:%s",
                    ZeroControl_IsEnabled() ? "RUN" : "STOP",
                    BallBalance_IsBallVisible() ? "OK" : "NO");
        OLED_Printf(0, 32, OLED_6X8, "P:%+.1fcm V:%+.0f",
                    (double)(HFZero_GetPositionMm() * 0.1f),
                    (double)HFZero_GetVelocityMmS());
        OLED_Printf(0, 40, OLED_6X8, "TV:%+.0f A:%+.2f",
                    (double)ZeroControl_GetTargetVelocityMmS(),
                    (double)HFZero_GetAngleDeg());
        OLED_Printf(0, 48, OLED_6X8, "PK:%.2f PD:%.2f",
                    (double)ZeroControl_GetPositionKp(),
                    (double)ZeroControl_GetPositionKd());
        OLED_Printf(0, 56, OLED_6X8, "VK:%.3f VD:%.4f",
                    (double)ZeroControl_GetVelocityKp(),
                    (double)ZeroControl_GetVelocityKd());
    } else if (mode == MODE_BALL_ROUTE) {
        const char *routeStr;
        switch (BallRoute_GetState()) {
        case BALL_ROUTE_IDLE:        routeStr = "IDLE"; break;
        case BALL_ROUTE_WAIT_ZERO:   routeStr = "ZERO"; break;
        case BALL_ROUTE_TO_POSITIVE: routeStr = "TO+5"; break;
        case BALL_ROUTE_TO_NEGATIVE: routeStr = "TO-5"; break;
        case BALL_ROUTE_DONE:        routeStr = "DONE"; break;
        default:                     routeStr = "????"; break;
        }
        OLED_Printf(0, 24, OLED_6X8, "ST:%s RX:%s", routeStr,
                    BallBalance_IsBallVisible() ? "OK" : "NO");
        OLED_Printf(0, 32, OLED_6X8, "P:%+.1f T:%+.1fcm",
                    (double)(HFZero_GetPositionMm() * 0.1f),
                    (double)(ZeroControl_GetTargetMm() * 0.1f));
        OLED_Printf(0, 40, OLED_6X8, "V:%+.0f TV:%+.0f",
                    (double)HFZero_GetVelocityMmS(),
                    (double)ZeroControl_GetTargetVelocityMmS());
        OLED_Printf(0, 48, OLED_6X8, "A:%+.2f", 
                    (double)HFZero_GetAngleDeg());
    } else if (mode == MODE_BALL_AB20) {
        const char *modeAbStr;
        switch (ModeAB_GetState()) {
        case MODEAB_IDLE:          modeAbStr = "IDLE"; break;
        case MODEAB_WAIT_BALL:     modeAbStr = "BALL"; break;
        case MODEAB_ACCEL:         modeAbStr = "ACC";  break;
        case MODEAB_CRUISE:        modeAbStr = "RUN";  break;
        case MODEAB_DECEL:         modeAbStr = "DEC";  break;
        case MODEAB_FINAL_BALANCE: modeAbStr = "SET";  break;
        case MODEAB_DONE:          modeAbStr = "DONE"; break;
        default:                   modeAbStr = "????"; break;
        }
        OLED_Printf(0, 24, OLED_6X8, "ST:%s RX:%s", modeAbStr,
                    BallBalance_IsBallVisible() ? "OK" : "NO");
        OLED_Printf(0, 32, OLED_6X8, "P:%+.1fcm V:%+.0f",
                    (double)(HFZero_GetPositionMm() * 0.1f),
                    (double)HFZero_GetVelocityMmS());
        OLED_Printf(0, 40, OLED_6X8, "D:%.1f/150cm",
                    (double)(ModeAB_GetDistanceMm() * 0.1f));
        OLED_Printf(0, 48, OLED_6X8, "PWM:%d L:%d R:%d",
                    ModeAB_GetDrivePercent(), left, right);
        OLED_Printf(0, 56, OLED_6X8, "ERR:%d MASK:%02X",
                    line->error, line->activeMask);
    } else if (mode == MODE_ABCD) {
        const char *abcdStr;
        switch (ABCD_GetState()) {
        case ABCD_IDLE:          abcdStr = "IDLE"; break;
        case ABCD_PRETILT:       abcdStr = "PRE";  break;
        case ABCD_STRAIGHT_1:    abcdStr = "S1";   break;
        case ABCD_ARC_1:         abcdStr = "C1";   break;
        case ABCD_STRAIGHT_2:    abcdStr = "S2";   break;
        case ABCD_ARC_2:         abcdStr = "C2";   break;
        case ABCD_FINAL_BALANCE: abcdStr = "SET";  break;
        case ABCD_DONE:          abcdStr = "DONE"; break;
        default:                 abcdStr = "????"; break;
        }
        OLED_Printf(0, 24, OLED_6X8, "ST:%s RX:%s", abcdStr,
                    BallBalance_IsBallVisible() ? "OK" : "NO");
        OLED_Printf(0, 32, OLED_6X8, "P:%+.1f A:%+.1f D:%d",
                    (double)(HFZero_GetPositionMm() * 0.1f),
                    (double)HFZero_GetAngleDeg(),
                    ABCD_GetCurveDirection());
        OLED_Printf(0, 40, OLED_6X8, "D:%.1f/614cm",
                    (double)(ABCD_GetDistanceMm() * 0.1f));
        OLED_Printf(0, 48, OLED_6X8, "PWM:%d L:%d R:%d",
                    ABCD_GetDrivePercent(), left, right);
        OLED_Printf(0, 56, OLED_6X8, "ERR:%d MASK:%02X",
                    line->error, line->activeMask);
    } else if (mode == MODE_HOLD6) {
        const char *m6Str;
        switch (Mode6_GetState()) {
        case MODE6_IDLE:  m6Str = "IDLE"; break;
        case MODE6_HOLD:  m6Str = "HOLD"; break;
        case MODE6_DONE:  m6Str = "DONE"; break;
        default:          m6Str = "????"; break;
        }
        OLED_Printf(0, 24, OLED_6X8, "ST:%s RX:%s", m6Str,
                    BallBalance_IsBallVisible() ? "OK" : "NO");
        OLED_Printf(0, 32, OLED_6X8, "P:%.1f T:%.1fcm",
                    (double)(HFZero_GetPositionMm() * 0.1f),
                    (double)(Mode6_GetTargetMm() * 0.1f));
        OLED_Printf(0, 40, OLED_6X8, "V:%+.0f A:%+.2f",
                    (double)HFZero_GetVelocityMmS(),
                    (double)HFZero_GetAngleDeg());
        OLED_ShowString(0, 48, "CAR:STOP", OLED_6X8);
    } else if (mode == MODE_006) {
        const char *m006Str;
        switch (Mode006_GetState()) {
        case MODE006_IDLE:  m006Str = "IDLE"; break;
        case MODE006_HOLD:  m006Str = "HOLD"; break;
        case MODE006_DRIVE: m006Str = "DRV";  break;
        case MODE006_TRACK: m006Str = "TRK";  break;
        case MODE006_FINAL: m006Str = "SET";  break;
        case MODE006_DONE:  m006Str = "DONE"; break;
        default:            m006Str = "????"; break;
        }
        OLED_Printf(0, 24, OLED_6X8, "ST:%s RX:%s", m006Str,
                    BallBalance_IsBallVisible() ? "OK" : "NO");
        OLED_Printf(0, 32, OLED_6X8, "P:%.1f T:%.1fcm",
                    (double)(HFZero_GetPositionMm() * 0.1f),
                    (double)(Mode006_GetTargetMm() * 0.1f));
        OLED_Printf(0, 40, OLED_6X8, "D:%.1f/%.0fcm",
                    (double)(Mode006_GetDistanceMm() * 0.1f),
                    (double)(Mode006_GetFinishMm() * 0.1f));
        OLED_Printf(0, 48, OLED_6X8, "PWM:%d L:%d R:%d",
                    Mode006_GetDrivePercent(), left, right);
    } else if (mode == MODE_007) {
        const char *m007Str;
        switch (Mode007_GetState()) {
        case MODE007_IDLE:  m007Str = "IDLE"; break;
        case MODE007_HOLD:  m007Str = "HOLD"; break;
        case MODE007_DRIVE: m007Str = "DRV";  break;
        case MODE007_FINAL: m007Str = "SET";  break;
        case MODE007_DONE:  m007Str = "DONE"; break;
        default:            m007Str = "????"; break;
        }
        OLED_Printf(0, 24, OLED_6X8, "ST:%s RX:%s", m007Str,
                    BallBalance_IsBallVisible() ? "OK" : "NO");
        OLED_Printf(0, 32, OLED_6X8, "P:%.1f T:%.1fcm",
                    (double)(HFZero_GetPositionMm() * 0.1f),
                    (double)(Mode007_GetTargetMm() * 0.1f));
        OLED_Printf(0, 40, OLED_6X8, "D:%.1f/%.0fcm",
                    (double)(Mode007_GetDistanceMm() * 0.1f),
                    (double)(Mode007_GetFinishMm() * 0.1f));
        OLED_Printf(0, 48, OLED_6X8, "PWM:%d L:%d R:%d",
                    Mode007_GetDrivePercent(), left, right);
    } else if (mode == MODE_5NEW) {
        const char *r5Str;
        switch (Route5New_GetState()) {
        case ROUTE5_NEW_IDLE:     r5Str = "IDLE"; break;
        case ROUTE5_NEW_RAMP_UP:  r5Str = "UP";   break;
        case ROUTE5_NEW_RAMP_DOWN: r5Str = "DOWN"; break;
        case ROUTE5_NEW_HOLD:      r5Str = "HOLD"; break;
        case ROUTE5_NEW_DONE:      r5Str = "DONE"; break;
        default:                   r5Str = "????"; break;
        }
        OLED_Printf(0, 24, OLED_6X8, "ST:%s RX:%s", r5Str,
                    BallBalance_IsBallVisible() ? "OK" : "NO");
        OLED_Printf(0, 32, OLED_6X8, "P:%.1f T:%.1fcm",
                    (double)(Route5New_GetPositionMm() * 0.1f),
                    (double)(Route5New_GetTargetMm() * 0.1f));
        OLED_ShowString(0, 40, "CAR:STOP", OLED_6X8);
    } else if (mode == MODE_006_1) {
        const char *m61Str;
        switch (Mode0061_GetState()) {
        case MODE0061_IDLE:  m61Str = "IDLE"; break;
        case MODE0061_HOLD:  m61Str = "HOLD"; break;
        case MODE0061_DRIVE: m61Str = "DRV";  break;
        case MODE0061_TRACK: m61Str = "TRK";  break;
        case MODE0061_FINAL: m61Str = "SET";  break;
        case MODE0061_DONE:  m61Str = "DONE"; break;
        default:             m61Str = "????"; break;
        }
        OLED_Printf(0, 24, OLED_6X8, "ST:%s RX:%s", m61Str,
                    BallBalance_IsBallVisible() ? "OK" : "NO");
        OLED_Printf(0, 32, OLED_6X8, "P:%.1f T:%.1fcm",
                    (double)(HFZero_GetPositionMm() * 0.1f),
                    (double)(Mode0061_GetTargetMm() * 0.1f));
        OLED_Printf(0, 40, OLED_6X8, "D:%.1f/%.0fcm",
                    (double)(Mode0061_GetDistanceMm() * 0.1f),
                    (double)(Mode0061_GetFinishMm() * 0.1f));
        OLED_Printf(0, 48, OLED_6X8, "PWM:%d L:%d R:%d",
                    Mode0061_GetDrivePercent(), left, right);
    } else if (mode == MODE_HF_ZERO) {
        const char *hfStr;
        switch (HFZero_GetState()) {
        case HF_ZERO_IDLE:    hfStr = "IDLE"; break;
        case HF_ZERO_ACQUIRE: hfStr = "ACQ";  break;
        case HF_ZERO_HOLD:    hfStr = "HOLD"; break;
        case HF_ZERO_FAST:    hfStr = "FAST"; break;
        case HF_ZERO_LOST:    hfStr = "LOST"; break;
        default:              hfStr = "????"; break;
        }
        OLED_Printf(0, 24, OLED_6X8, "ST:%s RX:%s", hfStr,
                    BallBalance_IsBallVisible() ? "OK" : "NO");
        OLED_Printf(0, 32, OLED_6X8, "P:%+.1fcm V:%+.0f",
                    (double)(HFZero_GetPositionMm() * 0.1f),
                    (double)HFZero_GetVelocityMmS());
        OLED_Printf(0, 40, OLED_6X8, "A:%+.2f KX:%.2f",
                    (double)HFZero_GetAngleDeg(),
                    (double)HFZero_GetPositionGain());
        OLED_ShowString(0, 48, "CAR:STOP", OLED_6X8);
    } else if (mode == MODE_DRIVE1) {
        /* ---- 驱动1模式信息: 状态/黑线路数/里程/循迹参数 ---- */
        const char *stStr;
        switch (Drive1_GetState()) {
        case DRIVE1_IDLE:  stStr = (Drive1_StartRejected() != 0U) ? "NEED3" : "IDLE"; break;
        case DRIVE1_RUN:   stStr = "RUN";  break;
        case DRIVE1_DONE:  stStr = "DONE"; break;
        default:           stStr = "????"; break;
        }
        OLED_Printf(0, 24, OLED_6X8, "ST:%s BLK:%u Kp%.2f",
                    stStr, Drive1_GetBlackCount(), (double)Drive1_GetKp());
        OLED_Printf(0, 32, OLED_6X8, "D:%d/%dcm",
                    (int)(Drive1_GetDistanceMm() * 0.1f),
                    (int)(DRIVE1_TARGET_MM * 0.1f));
        OLED_Printf(0, 40, OLED_6X8, "ERR:%d L:%d R:%d", line->error, left, right);
    } else if (mode == MODE_REMOTE) {
        const char *rcStr;
        switch (gRcState) {
        case RC_IDLE:   rcStr = "IDLE";  break;
        case RC_FWD:    rcStr = "FWD";   break;
        case RC_BWD:    rcStr = "BWD";   break;
        case RC_CW:     rcStr = "CW";    break;
        case RC_CCW:    rcStr = "CCW";   break;
        case RC_TURN:   rcStr = "TURN";  break;
        case RC_TEST_L: rcStr = "TST-L"; break;
        case RC_TEST_R: rcStr = "TST-R"; break;
        default:        rcStr = "????";  break;
        }
        OLED_Printf(0, 24, OLED_6X8, "RC:%s SP:%d", rcStr, gLineSpeed);
        OLED_Printf(0, 32, OLED_6X8, "Tgt:%+.1f L:%d R:%d", gRcTargetYaw, left, right);
        OLED_Printf(0, 40, OLED_6X8, "Yaw:%+.1f", gMpuYaw);
    }

    /* y=48: TIME for non-ball modes */
    if ((mode != MODE_BALL_LINE) && (mode != MODE_BALL_ROUTE) &&
        (mode != MODE_BALL_AB20) && (mode != MODE_ABCD) &&
        (mode != MODE_HF_ZERO) && (mode != MODE_HOLD6) &&
        (mode != MODE_006) && (mode != MODE_007) &&
        (mode != MODE_5NEW) && (mode != MODE_006_1)) {
        uint32_t totalTenths = gCarElapsedMs / 100U;
        uint32_t seconds = totalTenths / 10U;
        uint32_t tenths = totalTenths % 10U;
        OLED_Printf(0, 48, OLED_6X8, "time:%lu.%lus",
                    (unsigned long)seconds,
                    (unsigned long)tenths);
    }
    if (mode == MODE_LINE_TRACK) {
        /* Current average distance / automatic-stop target. */
        OLED_Printf(0, 56, OLED_6X8, "D:%.1f/%.0fcm",
                    (double)(EncoderDistance_GetAverageMm() * 0.1f),
                    (double)(LINE_TRACK_DISPLAY_TARGET_MM * 0.1f));
    }
    if ((mode != MODE_BALL_LINE) && (mode != MODE_BALL_ROUTE) &&
               (mode != MODE_BALL_AB20) && (mode != MODE_ABCD) &&
               (mode != MODE_HF_ZERO) && (mode != MODE_HOLD6) &&
               (mode != MODE_006) && (mode != MODE_007) &&
        (mode != MODE_5NEW) && (mode != MODE_006_1)) {
        OLED_Printf(0, 56, OLED_6X8, "D:%.0f/%.0fcm",
                    (double)(EncoderDistance_GetLeftMm() * 0.1f),
                    (double)(EncoderDistance_GetRightMm() * 0.1f));
    }
    /* y=56: TIME for ball modes (non-ball already shows TIME at y=48) */
    if ((mode == MODE_BALL_LINE) || (mode == MODE_BALL_ROUTE) ||
        (mode == MODE_BALL_AB20) || (mode == MODE_ABCD) ||
        (mode == MODE_HF_ZERO) || (mode == MODE_HOLD6) ||
        (mode == MODE_006) || (mode == MODE_007) ||
        (mode == MODE_5NEW) ||
        (mode == MODE_006_1)) {
        uint32_t totalTenths = gCarElapsedMs / 100U;
        uint32_t seconds = totalTenths / 10U;
        uint32_t tenths = totalTenths % 10U;
        OLED_Printf(0, 56, OLED_6X8, "time:%lu.%lus",
                    (unsigned long)seconds,
                    (unsigned long)tenths);
    }

    OLED_Update();
}

/* ===== 主函数 =====
 * 硬件平台: MSPM0G3507 (128KB Flash, 32KB SRAM)
 * 硬件接线要点(实际物理接线):
 *   - 左轮电机 = TB6612 B通道 (PWMB=PB21, BIN1=PA13, BIN2=PA12)
 *   - 右轮电机 = TB6612 A通道 (PWMA=PB22, AIN1=PB0,  AIN2=PB1)
 *   - 灰度传感器: Gray_1(PA30)=最右 ~ Gray_8(PB18)=最左
 *   - 相机/K230D: UART1 (PB6=TX, PB7=RX, 115200)
 *   - 步进电机驱动: UART2 (PB15=TX, PB16=RX, 115200)
 *   - OLED: I2C (SCL=PA31, SDA=PA28)
 *   - 按键: S1=PA25, S2=PA26, S3=PB23, S4=PA29
 * 模式切换: S2按键循环 TRACK -> HFZERO -> ZERO -> +5>-5 -> DRIVE1 -> HOLD6
 *   -> 006 -> 007 -> 5NEW -> 006(1) -> AB -> AB20 -> ABCD -> TRACK50 -> REMOTE -> TRACK
 *   (完整说明见 docs/task-mode-map.md) */
int main(void)
{
    LineTracker_State line = {0U, 0, 0U};
    int8_t  leftSpeed   = 0;
    int8_t  rightSpeed  = 0;
    /* leftSpeed/rightSpeed 是main局部变量, gLeftSpeed/gRightSpeed是全局镜像 */
    /* 在串口回调和遥测中使用全局变量 */
    uint8_t displayTick = DISPLAY_PERIOD_MS;

    SYSCFG_DL_init();
    Timebase_Init();  /* SysTick 1ms时基，供按键、编码器和运行计时使用 */
    BallBalance_Init(); /* 钢球位置PID；必须在Timebase_Init之后 */
    ZeroControl_Init();
    BallRoute_Init();
    ModeAB_Init();
    ABCD_Init();
    Mode6_Init();
    Mode006_Init();
    Mode0061_Init();
    Mode007_Init();
    Route5New_Init();
    HFZero_Init();
    EncoderDistance_Init();   /* 编码器: GPIO上升沿中断2倍频计程+测速, 必须在Timebase之后 */
    TB6612_Init();
    PID_Init(&gLinePid, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT,
             PID_CORRECTION_LIMIT);
    OLED_Init();

    OLED_Clear();
    OLED_ShowString(0, 0, "No Gyro", OLED_8X16);
    OLED_Update();
    Delay_ms(300);

    /*
     * UART2/PB15/PB16 is assigned to the stepper motor.  The legacy
     * wireless serial driver remains disabled in this build.
     */

    /* ===== K230D相机串口 (UART1: PB6=TX, PB7=RX) ===== */
    OLED_Clear();
    OLED_ShowString(0, 0, "ESP32 WiFi...", OLED_8X16);
    OLED_Update();
    OLED_ShowString(0, 18, "Camera UART Ready", OLED_6X8);
    OLED_ShowString(0, 28, "UART0 PA10/PA11", OLED_6X8);
    ESP32Wifi_Init();
    ESP32Wifi_QuickStart();
    OLED_Update();
    Delay_ms(2000);

    AB_Init();
    Drive1_Init();
    Mode006_Init();
    Mode0061_Init();
    Mode007_Init();
    Route5New_Init();
    updateDisplay(&line, leftSpeed, rightSpeed);

    while (1) {
        /* Highest-priority cooperative task, released by SysTick every 5 ms. */
        if (Timebase_TakeBallControlTick() != 0U) {
            BallBalance_Update();
            if ((AB_GetMode() == MODE_BALL_LINE) ||
                (AB_GetMode() == MODE_BALL_ROUTE) ||
                (AB_GetMode() == MODE_5NEW) ||
                (AB_GetMode() == MODE_ABCD)) {
                ZeroControl_Update();
            }
            if (AB_GetMode() == MODE_BALL_ROUTE) {
                BallRoute_Update();
            }
            if (AB_GetMode() == MODE_5NEW) {
                Route5New_Update();
            }
            if ((AB_GetMode() == MODE_HF_ZERO) ||
                (AB_GetMode() == MODE_AB) ||
                (AB_GetMode() == MODE_BALL_AB20) ||
                (AB_GetMode() == MODE_006_1) ||
                (AB_GetMode() == MODE_HOLD6) ||
                (AB_GetMode() == MODE_006) ||
                (AB_GetMode() == MODE_007)) {
                HFZero_SetStandaloneOscillationMode(
                    AB_GetMode() == MODE_HF_ZERO ||
                    AB_GetMode() == MODE_HOLD6);
                HFZero_Update();
            }
        }
        Key_Event event = Key_GetEvent();

        if (event == KEY_S2) {
            /* S2 (PA26): 切换模式 */
            AB_SwitchMode();
            EncoderDistance_Reset(); /* 切换模式后，编码器距离重新从0开始 */
            BallBalance_SetEnabled(false);
            BallRoute_Stop();
            ModeAB_Stop();
            ABCD_Stop();
            Mode6_Stop();
            Mode006_Stop();
            Mode0061_Stop();
            Mode007_Stop();
            Route5New_Stop();
            HFZero_SetEnabled(false);
            ZeroControl_SetEnabled(false);
            Drive1_Init();
            gRunning       = 0U;
            gRcState       = RC_IDLE;
            gLineDriveSpeed = gLineSpeed;
            leftSpeed      = 0;
            rightSpeed     = 0;
            PID_Reset(&gLinePid);

        } else if (event == KEY_S1) {
            /* S1 (PA25): 启动/停止 */
            if (AB_GetMode() == MODE_LINE_TRACK) {
                if (gRunning == 0U) {
                    EncoderDistance_Reset();
                    gLineDriveSpeed = gLineSpeed;
                    gLineTrackStopMm = LINE_TRACK_STOP_TRIGGER_MM + gEncoderAdjustMm;
                }
                gRunning = (uint8_t)(gRunning == 0U);
                if (gRunning == 0U) {
                    leftSpeed  = 0;
                    rightSpeed = 0;
                    PID_Reset(&gLinePid);
                }
            } else if (AB_GetMode() == MODE_AB) {
                AB_OnStartStop();
            } else if (AB_GetMode() == MODE_BALL_LINE) {
                if (!ZeroControl_IsEnabled()) {
                    ZeroControl_SetTargetMm(0.0f);
                }
                ZeroControl_Toggle();
            } else if (AB_GetMode() == MODE_BALL_ROUTE) {
                BallRoute_Toggle();
            } else if (AB_GetMode() == MODE_BALL_AB20) {
                ModeAB_OnStartStop();
            } else if (AB_GetMode() == MODE_ABCD) {
                ABCD_OnStartStop();
            } else if (AB_GetMode() == MODE_HOLD6) {
                Mode6_OnStartStop();
            } else if (AB_GetMode() == MODE_006) {
                Mode006_OnStartStop();
            } else if (AB_GetMode() == MODE_007) {
                Mode007_OnStartStop();
            } else if (AB_GetMode() == MODE_5NEW) {
                Route5New_Toggle();
            } else if (AB_GetMode() == MODE_006_1) {
                Mode0061_OnStartStop();
            } else if (AB_GetMode() == MODE_HF_ZERO) {
                HFZero_Toggle();
            } else if (AB_GetMode() == MODE_DRIVE1) {
                Drive1_OnStartStop();
            }

        } else if (event == KEY_S3) {
            /* S3: 编码器距离微调+0.5cm */
            if ((AB_GetMode() == MODE_BALL_LINE) ||
                (AB_GetMode() == MODE_HOLD6)) {
                ZeroControl_AdjustPositionKp(0.05f);
            } else if (AB_GetMode() == MODE_HF_ZERO) {
                HFZero_AdjustPositionGain(0.02f);
            } else {
                gEncoderAdjustMm += ENCODER_ADJUST_STEP_MM;
            }

        } else if (event == KEY_S4) {
            /* S4: 编码器距离微调-0.5cm */
            if ((AB_GetMode() == MODE_BALL_LINE) ||
                (AB_GetMode() == MODE_HOLD6)) {
                ZeroControl_AdjustPositionKp(-0.05f);
            } else if (AB_GetMode() == MODE_HF_ZERO) {
                HFZero_AdjustPositionGain(-0.02f);
            } else {
                gEncoderAdjustMm -= ENCODER_ADJUST_STEP_MM;
            }
        }

        /* 编码器测速测距更新 (内部按10ms窗口节流, 用实际毫秒计时,
         * OLED刷新阻塞不会丢脉冲也不会卡死) */
        EncoderDistance_Update();

        /* 读取灰度传感器 */
        LineTracker_Read(&line);

        /* ============================================================
         *  根据模式执行控制
         * ============================================================ */
        if (AB_GetMode() == MODE_LINE_TRACK) {
            if (gRunning != 0U) {
                {
                    float remainingMm = gLineTrackStopMm -
                                        EncoderDistance_GetAverageMm();
                    if (remainingMm <= 0.0f) {
                        gLineDriveSpeed = LINE_TRACK_MIN_SPEED;
                    } else if (remainingMm < LINE_TRACK_SLOW_REMAIN_MM) {
                        float speedRange = (float)(gLineSpeed -
                                                   LINE_TRACK_MIN_SPEED);
                        gLineDriveSpeed = (int8_t)(
                            (float)LINE_TRACK_MIN_SPEED +
                            speedRange * remainingMm /
                            LINE_TRACK_SLOW_REMAIN_MM);
                    } else {
                        gLineDriveSpeed = gLineSpeed;
                    }
                }
                if (EncoderDistance_GetAverageMm() >=
                     gLineTrackStopMm) {
                    /* Remove drive before the physical target to compensate
                     * for the measured 14 cm of coasting distance. */
                    gRunning = 0U;
                    leftSpeed = 0;
                    rightSpeed = 0;
                    TB6612_Stop();
                    PID_Reset(&gLinePid);
                } else {
                    updateMotor(&line, &leftSpeed, &rightSpeed);
                }
            } else {
                TB6612_Stop();
                leftSpeed  = 0;
                rightSpeed = 0;
                PID_Reset(&gLinePid);
                gFilteredLineError = 0.0f;
                gLineFilterLastMs = 0U;
                resetWheelSpeedPi();
            }
        } else if (AB_GetMode() == MODE_AB) {
            AB_Update(&line, &leftSpeed, &rightSpeed);
        } else if (AB_GetMode() == MODE_BALL_AB20) {
            ModeAB_Update(&line, &leftSpeed, &rightSpeed);
        } else if (AB_GetMode() == MODE_ABCD) {
            ABCD_Update(&line, &leftSpeed, &rightSpeed);
        } else if (AB_GetMode() == MODE_HOLD6) {
            Mode6_Update(&line, &leftSpeed, &rightSpeed);
        } else if (AB_GetMode() == MODE_006) {
            Mode006_Update(&line, &leftSpeed, &rightSpeed);
        } else if (AB_GetMode() == MODE_007) {
            Mode007_Update(&line, &leftSpeed, &rightSpeed);
        } else if (AB_GetMode() == MODE_006_1) {
            Mode0061_Update(&line, &leftSpeed, &rightSpeed);
        } else if (AB_GetMode() == MODE_HF_ZERO) {
            TB6612_Stop();
            leftSpeed = 0;
            rightSpeed = 0;
        } else if (AB_GetMode() == MODE_DRIVE1) {
            Drive1_Update(&line, &leftSpeed, &rightSpeed);
        } else if (AB_GetMode() == MODE_REMOTE) {
            /* 遥控模式: 根据gRcState驱动电机 */
            if (gRcState == RC_IDLE) {
                TB6612_Stop();  leftSpeed = 0;  rightSpeed = 0;
            } else if (gRcState == RC_FWD) {
                leftSpeed = gLineSpeed;  rightSpeed = gLineSpeed;
                TB6612_SetSpeed(leftSpeed, rightSpeed);
            } else if (gRcState == RC_BWD) {
                leftSpeed = -gLineSpeed; rightSpeed = -gLineSpeed;
                TB6612_SetSpeed(leftSpeed, rightSpeed);
            } else if (gRcState == RC_CW) {
                leftSpeed = gLineSpeed;  rightSpeed = -gLineSpeed;
                TB6612_SetSpeed(leftSpeed, rightSpeed);
            } else if (gRcState == RC_CCW) {
                leftSpeed = -gLineSpeed; rightSpeed = gLineSpeed;
                TB6612_SetSpeed(leftSpeed, rightSpeed);
            } else if (gRcState == RC_TURN) {
                float yawErr = rc_wrapAngle(gRcTargetYaw - gMpuYaw);
                if (yawErr < 0.0f) yawErr = -yawErr;
                if (yawErr < RC_TURN_TOL) {
                    gRcState = RC_IDLE;
                    TB6612_Stop();  leftSpeed = 0;  rightSpeed = 0;
                } else {
                    float dir = rc_wrapAngle(gRcTargetYaw - gMpuYaw);
                    if (dir > 0.0f) {
                        leftSpeed = gLineSpeed;  rightSpeed = -gLineSpeed;
                    } else {
                        leftSpeed = -gLineSpeed; rightSpeed = gLineSpeed;
                    }
                    TB6612_SetSpeed(leftSpeed, rightSpeed);
                }
            } else if (gRcState == RC_TEST_L) {
                /* 单轮测试: 仅左轮, 验证左编码器方向与测速 */
                leftSpeed = RC_TEST_SPEED;  rightSpeed = 0;
                TB6612_SetSpeed(leftSpeed, rightSpeed);
            } else if (gRcState == RC_TEST_R) {
                /* 单轮测试: 仅右轮, 验证右编码器方向与测速 */
                leftSpeed = 0;  rightSpeed = RC_TEST_SPEED;
                TB6612_SetSpeed(leftSpeed, rightSpeed);
            }
        }

        /* ===== 无线串口发送遥测数据 (每10ms一帧) ===== */
        updateCarTimer();

#if 0 /* Legacy UART3 telemetry disabled: UART3 now drives the stepper. */
        if (gWlTick == 0U) {
            uint8_t modeVal = (uint8_t)AB_GetMode();
            uint8_t stateVal = 0U;
            /* 获取当前模式的状态值 */
            if (modeVal == (uint8_t)MODE_LINE_TRACK || modeVal == (uint8_t)MODE_TRACK50) {
                stateVal = (gRunning != 0U) ? 1U : 0U;
            } else if (modeVal == (uint8_t)MODE_AB) {
                stateVal = (uint8_t)AB_GetState();
            } else if (modeVal == (uint8_t)MODE_REMOTE) {
                stateVal = (uint8_t)gRcState;
            } else if (modeVal == (uint8_t)MODE_DRIVE1) {
                stateVal = (Drive1_IsRunning() != 0U) ? 1U : 0U;
            } else {
                stateVal = 0U;
            }
            /* 遥测帧附带编码器实测速度/里程 (非阻塞发送, 不卡主循环) */
            WL_SendTelemetry(gMpuYaw, line.error, leftSpeed, rightSpeed,
                             gLinePid.kp, gLinePid.ki, gLinePid.kd,
                             modeVal, stateVal, line.activeMask, gLineSpeed,
                             EncoderDistance_GetLeftMmPs() * 0.1f,
                             EncoderDistance_GetRightMmPs() * 0.1f,
                             EncoderDistance_GetLeftMm() * 0.1f,
                             EncoderDistance_GetRightMm() * 0.1f);
            gWlTick = WL_TELEMETRY_PERIOD_MS;
        }
        --gWlTick;
#endif

        /* 刷新显示 */
        if (displayTick == 0U) {
            updateDisplay(&line, leftSpeed, rightSpeed);
            displayTick = DISPLAY_PERIOD_MS;
        }
        --displayTick;

        /*
         * OLED transmission may span a control release.  Service a pending
         * ball-control tick before entering the intentional 1 ms delay.
         */
        if (Timebase_TakeBallControlTick() != 0U) {
            BallBalance_Update();
            if ((AB_GetMode() == MODE_BALL_LINE) ||
                (AB_GetMode() == MODE_BALL_ROUTE) ||
                (AB_GetMode() == MODE_5NEW) ||
                (AB_GetMode() == MODE_ABCD)) {
                ZeroControl_Update();
            }
            if (AB_GetMode() == MODE_BALL_ROUTE) {
                BallRoute_Update();
            }
            if (AB_GetMode() == MODE_5NEW) {
                Route5New_Update();
            }
            if ((AB_GetMode() == MODE_HF_ZERO) ||
                (AB_GetMode() == MODE_AB) ||
                (AB_GetMode() == MODE_BALL_AB20) ||
                (AB_GetMode() == MODE_006) ||
                (AB_GetMode() == MODE_007)) {
                HFZero_SetStandaloneOscillationMode(
                    AB_GetMode() == MODE_HF_ZERO);
                HFZero_Update();
            }
        }
        Delay_ms(1U);
    }
}
