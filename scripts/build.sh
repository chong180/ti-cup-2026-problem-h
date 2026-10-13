#!/bin/bash
# ============================================================================
# 2026 电赛 H 题固件 — 命令行一键编译 (TI Arm Clang + MSPM0 SDK)
# 等价于 CCS 的 Debug 构建; 可用环境变量覆盖工具链路径:
#   CG_TOOL_ROOT / MSPM0_SDK_ROOT / SYSCONFIG_ROOT
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
FW="$REPO_ROOT/firmware"
DBG="$FW/Debug"

CG_TOOL_ROOT="${CG_TOOL_ROOT:-/c/ti/ti_cgt_arm_llvm_4.0.2.LTS}"
MSPM0_SDK_ROOT="${MSPM0_SDK_ROOT:-/c/ti/mspm0_sdk_2_10_00_04}"
SYSCONFIG_ROOT="${SYSCONFIG_ROOT:-/c/ti/sysconfig_1.26.2}"

CC="$CG_TOOL_ROOT/bin/tiarmclang.exe"
STARTUP="$MSPM0_SDK_ROOT/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c"
EXE="BallBalanceCar.out"

[ -x "$CC" ] || { echo "ERROR: compiler not found: $CC"; exit 1; }
[ -f "$STARTUP" ] || { echo "ERROR: startup file not found (check MSPM0_SDK_ROOT)"; exit 1; }

# 编译参数 (与 Debug/subdir_rules.mk 保持一致)
CFLAGS=(-c @device.opt -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft
        -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP)
INCS=(-I.. -I../app -I../drivers -I../ball_balance -I../legacy
      -I../legacy/mpu6050 -I.
      "-I$MSPM0_SDK_ROOT/source/third_party/CMSIS/Core/Include"
      "-I$MSPM0_SDK_ROOT/source")
LFLAGS=(@device.opt -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft
        -mlittle-endian -mthumb -gdwarf-3
        -Wl,-i"$MSPM0_SDK_ROOT/source" -Wl,-i"$CG_TOOL_ROOT/lib"
        -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections
        -Wl,--rom_model)

# 源码清单: "相对 Debug/ 的路径"
SOURCES=(
  ../main.c
  ../app/sys_mode.c ../app/mode_ab20.c ../app/mode_full_loop.c
  ../app/mode_arc_trajectory.c ../app/mode_loop_arc.c
  ../app/mode_loop_arc_stop.c ../app/mode_loop_offset.c
  ../app/mode_fixed_distance.c ../app/mode_fast_track.c
  ../app/mode_low_speed_balance.c
  ../ball_balance/ball_balance.c ../ball_balance/ball_zero_control.c
  ../ball_balance/ball_route.c ../ball_balance/ball_route_ramp.c
  ../ball_balance/ball_hf_zero.c ../ball_balance/ball_breakaway.c
  ../drivers/motor_tb6612.c ../drivers/line_tracker.c
  ../drivers/encoder_odometry.c ../drivers/encoder_speed.c
  ../drivers/oled.c ../drivers/oled_font.c ../drivers/key.c
  ../drivers/esp32_camera_link.c ../drivers/stepper_emm_v5.c
  ../drivers/pid.c ../drivers/delay.c ../drivers/timebase.c
  ../legacy/gyro_uart.c
  ti_msp_dl_config.c
)

cd "$DBG"
echo "==> [1/2] compiling ${#SOURCES[@]} sources + startup ..."
OBJS=()
for src in "${SOURCES[@]}"; do
  obj="${src#../}"                     # 去掉前导 ../ (ti_msp_dl_config.c 不受影响)
  obj="${obj%.c}.o"
  mkdir -p "$(dirname "$obj")"
  "$CC" "${CFLAGS[@]}" "${INCS[@]}" -o "$obj" "$src"
  OBJS+=("$obj")
done
"$CC" "${CFLAGS[@]}" "${INCS[@]}" -o startup_mspm0g350x_ticlang.o "$STARTUP"
OBJS+=(startup_mspm0g350x_ticlang.o)

echo "==> [2/2] linking $EXE ..."
"$CC" "${LFLAGS[@]}" \
  -Wl,-m"$EXE.map" \
  -Wl,-xml_link_info="${EXE%.out}_linkInfo.xml" \
  -o "$EXE" "${OBJS[@]}" \
  -Wl,-ldevice_linker.cmd -Wl,-ldevice.cmd.genlibs -Wl,-llibc.a

SIZE=$(stat -c%s "$EXE")
echo ""
echo "BUILD OK -> $DBG/$EXE ($SIZE bytes)"
echo "烧录: CCS 导入工程后 J-Link 下载, 或使用 ti-flash 工具直接烧写 $EXE"
