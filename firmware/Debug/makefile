################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# ============================================================================
# Toolchain locations (override on the command line or via environment):
#   make CG_TOOL_ROOT=D:/ti/ti_cgt_arm_llvm_4.0.2.LTS
# ============================================================================
CG_TOOL_ROOT   ?= C:/ti/ti_cgt_arm_llvm_4.0.2.LTS
MSPM0_SDK_ROOT ?= C:/ti/mspm0_sdk_2_10_00_04
SYSCONFIG_ROOT ?= C:/ti/sysconfig_1.26.2

CC := "$(CG_TOOL_ROOT)/bin/tiarmclang.exe"

GEN_OPTS__FLAG := @"./device.opt"
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd"

ORDERED_OBJS += \
"./main.o" \
"./app/sys_mode.o" \
"./app/mode_ab20.o" \
"./app/mode_full_loop.o" \
"./app/mode_arc_trajectory.o" \
"./app/mode_loop_arc.o" \
"./app/mode_loop_arc_stop.o" \
"./app/mode_loop_offset.o" \
"./app/mode_fixed_distance.o" \
"./app/mode_fast_track.o" \
"./app/mode_low_speed_balance.o" \
"./ball_balance/ball_balance.o" \
"./ball_balance/ball_zero_control.o" \
"./ball_balance/ball_route.o" \
"./ball_balance/ball_route_ramp.o" \
"./ball_balance/ball_hf_zero.o" \
"./ball_balance/ball_breakaway.o" \
"./drivers/motor_tb6612.o" \
"./drivers/line_tracker.o" \
"./drivers/encoder_odometry.o" \
"./drivers/encoder_speed.o" \
"./drivers/oled.o" \
"./drivers/oled_font.o" \
"./drivers/key.o" \
"./drivers/esp32_camera_link.o" \
"./drivers/stepper_emm_v5.o" \
"./drivers/pid.o" \
"./drivers/delay.o" \
"./drivers/timebase.o" \
"./legacy/gyro_uart.o" \
"./ti_msp_dl_config.o" \
"./startup_mspm0g350x_ticlang.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "BallBalanceCar.out"

# Tool invocations
BallBalanceCar.out: $(OBJS) $(GEN_CMDS)
	@echo 'Arm Linker - building target: "$@"'
	$(CC) @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -gdwarf-3 -Wl,-m"BallBalanceCar.map" -Wl,-i"$(MSPM0_SDK_ROOT)/source" -Wl,-i".." -Wl,-i"./syscfg" -Wl,-i"$(CG_TOOL_ROOT)/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="BallBalanceCar_linkInfo.xml" -Wl,--rom_model -o "BallBalanceCar.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS__QUOTED) 
	-$(RM) "device.opt" "device_linker.cmd" "device.cmd.genlibs" "ti_msp_dl_config.c" "ti_msp_dl_config.h" "Event.dot"
	-$(RM) "BallBalanceCar.out" "BallBalanceCar.map" "BallBalanceCar_linkInfo.xml"
	-$(RM) $(C_DEPS__QUOTED)
	-@echo ' '

.PHONY: all clean
.SECONDARY:
