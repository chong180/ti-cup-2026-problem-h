################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables
SYSCFG_SRCS += \
../board.syscfg

C_SRCS += \
../main.c \
../app/sys_mode.c \
../app/mode_ab20.c \
../app/mode_full_loop.c \
../app/mode_arc_trajectory.c \
../app/mode_loop_arc.c \
../app/mode_loop_arc_stop.c \
../app/mode_loop_offset.c \
../app/mode_fixed_distance.c \
../app/mode_fast_track.c \
../app/mode_low_speed_balance.c \
../ball_balance/ball_balance.c \
../ball_balance/ball_zero_control.c \
../ball_balance/ball_route.c \
../ball_balance/ball_route_ramp.c \
../ball_balance/ball_hf_zero.c \
../ball_balance/ball_breakaway.c \
../drivers/motor_tb6612.c \
../drivers/line_tracker.c \
../drivers/encoder_odometry.c \
../drivers/encoder_speed.c \
../drivers/oled.c \
../drivers/oled_font.c \
../drivers/key.c \
../drivers/esp32_camera_link.c \
../drivers/stepper_emm_v5.c \
../drivers/pid.c \
../drivers/delay.c \
../drivers/timebase.c \
../legacy/gyro_uart.c \
./ti_msp_dl_config.c \
$(MSPM0_SDK_ROOT)/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c

C_SRCS__QUOTED += \
"../main.c" \
"../app/sys_mode.c" \
"../app/mode_ab20.c" \
"../app/mode_full_loop.c" \
"../app/mode_arc_trajectory.c" \
"../app/mode_loop_arc.c" \
"../app/mode_loop_arc_stop.c" \
"../app/mode_loop_offset.c" \
"../app/mode_fixed_distance.c" \
"../app/mode_fast_track.c" \
"../app/mode_low_speed_balance.c" \
"../ball_balance/ball_balance.c" \
"../ball_balance/ball_zero_control.c" \
"../ball_balance/ball_route.c" \
"../ball_balance/ball_route_ramp.c" \
"../ball_balance/ball_hf_zero.c" \
"../ball_balance/ball_breakaway.c" \
"../drivers/motor_tb6612.c" \
"../drivers/line_tracker.c" \
"../drivers/encoder_odometry.c" \
"../drivers/encoder_speed.c" \
"../drivers/oled.c" \
"../drivers/oled_font.c" \
"../drivers/key.c" \
"../drivers/esp32_camera_link.c" \
"../drivers/stepper_emm_v5.c" \
"../drivers/pid.c" \
"../drivers/delay.c" \
"../drivers/timebase.c" \
"../legacy/gyro_uart.c" \
"./ti_msp_dl_config.c" \
"$(MSPM0_SDK_ROOT)/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c" \

OBJS += \
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
"./startup_mspm0g350x_ticlang.o"

OBJS__QUOTED += \
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

C_DEPS += \
./main.d
./app/sys_mode.d
./app/mode_ab20.d
./app/mode_full_loop.d
./app/mode_arc_trajectory.d
./app/mode_loop_arc.d
./app/mode_loop_arc_stop.d
./app/mode_loop_offset.d
./app/mode_fixed_distance.d
./app/mode_fast_track.d
./app/mode_low_speed_balance.d
./ball_balance/ball_balance.d
./ball_balance/ball_zero_control.d
./ball_balance/ball_route.d
./ball_balance/ball_route_ramp.d
./ball_balance/ball_hf_zero.d
./ball_balance/ball_breakaway.d
./drivers/motor_tb6612.d
./drivers/line_tracker.d
./drivers/encoder_odometry.d
./drivers/encoder_speed.d
./drivers/oled.d
./drivers/oled_font.d
./drivers/key.d
./drivers/esp32_camera_link.d
./drivers/stepper_emm_v5.d
./drivers/pid.d
./drivers/delay.d
./drivers/timebase.d
./legacy/gyro_uart.d
./ti_msp_dl_config.d
./startup_mspm0g350x_ticlang.d

C_DEPS__QUOTED += \
"./main.d" \
"./app/sys_mode.d" \
"./app/mode_ab20.d" \
"./app/mode_full_loop.d" \
"./app/mode_arc_trajectory.d" \
"./app/mode_loop_arc.d" \
"./app/mode_loop_arc_stop.d" \
"./app/mode_loop_offset.d" \
"./app/mode_fixed_distance.d" \
"./app/mode_fast_track.d" \
"./app/mode_low_speed_balance.d" \
"./ball_balance/ball_balance.d" \
"./ball_balance/ball_zero_control.d" \
"./ball_balance/ball_route.d" \
"./ball_balance/ball_route_ramp.d" \
"./ball_balance/ball_hf_zero.d" \
"./ball_balance/ball_breakaway.d" \
"./drivers/motor_tb6612.d" \
"./drivers/line_tracker.d" \
"./drivers/encoder_odometry.d" \
"./drivers/encoder_speed.d" \
"./drivers/oled.d" \
"./drivers/oled_font.d" \
"./drivers/key.d" \
"./drivers/esp32_camera_link.d" \
"./drivers/stepper_emm_v5.d" \
"./drivers/pid.d" \
"./drivers/delay.d" \
"./drivers/timebase.d" \
"./legacy/gyro_uart.d" \
"./ti_msp_dl_config.d" \
"./startup_mspm0g350x_ticlang.d" \

GEN_CMDS += \
./device_linker.cmd

GEN_FILES += \
./device_linker.cmd \
./device.opt \
./device.cmd.genlibs \
./ti_msp_dl_config.c \
./ti_msp_dl_config.h

