################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes

./main.o: ../main.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./main.d" -MT"./main.o" $(GEN_OPTS__FLAG) -o"./main.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./app/sys_mode.o: ../app/sys_mode.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./app/sys_mode.d" -MT"./app/sys_mode.o" $(GEN_OPTS__FLAG) -o"./app/sys_mode.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./app/mode_ab20.o: ../app/mode_ab20.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./app/mode_ab20.d" -MT"./app/mode_ab20.o" $(GEN_OPTS__FLAG) -o"./app/mode_ab20.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./app/mode_full_loop.o: ../app/mode_full_loop.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./app/mode_full_loop.d" -MT"./app/mode_full_loop.o" $(GEN_OPTS__FLAG) -o"./app/mode_full_loop.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./app/mode_arc_trajectory.o: ../app/mode_arc_trajectory.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./app/mode_arc_trajectory.d" -MT"./app/mode_arc_trajectory.o" $(GEN_OPTS__FLAG) -o"./app/mode_arc_trajectory.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./app/mode_loop_arc.o: ../app/mode_loop_arc.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./app/mode_loop_arc.d" -MT"./app/mode_loop_arc.o" $(GEN_OPTS__FLAG) -o"./app/mode_loop_arc.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./app/mode_loop_arc_stop.o: ../app/mode_loop_arc_stop.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./app/mode_loop_arc_stop.d" -MT"./app/mode_loop_arc_stop.o" $(GEN_OPTS__FLAG) -o"./app/mode_loop_arc_stop.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./app/mode_loop_offset.o: ../app/mode_loop_offset.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./app/mode_loop_offset.d" -MT"./app/mode_loop_offset.o" $(GEN_OPTS__FLAG) -o"./app/mode_loop_offset.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./app/mode_fixed_distance.o: ../app/mode_fixed_distance.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./app/mode_fixed_distance.d" -MT"./app/mode_fixed_distance.o" $(GEN_OPTS__FLAG) -o"./app/mode_fixed_distance.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./app/mode_fast_track.o: ../app/mode_fast_track.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./app/mode_fast_track.d" -MT"./app/mode_fast_track.o" $(GEN_OPTS__FLAG) -o"./app/mode_fast_track.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./app/mode_low_speed_balance.o: ../app/mode_low_speed_balance.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./app/mode_low_speed_balance.d" -MT"./app/mode_low_speed_balance.o" $(GEN_OPTS__FLAG) -o"./app/mode_low_speed_balance.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./ball_balance/ball_balance.o: ../ball_balance/ball_balance.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./ball_balance/ball_balance.d" -MT"./ball_balance/ball_balance.o" $(GEN_OPTS__FLAG) -o"./ball_balance/ball_balance.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./ball_balance/ball_zero_control.o: ../ball_balance/ball_zero_control.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./ball_balance/ball_zero_control.d" -MT"./ball_balance/ball_zero_control.o" $(GEN_OPTS__FLAG) -o"./ball_balance/ball_zero_control.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./ball_balance/ball_route.o: ../ball_balance/ball_route.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./ball_balance/ball_route.d" -MT"./ball_balance/ball_route.o" $(GEN_OPTS__FLAG) -o"./ball_balance/ball_route.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./ball_balance/ball_route_ramp.o: ../ball_balance/ball_route_ramp.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./ball_balance/ball_route_ramp.d" -MT"./ball_balance/ball_route_ramp.o" $(GEN_OPTS__FLAG) -o"./ball_balance/ball_route_ramp.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./ball_balance/ball_hf_zero.o: ../ball_balance/ball_hf_zero.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./ball_balance/ball_hf_zero.d" -MT"./ball_balance/ball_hf_zero.o" $(GEN_OPTS__FLAG) -o"./ball_balance/ball_hf_zero.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./ball_balance/ball_breakaway.o: ../ball_balance/ball_breakaway.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./ball_balance/ball_breakaway.d" -MT"./ball_balance/ball_breakaway.o" $(GEN_OPTS__FLAG) -o"./ball_balance/ball_breakaway.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./drivers/motor_tb6612.o: ../drivers/motor_tb6612.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./drivers/motor_tb6612.d" -MT"./drivers/motor_tb6612.o" $(GEN_OPTS__FLAG) -o"./drivers/motor_tb6612.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./drivers/line_tracker.o: ../drivers/line_tracker.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./drivers/line_tracker.d" -MT"./drivers/line_tracker.o" $(GEN_OPTS__FLAG) -o"./drivers/line_tracker.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./drivers/encoder_odometry.o: ../drivers/encoder_odometry.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./drivers/encoder_odometry.d" -MT"./drivers/encoder_odometry.o" $(GEN_OPTS__FLAG) -o"./drivers/encoder_odometry.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./drivers/encoder_speed.o: ../drivers/encoder_speed.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./drivers/encoder_speed.d" -MT"./drivers/encoder_speed.o" $(GEN_OPTS__FLAG) -o"./drivers/encoder_speed.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./drivers/oled.o: ../drivers/oled.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./drivers/oled.d" -MT"./drivers/oled.o" $(GEN_OPTS__FLAG) -o"./drivers/oled.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./drivers/oled_font.o: ../drivers/oled_font.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./drivers/oled_font.d" -MT"./drivers/oled_font.o" $(GEN_OPTS__FLAG) -o"./drivers/oled_font.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./drivers/key.o: ../drivers/key.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./drivers/key.d" -MT"./drivers/key.o" $(GEN_OPTS__FLAG) -o"./drivers/key.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./drivers/esp32_camera_link.o: ../drivers/esp32_camera_link.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./drivers/esp32_camera_link.d" -MT"./drivers/esp32_camera_link.o" $(GEN_OPTS__FLAG) -o"./drivers/esp32_camera_link.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./drivers/stepper_emm_v5.o: ../drivers/stepper_emm_v5.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./drivers/stepper_emm_v5.d" -MT"./drivers/stepper_emm_v5.o" $(GEN_OPTS__FLAG) -o"./drivers/stepper_emm_v5.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./drivers/pid.o: ../drivers/pid.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./drivers/pid.d" -MT"./drivers/pid.o" $(GEN_OPTS__FLAG) -o"./drivers/pid.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./drivers/delay.o: ../drivers/delay.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./drivers/delay.d" -MT"./drivers/delay.o" $(GEN_OPTS__FLAG) -o"./drivers/delay.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./drivers/timebase.o: ../drivers/timebase.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./drivers/timebase.d" -MT"./drivers/timebase.o" $(GEN_OPTS__FLAG) -o"./drivers/timebase.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./legacy/gyro_uart.o: ../legacy/gyro_uart.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./legacy/gyro_uart.d" -MT"./legacy/gyro_uart.o" $(GEN_OPTS__FLAG) -o"./legacy/gyro_uart.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./ti_msp_dl_config.o: ./ti_msp_dl_config.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./ti_msp_dl_config.d" -MT"./ti_msp_dl_config.o" $(GEN_OPTS__FLAG) -o"./ti_msp_dl_config.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

./startup_mspm0g350x_ticlang.o: $(MSPM0_SDK_ROOT)/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	$(CC) -c @"device.opt" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -gdwarf-3 -MMD -MP -I".." -I"../app" -I"../drivers" -I"../ball_balance" -I"../legacy" -I"../legacy/mpu6050" -I"." -I"$(MSPM0_SDK_ROOT)/source/third_party/CMSIS/Core/Include" -I"$(MSPM0_SDK_ROOT)/source" -MF"./startup_mspm0g350x_ticlang.d" -MT"./startup_mspm0g350x_ticlang.o" $(GEN_OPTS__FLAG) -o"./startup_mspm0g350x_ticlang.o" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


build-474970721: ../board.syscfg
	@echo 'SysConfig - building file: "$<"'
	"$(SYSCONFIG_ROOT)/sysconfig_cli.bat" -s "$(MSPM0_SDK_ROOT)/.metadata/product.json" --script "../board.syscfg" -o "." --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-474970721 ../board.syscfg
device.opt: build-474970721
device.cmd.genlibs: build-474970721
ti_msp_dl_config.c: build-474970721
ti_msp_dl_config.h: build-474970721
Event.dot: build-474970721
