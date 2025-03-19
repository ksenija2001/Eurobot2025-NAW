################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../TOF/VL53LMZ_Driver/Src/vl53lmz_api.c \
../TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_cnh.c \
../TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_detection_thresholds.c \
../TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_motion_indicator.c \
../TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_xtalk.c 

OBJS += \
./TOF/VL53LMZ_Driver/Src/vl53lmz_api.o \
./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_cnh.o \
./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_detection_thresholds.o \
./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_motion_indicator.o \
./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_xtalk.o 

C_DEPS += \
./TOF/VL53LMZ_Driver/Src/vl53lmz_api.d \
./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_cnh.d \
./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_detection_thresholds.d \
./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_motion_indicator.d \
./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_xtalk.d 


# Each subdirectory must supply rules for building sources it contributes
TOF/VL53LMZ_Driver/Src/%.o TOF/VL53LMZ_Driver/Src/%.su TOF/VL53LMZ_Driver/Src/%.cyclo: ../TOF/VL53LMZ_Driver/Src/%.c TOF/VL53LMZ_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/Core/Lib/PointCloud/Inc" -I"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/Core/Lib/Vector/Inc" -I"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/TOF/Target/Inc" -I"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/TOF/Platform" -I"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/TOF/VL53LMZ_Driver/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-TOF-2f-VL53LMZ_Driver-2f-Src

clean-TOF-2f-VL53LMZ_Driver-2f-Src:
	-$(RM) ./TOF/VL53LMZ_Driver/Src/vl53lmz_api.cyclo ./TOF/VL53LMZ_Driver/Src/vl53lmz_api.d ./TOF/VL53LMZ_Driver/Src/vl53lmz_api.o ./TOF/VL53LMZ_Driver/Src/vl53lmz_api.su ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_cnh.cyclo ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_cnh.d ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_cnh.o ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_cnh.su ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_detection_thresholds.cyclo ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_detection_thresholds.d ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_detection_thresholds.o ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_detection_thresholds.su ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_motion_indicator.cyclo ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_motion_indicator.d ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_motion_indicator.o ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_motion_indicator.su ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_xtalk.cyclo ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_xtalk.d ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_xtalk.o ./TOF/VL53LMZ_Driver/Src/vl53lmz_plugin_xtalk.su

.PHONY: clean-TOF-2f-VL53LMZ_Driver-2f-Src

