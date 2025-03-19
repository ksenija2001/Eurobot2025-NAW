################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../TOF/Platform/platform.c 

OBJS += \
./TOF/Platform/platform.o 

C_DEPS += \
./TOF/Platform/platform.d 


# Each subdirectory must supply rules for building sources it contributes
TOF/Platform/%.o TOF/Platform/%.su TOF/Platform/%.cyclo: ../TOF/Platform/%.c TOF/Platform/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/Core/Lib/PointCloud/Inc" -I"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/Core/Lib/Vector/Inc" -I"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/TOF/Target/Inc" -I"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/TOF/Platform" -I"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/TOF/VL53LMZ_Driver/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-TOF-2f-Platform

clean-TOF-2f-Platform:
	-$(RM) ./TOF/Platform/platform.cyclo ./TOF/Platform/platform.d ./TOF/Platform/platform.o ./TOF/Platform/platform.su

.PHONY: clean-TOF-2f-Platform

