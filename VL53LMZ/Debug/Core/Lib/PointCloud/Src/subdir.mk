################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/PointCloud/Src/point_cloud.c 

OBJS += \
./Core/Lib/PointCloud/Src/point_cloud.o 

C_DEPS += \
./Core/Lib/PointCloud/Src/point_cloud.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/PointCloud/Src/%.o Core/Lib/PointCloud/Src/%.su Core/Lib/PointCloud/Src/%.cyclo: ../Core/Lib/PointCloud/Src/%.c Core/Lib/PointCloud/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/Core/Lib/PointCloud/Inc" -I"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/Core/Lib/Vector/Inc" -I"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/TOF/Target/Inc" -I"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/TOF/Platform" -I"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/TOF/VL53LMZ_Driver/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-PointCloud-2f-Src

clean-Core-2f-Lib-2f-PointCloud-2f-Src:
	-$(RM) ./Core/Lib/PointCloud/Src/point_cloud.cyclo ./Core/Lib/PointCloud/Src/point_cloud.d ./Core/Lib/PointCloud/Src/point_cloud.o ./Core/Lib/PointCloud/Src/point_cloud.su

.PHONY: clean-Core-2f-Lib-2f-PointCloud-2f-Src

