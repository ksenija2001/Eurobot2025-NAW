################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include TOF/VL53LMZ_Driver/Src/subdir.mk
-include TOF/Target/Src/subdir.mk
-include TOF/Platform/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Core/Lib/Vector/Src/subdir.mk
-include Core/Lib/PointCloud/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := VL53LMZ
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
VL53LMZ.elf \

MAP_FILES += \
VL53LMZ.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
VL53LMZ.list \


# All Target
all: main-build

# Main-build Target
main-build: VL53LMZ.elf secondary-outputs

# Tool invocations
VL53LMZ.elf VL53LMZ.map: $(OBJS) $(USER_OBJS) /home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/STM32F401RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "VL53LMZ.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/STM32F401RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="VL53LMZ.map" -Wl,--gc-sections -static -L"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/TOF/VL53LMZ_Driver/Inc" -L"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/Core/Lib/PointCloud/Inc" -L"/home/xenia/STM32CubeIDE/workspace_1.14.0/VL53LMZ/Core/Lib/Vector/Inc" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

VL53LMZ.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "VL53LMZ.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) VL53LMZ.elf VL53LMZ.list VL53LMZ.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
