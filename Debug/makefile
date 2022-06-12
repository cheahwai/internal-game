################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/VESC_CAN/subdir.mk
-include Core/Src/TuningInterface/subdir.mk
-include Core/Src/TFmini/subdir.mk
-include Core/Src/STEPPER/subdir.mk
-include Core/Src/SPI/subdir.mk
-include Core/Src/SERVO_DRIVER/subdir.mk
-include Core/Src/SERVO/subdir.mk
-include Core/Src/ROS_Interface/subdir.mk
-include Core/Src/RNS_interface/subdir.mk
-include Core/Src/RGB/subdir.mk
-include Core/Src/PSx_Interface/subdir.mk
-include Core/Src/PID/subdir.mk
-include Core/Src/Moving_Average/subdir.mk
-include Core/Src/MODN/subdir.mk
-include Core/Src/MAIN/subdir.mk
-include Core/Src/LASER/subdir.mk
-include Core/Src/KF/subdir.mk
-include Core/Src/ILI9341/subdir.mk
-include Core/Src/I2C/subdir.mk
-include Core/Src/Faulhaber/subdir.mk
-include Core/Src/Eeprom/subdir.mk
-include Core/Src/ESP32/subdir.mk
-include Core/Src/Dongbei/subdir.mk
-include Core/Src/DTLOG/subdir.mk
-include Core/Src/CAN/subdir.mk
-include Core/Src/BIOS/subdir.mk
-include Core/Src/ADC/subdir.mk
-include Core/Src/ABT/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
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


BUILD_ARTIFACT_NAME := domestic
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
domestic.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
domestic.list \

OBJCOPY_BIN += \
domestic.bin \


# All Target
all: main-build

# Main-build Target
main-build: domestic.elf secondary-outputs

# Tool invocations
domestic.elf: $(OBJS) $(USER_OBJS) /home/wai/Documents/domestic/STM32F407VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "domestic.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/wai/Documents/domestic/STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="domestic.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

domestic.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "domestic.list"
	@echo 'Finished building: $@'
	@echo ' '

domestic.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "domestic.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SIZE_OUTPUT)$(OBJDUMP_LIST)$(EXECUTABLES)$(OBJS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS)$(OBJCOPY_BIN) domestic.elf
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
