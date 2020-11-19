################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
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

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
UNITED_BLOCK_04_09_2020_Clear.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
UNITED_BLOCK_04_09_2020_Clear.list \

OBJCOPY_BIN += \
UNITED_BLOCK_04_09_2020_Clear.bin \


# All Target
all: UNITED_BLOCK_04_09_2020_Clear.elf secondary-outputs

# Tool invocations
UNITED_BLOCK_04_09_2020_Clear.elf: $(OBJS) $(USER_OBJS) D:\cytometr_firrmware-alpha\UNITED_BLOCK_04_09_2020_Clear\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "UNITED_BLOCK_04_09_2020_Clear.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\cytometr_firrmware-alpha\UNITED_BLOCK_04_09_2020_Clear\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="UNITED_BLOCK_04_09_2020_Clear.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

UNITED_BLOCK_04_09_2020_Clear.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "UNITED_BLOCK_04_09_2020_Clear.list"
	@echo 'Finished building: $@'
	@echo ' '

UNITED_BLOCK_04_09_2020_Clear.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "UNITED_BLOCK_04_09_2020_Clear.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
