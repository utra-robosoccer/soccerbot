
RM := rm -rf
BUILD_DIR = Build
MAKEFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
PROJECT_DIR := $(CURDIR)
COMMON_DIR := $(abspath ../Common)



# Every subdirectory with source files must be described here
SUBDIRS := \
Drivers/STM32F4xx_HAL_Driver/Src \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
Middlewares/Third_Party/FreeRTOS/Source \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang \
Src \
$(COMMON_DIR)/app \
$(COMMON_DIR)/component/CircularDmaBuffer \
$(COMMON_DIR)/component/Communication \
$(COMMON_DIR)/component/DSP \
$(COMMON_DIR)/component/DaisyChain \
$(COMMON_DIR)/component/Dynamixel \
$(COMMON_DIR)/component/MPU6050 \
$(COMMON_DIR)/component/UartDriver \
$(COMMON_DIR)/hardware \
$(COMMON_DIR)/startup \

EXCL_SRCS := $(COMMON_DIR)/hardware/LwipUdpInterface.cpp

C_SRCS := $(filter-out $(EXCL_SRCS),$(wildcard $(addsuffix /*.c,$(SUBDIRS))))
CPP_SRCS := $(filter-out $(EXCL_SRCS),$(wildcard $(addsuffix /*.cpp,$(SUBDIRS))))
C_OBJECTS := $(addprefix $(BUILD_DIR)/,$(patsubst $(COMMON_DIR)/%,%,$(C_SRCS:.c=.o)))
CPP_OBJECTS := $(addprefix $(BUILD_DIR)/,$(patsubst $(COMMON_DIR)/%,%,$(CPP_SRCS:.cpp=.o)))

CFLAGS := -fprofile-arcs -ftest-coverage -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F446xx -Og -g3 -Wall -fmessage-length=0 -ffunction-sections

INC := \
-I"$(PROJECT_DIR)/Inc" \
-I"$(COMMON_DIR)/include" \
-I"$(PROJECT_DIR)/Drivers/STM32F4xx_HAL_Driver/Inc" \
-I"$(PROJECT_DIR)/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" \
-I"$(PROJECT_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Include" \
-I"$(PROJECT_DIR)/Drivers/CMSIS/Include" \
-I"$(PROJECT_DIR)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" \
-I"$(PROJECT_DIR)/Middlewares/Third_Party/FreeRTOS/Source/include" \
-I"$(PROJECT_DIR)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" \


$(BUILD_DIR)/%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	$(info [$(C_SRCS)])
	$(info [$(ROOT_DIR)])
	$(info $(PROJECT_DIR))
	@mkdir -p $(BUILD_DIR)/$(patsubst /%.c,,$*)
	arm-none-eabi-gcc $(CFLAGS) $(INC)  -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
	
$(BUILD_DIR)/%.o: %.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	$(info [$(CPP_SRCS)])
	$(info $(ROOT_DIR))
	$(info $(PROJECT_DIR))
	@mkdir -p $(BUILD_DIR)/$(patsubst /%.cpp,,$*)
	arm-none-eabi-g++ $(CFLAGS) $(INC) -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '	
	
$(BUILD_DIR)/%.o: $(COMMON_DIR)/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@mkdir -p $(BUILD_DIR)/$(patsubst /%.c,,$*)
	arm-none-eabi-gcc $(CFLAGS) $(INC)  -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
	
$(BUILD_DIR)/%.o: $(COMMON_DIR)/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	$(info [$(CPP_SRCS)])
	@mkdir -p $(BUILD_DIR)/$(patsubst /%.cpp,,$*)
	arm-none-eabi-g++ $(CFLAGS) $(INC) -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '	

LIBS := -larm_cortexM4lf_math

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(BUILD_DIR)/Robot_F4.elf

# Tool invocations
$(BUILD_DIR)/Robot_F4.elf: $(C_OBJECTS) $(CPP_OBJECTS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU G++ Linker'
	$(info [$(CPP_OBJECTS)])
	$(info $ROOT_DIR)
	arm-none-eabi-g++ -lgcov --coverage -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -L"Lib"  -specs=nosys.specs -specs=nano.specs -T"STM32F446RETx_FLASH.ld" -Wl,-Map=output.map -Wl,--gc-sections -fno-exceptions -fno-rtti -o "$(BUILD_DIR)/Robot_F4.elf" $(C_OBJECTS) $(CPP_OPJECTS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(BUILD_DIR)/*
	-@echo ' '

post-build:
	-@echo 'Generating hex and Printing size information:'
	arm-none-eabi-objcopy -O ihex "$(BUILD_DIR)/Robot_F4.elf" "$(BUILD_DIR)/Robot_F4.hex"
	arm-none-eabi-size "$(BUILD_DIR)/Robot_F4.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build
