#@file    Robot_F4 makefile
#@author  Gokul Dharan
#TODO: make this parseable by Doxygen

#Valid Targets
F4:=F4
F7:=F7
  
#User options
TARGET ?=$(F4)

RM := rm -rf
BUILD_DIR = Build
MAKEFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
PROJECT_DIR := $(CURDIR)
COMMON_DIR := $(abspath ../Common)


# Every subdirectory with source files must be described here
ifeq ($(TARGET),$(F4))
MCU_UNIQUE_SRC_SUBDIRS := \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
Drivers/STM32F4xx_HAL_Driver/Src \
Src/F4 \
startup/F4 
else ifeq ($(TARGET),$(F7))
MCU_UNIQUE_SRC_SUBDIRS := \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 \
Middlewares/Third_Party/LwIP/src/* \
Middlewares/Third_Party/LwIP/system/* \
Drivers/STM32F7xx_HAL_Driver/Src  \
Src/F7 \
startup/F7 
else
$(error MCU_UNIQUE_SRC_SUBDIRS not configured for target $(TARGET))
endif

SUBDIRS := \
$(MCU_UNIQUE_SRC_SUBDIRS) \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
Middlewares/Third_Party/FreeRTOS/Source \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang \
$(COMMON_DIR)/app \
$(COMMON_DIR)/component/CircularDmaBuffer \
$(COMMON_DIR)/component/Communication \
$(COMMON_DIR)/component/DSP \
$(COMMON_DIR)/component/DaisyChain \
$(COMMON_DIR)/component/Dynamixel \
$(COMMON_DIR)/component/MPU6050 \
$(COMMON_DIR)/component/UartDriver \
$(COMMON_DIR)/hardware \

TEST_SUBDIR := \
$(SUBDIRS) \
$(COMMON_DIR)/test \

# Sources to be excluded from the build
ifeq ($(TARGET),$(F4))
EXCL_SRCS :=\
$(COMMON_DIR)/hardware/UdpRawInterfaceImpl.cpp \
%_test.cpp
else ifeq ($(TARGET),$(F7))
EXCL_SRCS :=\
%_test.cpp
else
$(error EXCL_SRCS not configured for target $(TARGET))
endif

# Define all include directories
ifeq ($(TARGET),$(F4))
MCU_UNIQUE_INC :=\
-I"$(PROJECT_DIR)/Drivers/STM32F4xx_HAL_Driver/Inc" \
-I"$(PROJECT_DIR)/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" \
-I"$(PROJECT_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Include" \
-I"$(PROJECT_DIR)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
else ifeq ($(TARGET),$(F7))
MCU_UNIQUE_INC :=\
-I"$(PROJECT_DIR)/Drivers/STM32F7xx_HAL_Driver/Inc" \
-I"$(PROJECT_DIR)/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" \
-I"$(PROJECT_DIR)/Drivers/CMSIS/Device/ST/STM32F7xx/Include" \
-I"$(PROJECT_DIR)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" \
-I"$(PROJECT_DIR)/Middlewares/Third_Party/LwIP/src/include" \
-I"$(PROJECT_DIR)/Middlewares/Third_Party/LwIP/system"
else
$(error MCU_UNIQUE_INC not configured for target $(TARGET))
endif

INC := \
-I"$(PROJECT_DIR)/Inc/$(TARGET)" \
-I"$(COMMON_DIR)/component/" \
-I"$(COMMON_DIR)/hardware" \
-I"$(COMMON_DIR)/include" \
-I"$(COMMON_DIR)/interface" \
-I"$(COMMON_DIR)/mock" \
-I"$(PROJECT_DIR)/Drivers/CMSIS/Include" \
-I"$(PROJECT_DIR)/Middlewares/Third_Party/FreeRTOS/Source/include" \
-I"$(PROJECT_DIR)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" \
$(MCU_UNIQUE_INC)

TEST_INC := \
-I"C:\Users\gokul\Documents\GitHub\soccer-embedded\RobotTest\googletest\googletest\include" \
-I"C:\Users\gokul\Documents\GitHub\soccer-embedded\RobotTest\googletest\googlemock\include" \


# Find all sources in given subdirectories, filter out exclusions, and create objects list
C_SRCS := $(filter-out $(EXCL_SRCS),$(wildcard $(addsuffix /*.c,$(SUBDIRS))))
CPP_SRCS := $(filter-out $(EXCL_SRCS),$(wildcard $(addsuffix /*.cpp,$(SUBDIRS))))
S_SRCS := $(filter-out $(EXCL_SRCS),$(wildcard $(addsuffix /*.s,$(SUBDIRS))))
C_OBJECTS := $(addprefix $(BUILD_DIR)/$(TARGET)/,$(patsubst $(COMMON_DIR)/%,%,$(C_SRCS:.c=.o)))
CPP_OBJECTS := $(addprefix $(BUILD_DIR)/$(TARGET)/,$(patsubst $(COMMON_DIR)/%,%,$(CPP_SRCS:.cpp=.o)))
S_OBJECTS := $(addprefix $(BUILD_DIR)/$(TARGET)/,$(patsubst $(COMMON_DIR)/%,%,$(S_SRCS:.s=.o)))
CPP_TEST_SRCS := $(wildcard $(addsuffix /*_test.cpp,$(TEST_SUBDIR)))
CPP_TEST_INC_SRCS := $(filter-out $(EXCL_SRCS),$(wildcard $(patsubst %_test.cpp,%.cpp,$(CPP_TEST_SRCS))))
CPP_TEST_OBJECTS := $(addprefix $(BUILD_DIR)/$(TARGET)/Test/,$(patsubst $(COMMON_DIR)/%,%,$(CPP_TEST_SRCS:.cpp=.o)))
CPP_TEST_OBJECTS += $(addprefix $(BUILD_DIR)/$(TARGET)/Test/,$(patsubst $(COMMON_DIR)/%,%,$(CPP_TEST_INC_SRCS:.cpp=.o)))
CPP_TEST_OBJECTS += ../gmock/src/Test.o 

# Define compiler flags
ifeq ($(TARGET),$(F4))
MCPU := -mcpu=cortex-m4
MCU_DSTM := -DSTM32F446xx
MCU_FLAG := -mcpu=cortex-m4 -DSTM32F446xx
else ifeq ($(TARGET),$(F7))
MCPU := -mcpu=cortex-m7
MCU_DSTM := -DSTM32F767xx
MCU_FLAG := -mcpu=cortex-m7 -DSTM32F767xx
else
$(error MCU_FLAG not configured for target $(TARGET))
endif

CFLAGS := -fprofile-arcs -ftest-coverage $(MCU_FLAG) -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -Og -g3 -Wall -fmessage-length=0 -ffunction-sections
TEST_LIBS := -lgtest -lgmock -lgmock_main

# Define target recipes for all objects in object list, create object in BUILD_DIR
TARGET_BUILD_DIR := $(BUILD_DIR)/$(TARGET)
$(TARGET_BUILD_DIR)/%.o: %.c
	@echo 'MCU GCC Compiler: Building file: $<'
	@mkdir -p $(TARGET_BUILD_DIR)/$(patsubst /%.c,,$*)
	arm-none-eabi-gcc $(CFLAGS) $(INC)  -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
	
$(TARGET_BUILD_DIR)/%.o: %.cpp
	@echo 'MCU G++ Compiler: Building file: $<'
	@mkdir -p $(TARGET_BUILD_DIR)/$(patsubst /%.cpp,,$*)
	arm-none-eabi-g++ $(CFLAGS) $(INC) -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '	
	
$(TARGET_BUILD_DIR)/%.o: $(COMMON_DIR)/%.c
	@echo 'MCU GCC Compiler: Building file: $<'
	@mkdir -p $(TARGET_BUILD_DIR)/$(patsubst /%.c,,$*)
	arm-none-eabi-gcc $(CFLAGS) $(INC)  -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
	
$(TARGET_BUILD_DIR)/%.o: $(COMMON_DIR)/%.cpp
	@echo 'MCU G++ Compiler: Building file: $<'
	@mkdir -p $(TARGET_BUILD_DIR)/$(patsubst /%.cpp,,$*)
	arm-none-eabi-g++ $(CFLAGS) $(INC) -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '	

# Note: The .s recipe only includes Common/include	
$(TARGET_BUILD_DIR)/%.o: %.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@mkdir -p $(TARGET_BUILD_DIR)/$(patsubst /%.s,,$*)
	arm-none-eabi-as $(MCPU) -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 -I$(COMMON_DIR)/include -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

$(TARGET_BUILD_DIR)/Test/%.o: %.cpp
	@echo 'GCC C++ Compiler: Building file: $<'
	@mkdir -p $(@D)
	g++ $(MCU_DSTM) $(TEST_INC) $(INC) -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
	
$(TARGET_BUILD_DIR)/Test/%.o: $(COMMON_DIR)/%.cpp
	@echo 'GCC C++ Compiler: Building file: $<'
	$(info $(@D))
	$(info directory $(TARGET_BUILD_DIR)/Test/$(patsubst /%.cpp,,$*))
	@mkdir -p $(@D)
	g++ $(MCU_DSTM) $(TEST_INC) $(INC) -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '	
	

# Define libraries
ifeq ($(TARGET),$(F4))
LIBS := -larm_cortexM4lf_math
else ifeq ($(TARGET),$(F7))
LIBS := -l:libarm_cortexM7lfdp_math.a
else
$(error LIBS not configured for target $(TARGET))
endif


# All Target
all: $(BUILD_DIR)/$(TARGET)/Robot_$(TARGET).elf $(BUILD_DIR)/$(TARGET)/Test/RobotTest.exe 

# Tool invocations
$(BUILD_DIR)/F4/Robot_F4.elf: $(C_OBJECTS) $(CPP_OBJECTS) $(S_OBJECTS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU G++ Linker'
	$(info $(C_SRCS))
	$(info $(C_OBJECTS))
	arm-none-eabi-g++ -lgcov --coverage $(MCU_FLAG) -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -L"Lib/F4"  -specs=nosys.specs -specs=nano.specs -T"STM32F446RETx_FLASH.ld" -Wl,-Map=output.map -Wl,--gc-sections -fno-exceptions -fno-rtti -o "$(BUILD_DIR)/F4/Robot_F4.elf" $(C_OBJECTS) $(CPP_OBJECTS) $(S_OBJECTS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build
	
$(BUILD_DIR)/F7/Robot_F7.elf: $(C_OBJECTS) $(CPP_OBJECTS) $(S_OBJECTS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU G++ Linker'
	arm-none-eabi-g++ -lgcov --coverage $(MCU_FLAG) -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 -L"Lib/F7"  -specs=nosys.specs -specs=nano.specs -T"STM32F767ZITx_FLASH.ld" -Wl,-Map=output.map -Wl,--gc-sections -fno-exceptions -fno-rtti -o "$(BUILD_DIR)/F7/Robot_F7.elf" $(C_OBJECTS) $(CPP_OBJECTS) $(S_OBJECTS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build
	
$(BUILD_DIR)/$(TARGET)/Test/RobotTest.exe: $(CPP_TEST_OBJECTS)
	$(info $(CPP_TEST_SRCS))
	$(info $(CPP_TEST_INC_SRCS))
	$(info $(CPP_TEST_OBJECTS))
	@echo 'Building target: $@'
	@echo 'Invoking: MinGW C++ Linker'
	g++ -L"C:\Users\gokul\Documents\GitHub\soccer-embedded\RobotTest\googletest\build\googlemock" -L"C:\Users\gokul\Documents\GitHub\soccer-embedded\RobotTest\googletest\build\googlemock\gtest" -o "$(BUILD_DIR)/$(TARGET)/Test/RobotTest.exe" $(CPP_TEST_OBJECTS) $(TEST_LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	./$(BUILD_DIR)/$(TARGET)/Test/RobotTest.exe 

# Other Targets
clean:
	-$(RM) $(BUILD_DIR)/*
	-@echo ' '

post-build:
	-@echo 'Generating hex and Printing size information:'
	arm-none-eabi-objcopy -O ihex "$(BUILD_DIR)/$(TARGET)/Robot_$(TARGET).elf" "$(BUILD_DIR)/$(TARGET)/Robot_$(TARGET).hex"
	arm-none-eabi-size "$(BUILD_DIR)/$(TARGET)/Robot_$(TARGET).elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include $(C_OBJECTS:.o=.d)
-include $(CPP_OBJECTS:.o=.d)
