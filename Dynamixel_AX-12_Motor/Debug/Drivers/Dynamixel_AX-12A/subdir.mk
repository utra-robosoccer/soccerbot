################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Dynamixel_AX-12A/Dynamixel_AX-12A.c 

OBJS += \
./Drivers/Dynamixel_AX-12A/Dynamixel_AX-12A.o 

C_DEPS += \
./Drivers/Dynamixel_AX-12A/Dynamixel_AX-12A.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Dynamixel_AX-12A/%.o: ../Drivers/Dynamixel_AX-12A/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F446xx -I"/other/STM32/soccer-embedded/Dynamixel_AX-12_Motor/Inc" -I"/other/STM32/soccer-embedded/Dynamixel_AX-12_Motor/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/other/STM32/soccer-embedded/Dynamixel_AX-12_Motor/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/other/STM32/soccer-embedded/Dynamixel_AX-12_Motor/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/other/STM32/soccer-embedded/Dynamixel_AX-12_Motor/Drivers/CMSIS/Include" -I"/other/STM32/soccer-embedded/Dynamixel_AX-12_Motor/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


