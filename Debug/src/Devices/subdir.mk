################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/Devices/ee_hmc5883l.c \
../src/Devices/ee_mpu6050.c \
../src/Devices/tmp006.c 

OBJS += \
./src/Devices/ee_hmc5883l.o \
./src/Devices/ee_mpu6050.o \
./src/Devices/tmp006.o 

C_DEPS += \
./src/Devices/ee_hmc5883l.d \
./src/Devices/ee_mpu6050.d \
./src/Devices/tmp006.d 


# Each subdirectory must supply rules for building sources it contributes
src/Devices/%.o: ../src/Devices/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DSTM32F411xE -DTRACE -I"../include" -I"../system/include/FreeRTOS" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4xx" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


