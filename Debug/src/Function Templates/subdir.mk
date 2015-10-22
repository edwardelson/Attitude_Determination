################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/Function\ Templates/ee_3Dcoordinates.c \
../src/Function\ Templates/ee_functions.c \
../src/Function\ Templates/ee_kalman.c 

OBJS += \
./src/Function\ Templates/ee_3Dcoordinates.o \
./src/Function\ Templates/ee_functions.o \
./src/Function\ Templates/ee_kalman.o 

C_DEPS += \
./src/Function\ Templates/ee_3Dcoordinates.d \
./src/Function\ Templates/ee_functions.d \
./src/Function\ Templates/ee_kalman.d 


# Each subdirectory must supply rules for building sources it contributes
src/Function\ Templates/ee_3Dcoordinates.o: ../src/Function\ Templates/ee_3Dcoordinates.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DSTM32F411xE -DTRACE -I"../include" -I"../system/include/FreeRTOS" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4xx" -std=gnu11 -MMD -MP -MF"src/Function Templates/ee_3Dcoordinates.d" -MT"src/Function\ Templates/ee_3Dcoordinates.d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/Function\ Templates/ee_functions.o: ../src/Function\ Templates/ee_functions.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DSTM32F411xE -DTRACE -I"../include" -I"../system/include/FreeRTOS" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4xx" -std=gnu11 -MMD -MP -MF"src/Function Templates/ee_functions.d" -MT"src/Function\ Templates/ee_functions.d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/Function\ Templates/ee_kalman.o: ../src/Function\ Templates/ee_kalman.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DSTM32F411xE -DTRACE -I"../include" -I"../system/include/FreeRTOS" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4xx" -std=gnu11 -MMD -MP -MF"src/Function Templates/ee_kalman.d" -MT"src/Function\ Templates/ee_kalman.d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


