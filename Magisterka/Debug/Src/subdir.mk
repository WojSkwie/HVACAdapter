################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/comm.c \
../Src/gpio.c \
../Src/main.c \
../Src/pwm.c \
../Src/stm32l0xx_hal_msp.c \
../Src/stm32l0xx_it.c \
../Src/system_stm32l0xx.c 

OBJS += \
./Src/adc.o \
./Src/comm.o \
./Src/gpio.o \
./Src/main.o \
./Src/pwm.o \
./Src/stm32l0xx_hal_msp.o \
./Src/stm32l0xx_it.o \
./Src/system_stm32l0xx.o 

C_DEPS += \
./Src/adc.d \
./Src/comm.d \
./Src/gpio.d \
./Src/main.d \
./Src/pwm.d \
./Src/stm32l0xx_hal_msp.d \
./Src/stm32l0xx_it.d \
./Src/system_stm32l0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32L053xx -I"D:/materialy_na_studia/MAGISTERKA/cube/Magisterka/Inc" -I"D:/materialy_na_studia/MAGISTERKA/cube/Magisterka/Drivers/STM32L0xx_HAL_Driver/Inc" -I"D:/materialy_na_studia/MAGISTERKA/cube/Magisterka/Drivers/STM32L0xx_HAL_Driver/Inc/Legacy" -I"D:/materialy_na_studia/MAGISTERKA/cube/Magisterka/Drivers/CMSIS/Device/ST/STM32L0xx/Include" -I"D:/materialy_na_studia/MAGISTERKA/cube/Magisterka/Drivers/CMSIS/Include" -I"D:/materialy_na_studia/MAGISTERKA/cube/Magisterka/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


