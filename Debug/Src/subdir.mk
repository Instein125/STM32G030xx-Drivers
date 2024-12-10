################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/gpio.c \
../Src/led_toggle.c \
../Src/stm32g030xx.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/gpio.o \
./Src/led_toggle.o \
./Src/stm32g030xx.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/gpio.d \
./Src/led_toggle.d \
./Src/stm32g030xx.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g -DDEBUG -DSTM32G030F6Px -DSTM32 -DSTM32G0 -c -I"C:/Users/shres/OneDrive/Documents/stm32 projects/MCU1/stm32g030xx_drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/gpio.cyclo ./Src/gpio.d ./Src/gpio.o ./Src/gpio.su ./Src/led_toggle.cyclo ./Src/led_toggle.d ./Src/led_toggle.o ./Src/led_toggle.su ./Src/stm32g030xx.cyclo ./Src/stm32g030xx.d ./Src/stm32g030xx.o ./Src/stm32g030xx.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

