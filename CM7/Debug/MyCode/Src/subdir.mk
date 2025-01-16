################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MyCode/Src/lps22hh.c 

OBJS += \
./MyCode/Src/lps22hh.o 

C_DEPS += \
./MyCode/Src/lps22hh.d 


# Each subdirectory must supply rules for building sources it contributes
MyCode/Src/%.o MyCode/Src/%.su MyCode/Src/%.cyclo: ../MyCode/Src/%.c MyCode/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../MyCode/Inc -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MyCode-2f-Src

clean-MyCode-2f-Src:
	-$(RM) ./MyCode/Src/lps22hh.cyclo ./MyCode/Src/lps22hh.d ./MyCode/Src/lps22hh.o ./MyCode/Src/lps22hh.su

.PHONY: clean-MyCode-2f-Src

