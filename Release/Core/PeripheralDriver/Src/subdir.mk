################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/PeripheralDriver/Src/oled.c 

OBJS += \
./Core/PeripheralDriver/Src/oled.o 

C_DEPS += \
./Core/PeripheralDriver/Src/oled.d 


# Each subdirectory must supply rules for building sources it contributes
Core/PeripheralDriver/Src/%.o: ../Core/PeripheralDriver/Src/%.c Core/PeripheralDriver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I"/Users/yulinzou/STM32CubeIDE/workspace_1.8.0/MDP_HelloWorld/Core/PeripheralDriver/Inc" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-PeripheralDriver-2f-Src

clean-Core-2f-PeripheralDriver-2f-Src:
	-$(RM) ./Core/PeripheralDriver/Src/oled.d ./Core/PeripheralDriver/Src/oled.o

.PHONY: clean-Core-2f-PeripheralDriver-2f-Src

