################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/startup_stm32f091xc.s 

C_SRCS += \
C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7.c \
C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_adc.c \
C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_dma.c \
C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_eeprom.c \
C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_gpio.c \
C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_inputs.c \
C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_it.c \
C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_rtc.c \
C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_timers.c \
C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_uart.c 

OBJS += \
./H0FR7/H0FR7.o \
./H0FR7/H0FR7_adc.o \
./H0FR7/H0FR7_dma.o \
./H0FR7/H0FR7_eeprom.o \
./H0FR7/H0FR7_gpio.o \
./H0FR7/H0FR7_inputs.o \
./H0FR7/H0FR7_it.o \
./H0FR7/H0FR7_rtc.o \
./H0FR7/H0FR7_timers.o \
./H0FR7/H0FR7_uart.o \
./H0FR7/startup_stm32f091xc.o 

S_DEPS += \
./H0FR7/startup_stm32f091xc.d 

C_DEPS += \
./H0FR7/H0FR7.d \
./H0FR7/H0FR7_adc.d \
./H0FR7/H0FR7_dma.d \
./H0FR7/H0FR7_eeprom.d \
./H0FR7/H0FR7_gpio.d \
./H0FR7/H0FR7_inputs.d \
./H0FR7/H0FR7_it.d \
./H0FR7/H0FR7_rtc.d \
./H0FR7/H0FR7_timers.d \
./H0FR7/H0FR7_uart.d 


# Each subdirectory must supply rules for building sources it contributes
H0FR7/H0FR7.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"H0FR7/H0FR7.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0FR7/H0FR7_adc.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_adc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"H0FR7/H0FR7_adc.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0FR7/H0FR7_dma.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_dma.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"H0FR7/H0FR7_dma.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0FR7/H0FR7_eeprom.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_eeprom.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"H0FR7/H0FR7_eeprom.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0FR7/H0FR7_gpio.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"H0FR7/H0FR7_gpio.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0FR7/H0FR7_inputs.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_inputs.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"H0FR7/H0FR7_inputs.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0FR7/H0FR7_it.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"H0FR7/H0FR7_it.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0FR7/H0FR7_rtc.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_rtc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"H0FR7/H0FR7_rtc.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0FR7/H0FR7_timers.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_timers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"H0FR7/H0FR7_timers.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0FR7/H0FR7_uart.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/H0FR7_uart.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"H0FR7/H0FR7_uart.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0FR7/startup_stm32f091xc.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/H0FR7/startup_stm32f091xc.s
	arm-none-eabi-gcc -mcpu=cortex-m0 -g3 -c -x assembler-with-cpp -MMD -MP -MF"H0FR7/startup_stm32f091xc.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@" "$<"

