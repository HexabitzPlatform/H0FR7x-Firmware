################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/ASUS/Documents/H0FR7x-Firmware/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/FreeRTOS_CLI.c \
C:/Users/ASUS/Documents/H0FR7x-Firmware/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/croutine.c \
C:/Users/ASUS/Documents/H0FR7x-Firmware/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/event_groups.c \
C:/Users/ASUS/Documents/H0FR7x-Firmware/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/list.c \
C:/Users/ASUS/Documents/H0FR7x-Firmware/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/queue.c \
C:/Users/ASUS/Documents/H0FR7x-Firmware/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/tasks.c \
C:/Users/ASUS/Documents/H0FR7x-Firmware/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/timers.c 

OBJS += \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/FreeRTOS_CLI.o \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/croutine.o \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/event_groups.o \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/list.o \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/queue.o \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/tasks.o \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/timers.o 

C_DEPS += \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/FreeRTOS_CLI.d \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/croutine.d \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/event_groups.d \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/list.d \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/queue.d \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/tasks.d \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/FreeRTOS_CLI.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/FreeRTOS_CLI.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/FreeRTOS_CLI.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/croutine.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/croutine.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/croutine.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/event_groups.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/event_groups.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/event_groups.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/list.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/list.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/list.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/queue.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/queue.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/queue.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/tasks.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/tasks.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/tasks.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/timers.o: C:/Users/ASUS/Documents/H0FR7x-Firmware/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/timers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xC '-D_module=1' -DH0FR7 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../BOS -I../../User -I../../H0FR7 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -O2 -ffunction-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/timers.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"

