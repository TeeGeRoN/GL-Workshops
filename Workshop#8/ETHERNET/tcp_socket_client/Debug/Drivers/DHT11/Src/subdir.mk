################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/DHT11/Src/DHT.c \
../Drivers/DHT11/Src/dht11.c \
../Drivers/DHT11/Src/dht11_stm32.c 

OBJS += \
./Drivers/DHT11/Src/DHT.o \
./Drivers/DHT11/Src/dht11.o \
./Drivers/DHT11/Src/dht11_stm32.o 

C_DEPS += \
./Drivers/DHT11/Src/DHT.d \
./Drivers/DHT11/Src/dht11.d \
./Drivers/DHT11/Src/dht11_stm32.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/DHT11/Src/%.o: ../Drivers/DHT11/Src/%.c Drivers/DHT11/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HTTP_SERVER_SOLUTION -DUSE_HTTP_SERVER -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Drivers/KSZ8081RNB/Inc -I../Drivers/DHT11/Inc -I../Drivers/WH1602B/Inc -I../Drivers/BSP/Inc -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

