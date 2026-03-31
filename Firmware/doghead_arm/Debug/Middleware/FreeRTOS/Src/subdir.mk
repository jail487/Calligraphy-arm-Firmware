################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middleware/FreeRTOS/Src/freertos.c 

C_DEPS += \
./Middleware/FreeRTOS/Src/freertos.d 

OBJS += \
./Middleware/FreeRTOS/Src/freertos.o 


# Each subdirectory must supply rules for building sources it contributes
Middleware/FreeRTOS/Src/%.o Middleware/FreeRTOS/Src/%.su Middleware/FreeRTOS/Src/%.cyclo: ../Middleware/FreeRTOS/Src/%.c Middleware/FreeRTOS/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/jail487/doghead/dogArm-project/Firmware/doghead_arm/micro_ros_stm32cubemx_utils/extra_sources/microros_transports" -I"/home/jail487/doghead/dogArm-project/Firmware/doghead_arm/micro_ros_stm32cubemx_utils/extra_sources" -I../Drivers/Inc -I../Application/Inc -I"/home/jail487/doghead/dogArm-project/Firmware/doghead_arm/Application/Inc" -I../Middleware/MicroROS/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middleware-2f-FreeRTOS-2f-Src

clean-Middleware-2f-FreeRTOS-2f-Src:
	-$(RM) ./Middleware/FreeRTOS/Src/freertos.cyclo ./Middleware/FreeRTOS/Src/freertos.d ./Middleware/FreeRTOS/Src/freertos.o ./Middleware/FreeRTOS/Src/freertos.su

.PHONY: clean-Middleware-2f-FreeRTOS-2f-Src

