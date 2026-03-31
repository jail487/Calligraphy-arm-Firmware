################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middleware/MicroROS/Src/custom_memory_manager.c \
../Middleware/MicroROS/Src/it_transport.c \
../Middleware/MicroROS/Src/microros_allocators.c \
../Middleware/MicroROS/Src/microros_app.c \
../Middleware/MicroROS/Src/microros_stubs.c \
../Middleware/MicroROS/Src/microros_time.c 

C_DEPS += \
./Middleware/MicroROS/Src/custom_memory_manager.d \
./Middleware/MicroROS/Src/it_transport.d \
./Middleware/MicroROS/Src/microros_allocators.d \
./Middleware/MicroROS/Src/microros_app.d \
./Middleware/MicroROS/Src/microros_stubs.d \
./Middleware/MicroROS/Src/microros_time.d 

OBJS += \
./Middleware/MicroROS/Src/custom_memory_manager.o \
./Middleware/MicroROS/Src/it_transport.o \
./Middleware/MicroROS/Src/microros_allocators.o \
./Middleware/MicroROS/Src/microros_app.o \
./Middleware/MicroROS/Src/microros_stubs.o \
./Middleware/MicroROS/Src/microros_time.o 


# Each subdirectory must supply rules for building sources it contributes
Middleware/MicroROS/Src/%.o Middleware/MicroROS/Src/%.su Middleware/MicroROS/Src/%.cyclo: ../Middleware/MicroROS/Src/%.c Middleware/MicroROS/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/jail487/doghead/dogArm-project/Firmware/doghead_arm/micro_ros_stm32cubemx_utils/extra_sources/microros_transports" -I"/home/jail487/doghead/dogArm-project/Firmware/doghead_arm/micro_ros_stm32cubemx_utils/extra_sources" -I../Drivers/Inc -I../Application/Inc -I"/home/jail487/doghead/dogArm-project/Firmware/doghead_arm/Application/Inc" -I../Middleware/MicroROS/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middleware-2f-MicroROS-2f-Src

clean-Middleware-2f-MicroROS-2f-Src:
	-$(RM) ./Middleware/MicroROS/Src/custom_memory_manager.cyclo ./Middleware/MicroROS/Src/custom_memory_manager.d ./Middleware/MicroROS/Src/custom_memory_manager.o ./Middleware/MicroROS/Src/custom_memory_manager.su ./Middleware/MicroROS/Src/it_transport.cyclo ./Middleware/MicroROS/Src/it_transport.d ./Middleware/MicroROS/Src/it_transport.o ./Middleware/MicroROS/Src/it_transport.su ./Middleware/MicroROS/Src/microros_allocators.cyclo ./Middleware/MicroROS/Src/microros_allocators.d ./Middleware/MicroROS/Src/microros_allocators.o ./Middleware/MicroROS/Src/microros_allocators.su ./Middleware/MicroROS/Src/microros_app.cyclo ./Middleware/MicroROS/Src/microros_app.d ./Middleware/MicroROS/Src/microros_app.o ./Middleware/MicroROS/Src/microros_app.su ./Middleware/MicroROS/Src/microros_stubs.cyclo ./Middleware/MicroROS/Src/microros_stubs.d ./Middleware/MicroROS/Src/microros_stubs.o ./Middleware/MicroROS/Src/microros_stubs.su ./Middleware/MicroROS/Src/microros_time.cyclo ./Middleware/MicroROS/Src/microros_time.d ./Middleware/MicroROS/Src/microros_time.o ./Middleware/MicroROS/Src/microros_time.su

.PHONY: clean-Middleware-2f-MicroROS-2f-Src

