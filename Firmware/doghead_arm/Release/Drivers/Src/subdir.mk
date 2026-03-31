################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/nidec_motor_driver.c 

C_DEPS += \
./Drivers/Src/nidec_motor_driver.d 

OBJS += \
./Drivers/Src/nidec_motor_driver.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/Inc -I../Application/Inc -I"/home/jail487/doghead/dogArm-project/Firmware/doghead_arm/Application/Inc" -I"/home/jail487/doghead/dogArm-project/Firmware/doghead_arm/Middleware/MicroROS/Inc" -I../Middleware/MicroROS/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/nidec_motor_driver.cyclo ./Drivers/Src/nidec_motor_driver.d ./Drivers/Src/nidec_motor_driver.o ./Drivers/Src/nidec_motor_driver.su

.PHONY: clean-Drivers-2f-Src

