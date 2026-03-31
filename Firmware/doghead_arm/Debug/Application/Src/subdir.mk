################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Application/Src/kinematics.cpp \
../Application/Src/robot_arm_core.cpp 

OBJS += \
./Application/Src/kinematics.o \
./Application/Src/robot_arm_core.o 

CPP_DEPS += \
./Application/Src/kinematics.d \
./Application/Src/robot_arm_core.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Src/%.o Application/Src/%.su Application/Src/%.cyclo: ../Application/Src/%.cpp Application/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/Inc -I../Application/Inc -I"/home/jail487/doghead/dogArm-project/Firmware/doghead_arm/Application/Inc" -I../Middleware/MicroROS/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-Src

clean-Application-2f-Src:
	-$(RM) ./Application/Src/kinematics.cyclo ./Application/Src/kinematics.d ./Application/Src/kinematics.o ./Application/Src/kinematics.su ./Application/Src/robot_arm_core.cyclo ./Application/Src/robot_arm_core.d ./Application/Src/robot_arm_core.o ./Application/Src/robot_arm_core.su

.PHONY: clean-Application-2f-Src

