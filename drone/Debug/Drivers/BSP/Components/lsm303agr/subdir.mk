################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lsm303agr/lsm303agr.c \
../Drivers/BSP/Components/lsm303agr/lsm303agr_reg.c 

OBJS += \
./Drivers/BSP/Components/lsm303agr/lsm303agr.o \
./Drivers/BSP/Components/lsm303agr/lsm303agr_reg.o 

C_DEPS += \
./Drivers/BSP/Components/lsm303agr/lsm303agr.d \
./Drivers/BSP/Components/lsm303agr/lsm303agr_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lsm303agr/%.o Drivers/BSP/Components/lsm303agr/%.su Drivers/BSP/Components/lsm303agr/%.cyclo: ../Drivers/BSP/Components/lsm303agr/%.c Drivers/BSP/Components/lsm303agr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/lsm6dsl -I../Drivers/BSP/Components/lsm303agr -I../Drivers/BSP/Components/hts221 -I../Drivers/BSP/Components/lps22hb -I../Drivers/BSP/IKS01A2 -I../Drivers/BSP/Components/Common -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lsm303agr

clean-Drivers-2f-BSP-2f-Components-2f-lsm303agr:
	-$(RM) ./Drivers/BSP/Components/lsm303agr/lsm303agr.cyclo ./Drivers/BSP/Components/lsm303agr/lsm303agr.d ./Drivers/BSP/Components/lsm303agr/lsm303agr.o ./Drivers/BSP/Components/lsm303agr/lsm303agr.su ./Drivers/BSP/Components/lsm303agr/lsm303agr_reg.cyclo ./Drivers/BSP/Components/lsm303agr/lsm303agr_reg.d ./Drivers/BSP/Components/lsm303agr/lsm303agr_reg.o ./Drivers/BSP/Components/lsm303agr/lsm303agr_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lsm303agr

