################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.s 

C_SRCS += \
../Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c 

OBJS += \
./Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.o \
./Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/system_ARMCM7.o 

S_DEPS += \
./Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.d 

C_DEPS += \
./Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/system_ARMCM7.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/%.o: ../Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/%.s Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/%.o Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/%.su: ../Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/%.c Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../FATFS/Target -I../FATFS/App -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/DSP/Include/ -I../Drivers/CMSIS/NN/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_dotproduct_example-2f-RTE-2f-Device-2f-ARMCM7_SP

clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_dotproduct_example-2f-RTE-2f-Device-2f-ARMCM7_SP:
	-$(RM) ./Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.d ./Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.o ./Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/system_ARMCM7.d ./Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/system_ARMCM7.o ./Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/system_ARMCM7.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_dotproduct_example-2f-RTE-2f-Device-2f-ARMCM7_SP

