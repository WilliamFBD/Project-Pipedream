################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/acc_hal_integration_stm32cube_sparkfun_a111.c \
../Src/acc_integration_cortex.c \
../Src/acc_integration_log.c \
../Src/acc_integration_stm32.c \
../Src/acc_ms_system_cortex.c \
../Src/acc_ms_system_stm32cube.c \
../Src/acc_wrap_printf.c \
../Src/example_assembly_test.c \
../Src/example_service_envelope.c \
../Src/main.c \
../Src/printf.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32l4xx.c 

OBJS += \
./Src/acc_hal_integration_stm32cube_sparkfun_a111.o \
./Src/acc_integration_cortex.o \
./Src/acc_integration_log.o \
./Src/acc_integration_stm32.o \
./Src/acc_ms_system_cortex.o \
./Src/acc_ms_system_stm32cube.o \
./Src/acc_wrap_printf.o \
./Src/example_assembly_test.o \
./Src/example_service_envelope.o \
./Src/main.o \
./Src/printf.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32l4xx.o 

C_DEPS += \
./Src/acc_hal_integration_stm32cube_sparkfun_a111.d \
./Src/acc_integration_cortex.d \
./Src/acc_integration_log.d \
./Src/acc_integration_stm32.d \
./Src/acc_ms_system_cortex.d \
./Src/acc_ms_system_stm32cube.d \
./Src/acc_wrap_printf.d \
./Src/example_assembly_test.d \
./Src/example_service_envelope.d \
./Src/main.d \
./Src/printf.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu99 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Nicolai/Documents/NMBU/TEL320/Project/stm32l476_module_software/cortex_m4/rss/include" -I"C:/Users/Nicolai/Documents/NMBU/TEL320/Project/stm32l476_module_software/cortex_m4/integration" -I"C:/Users/Nicolai/Documents/NMBU/TEL320/Project/stm32l476_module_software/cortex_m4/examples" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/acc_hal_integration_stm32cube_sparkfun_a111.cyclo ./Src/acc_hal_integration_stm32cube_sparkfun_a111.d ./Src/acc_hal_integration_stm32cube_sparkfun_a111.o ./Src/acc_hal_integration_stm32cube_sparkfun_a111.su ./Src/acc_integration_cortex.cyclo ./Src/acc_integration_cortex.d ./Src/acc_integration_cortex.o ./Src/acc_integration_cortex.su ./Src/acc_integration_log.cyclo ./Src/acc_integration_log.d ./Src/acc_integration_log.o ./Src/acc_integration_log.su ./Src/acc_integration_stm32.cyclo ./Src/acc_integration_stm32.d ./Src/acc_integration_stm32.o ./Src/acc_integration_stm32.su ./Src/acc_ms_system_cortex.cyclo ./Src/acc_ms_system_cortex.d ./Src/acc_ms_system_cortex.o ./Src/acc_ms_system_cortex.su ./Src/acc_ms_system_stm32cube.cyclo ./Src/acc_ms_system_stm32cube.d ./Src/acc_ms_system_stm32cube.o ./Src/acc_ms_system_stm32cube.su ./Src/acc_wrap_printf.cyclo ./Src/acc_wrap_printf.d ./Src/acc_wrap_printf.o ./Src/acc_wrap_printf.su ./Src/example_assembly_test.cyclo ./Src/example_assembly_test.d ./Src/example_assembly_test.o ./Src/example_assembly_test.su ./Src/example_service_envelope.cyclo ./Src/example_service_envelope.d ./Src/example_service_envelope.o ./Src/example_service_envelope.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/printf.cyclo ./Src/printf.d ./Src/printf.o ./Src/printf.su ./Src/stm32l4xx_hal_msp.cyclo ./Src/stm32l4xx_hal_msp.d ./Src/stm32l4xx_hal_msp.o ./Src/stm32l4xx_hal_msp.su ./Src/stm32l4xx_it.cyclo ./Src/stm32l4xx_it.d ./Src/stm32l4xx_it.o ./Src/stm32l4xx_it.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32l4xx.cyclo ./Src/system_stm32l4xx.d ./Src/system_stm32l4xx.o ./Src/system_stm32l4xx.su

.PHONY: clean-Src

