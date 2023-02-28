################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f303xx_GPIO_driver.c \
../Drivers/Src/stm32f303xx_SPI.c 

OBJS += \
./Drivers/Src/stm32f303xx_GPIO_driver.o \
./Drivers/Src/stm32f303xx_SPI.o 

C_DEPS += \
./Drivers/Src/stm32f303xx_GPIO_driver.d \
./Drivers/Src/stm32f303xx_SPI.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F303VCTx -DSTM32 -DSTM32F3 -DSTM32F3DISCOVERY -c -I../Inc -I"H:/STM32_driver/GPIO_driver/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/stm32f303xx_GPIO_driver.d ./Drivers/Src/stm32f303xx_GPIO_driver.o ./Drivers/Src/stm32f303xx_GPIO_driver.su ./Drivers/Src/stm32f303xx_SPI.d ./Drivers/Src/stm32f303xx_SPI.o ./Drivers/Src/stm32f303xx_SPI.su

.PHONY: clean-Drivers-2f-Src

