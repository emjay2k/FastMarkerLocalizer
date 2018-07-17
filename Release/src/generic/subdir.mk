################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/generic/threadrunnable.cpp 

OBJS += \
./src/generic/threadrunnable.o 

CPP_DEPS += \
./src/generic/threadrunnable.d 


# Each subdirectory must supply rules for building sources it contributes
src/generic/%.o: ../src/generic/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -I"/home/pi/fml/FastMarkerLocalizer/src" -O3 -ffast-math -march=armv6zk -mfpu=vfp -mcpu=arm1176jzf-s -mtune=arm1176jzf-s -mfloat-abi=hard -pedantic -Wall -Wextra -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


