################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/net/posetopayloadtranslator.cpp \
../src/net/udppacketsender.cpp 

OBJS += \
./src/net/posetopayloadtranslator.o \
./src/net/udppacketsender.o 

CPP_DEPS += \
./src/net/posetopayloadtranslator.d \
./src/net/udppacketsender.d 


# Each subdirectory must supply rules for building sources it contributes
src/net/%.o: ../src/net/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -I"/home/matthias/workspace/FastMarkerLocalizer_rpi/src" -O0 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


