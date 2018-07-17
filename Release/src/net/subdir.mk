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
	g++ -std=c++1y -DRPI -I"/home/pi/fml/FastMarkerLocalizer/src" -O3 -fomit-frame-pointer -pipe -ffast-math -march=armv6j -mfpu=vfp -mfloat-abi=hard -ftree-vectorize -pedantic -Wall -Wextra -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


