################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/imageprocessor.cpp \
../src/main.cpp \
../src/timestampedcameracapture.cpp \
../src/timestampedvideocapture.cpp 

OBJS += \
./src/imageprocessor.o \
./src/main.o \
./src/timestampedcameracapture.o \
./src/timestampedvideocapture.o 

CPP_DEPS += \
./src/imageprocessor.d \
./src/main.d \
./src/timestampedcameracapture.d \
./src/timestampedvideocapture.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -I"/home/matthias/workspace/FastMarkerLocalizer_rpi/src" -O0 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


