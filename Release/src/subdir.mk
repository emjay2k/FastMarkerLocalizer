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
	g++ -std=c++1y -DRPI -I"/home/pi/fml/FastMarkerLocalizer/src" -O3 -fomit-frame-pointer -pipe -ffast-math -march=armv6j -mfpu=vfp -mfloat-abi=hard -ftree-vectorize -pedantic -Wall -Wextra -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


