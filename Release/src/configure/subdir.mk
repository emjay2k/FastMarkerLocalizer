################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/configure/calibrationparameters.cpp \
../src/configure/cameraparameters.cpp \
../src/configure/config.cpp \
../src/configure/markerparameters.cpp \
../src/configure/trackerparameters.cpp 

OBJS += \
./src/configure/calibrationparameters.o \
./src/configure/cameraparameters.o \
./src/configure/config.o \
./src/configure/markerparameters.o \
./src/configure/trackerparameters.o 

CPP_DEPS += \
./src/configure/calibrationparameters.d \
./src/configure/cameraparameters.d \
./src/configure/config.d \
./src/configure/markerparameters.d \
./src/configure/trackerparameters.d 


# Each subdirectory must supply rules for building sources it contributes
src/configure/%.o: ../src/configure/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++1y -DRPI -I"/home/pi/fml/FastMarkerLocalizer/src" -O3 -fomit-frame-pointer -pipe -ffast-math -march=armv6j -mfpu=vfp -mfloat-abi=hard -ftree-vectorize -pedantic -Wall -Wextra -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


