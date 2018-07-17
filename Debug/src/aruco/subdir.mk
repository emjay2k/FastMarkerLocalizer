################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/aruco/arucofidmarkers.cpp \
../src/aruco/board.cpp \
../src/aruco/cameraparameters.cpp \
../src/aruco/highlyreliablemarkers.cpp \
../src/aruco/marker.cpp \
../src/aruco/subpixelcorner.cpp 

OBJS += \
./src/aruco/arucofidmarkers.o \
./src/aruco/board.o \
./src/aruco/cameraparameters.o \
./src/aruco/highlyreliablemarkers.o \
./src/aruco/marker.o \
./src/aruco/subpixelcorner.o 

CPP_DEPS += \
./src/aruco/arucofidmarkers.d \
./src/aruco/board.d \
./src/aruco/cameraparameters.d \
./src/aruco/highlyreliablemarkers.d \
./src/aruco/marker.d \
./src/aruco/subpixelcorner.d 


# Each subdirectory must supply rules for building sources it contributes
src/aruco/%.o: ../src/aruco/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -I"/home/matthias/workspace/FastMarkerLocalizer_rpi/src" -O0 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


