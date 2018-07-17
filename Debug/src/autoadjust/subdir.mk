################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/autoadjust/autoexposurealgorithm.cpp \
../src/autoadjust/autosensoralgorithm.cpp \
../src/autoadjust/exposurelimiter.cpp \
../src/autoadjust/matrixautoexposurealgorithm.cpp \
../src/autoadjust/meanautoexposurealgorithm.cpp 

OBJS += \
./src/autoadjust/autoexposurealgorithm.o \
./src/autoadjust/autosensoralgorithm.o \
./src/autoadjust/exposurelimiter.o \
./src/autoadjust/matrixautoexposurealgorithm.o \
./src/autoadjust/meanautoexposurealgorithm.o 

CPP_DEPS += \
./src/autoadjust/autoexposurealgorithm.d \
./src/autoadjust/autosensoralgorithm.d \
./src/autoadjust/exposurelimiter.d \
./src/autoadjust/matrixautoexposurealgorithm.d \
./src/autoadjust/meanautoexposurealgorithm.d 


# Each subdirectory must supply rules for building sources it contributes
src/autoadjust/%.o: ../src/autoadjust/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -I"/home/matthias/workspace/FastMarkerLocalizer_rpi/src" -O0 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


