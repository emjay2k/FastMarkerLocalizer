################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/location/coordinatetransformer.cpp \
../src/location/evaluationposestorage.cpp \
../src/location/kalmanlocationpredictor.cpp \
../src/location/locationpredictor.cpp \
../src/location/pnpestimator.cpp \
../src/location/pose2d.cpp \
../src/location/poseestimator.cpp \
../src/location/posestorage.cpp \
../src/location/pppestimator.cpp \
../src/location/simpleposestorage.cpp \
../src/location/vectorlocationpredictor.cpp 

OBJS += \
./src/location/coordinatetransformer.o \
./src/location/evaluationposestorage.o \
./src/location/kalmanlocationpredictor.o \
./src/location/locationpredictor.o \
./src/location/pnpestimator.o \
./src/location/pose2d.o \
./src/location/poseestimator.o \
./src/location/posestorage.o \
./src/location/pppestimator.o \
./src/location/simpleposestorage.o \
./src/location/vectorlocationpredictor.o 

CPP_DEPS += \
./src/location/coordinatetransformer.d \
./src/location/evaluationposestorage.d \
./src/location/kalmanlocationpredictor.d \
./src/location/locationpredictor.d \
./src/location/pnpestimator.d \
./src/location/pose2d.d \
./src/location/poseestimator.d \
./src/location/posestorage.d \
./src/location/pppestimator.d \
./src/location/simpleposestorage.d \
./src/location/vectorlocationpredictor.d 


# Each subdirectory must supply rules for building sources it contributes
src/location/%.o: ../src/location/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++1y -DRPI -I"/home/pi/fml/FastMarkerLocalizer/src" -O3 -fomit-frame-pointer -pipe -ffast-math -march=armv6j -mfpu=vfp -mfloat-abi=hard -ftree-vectorize -pedantic -Wall -Wextra -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


