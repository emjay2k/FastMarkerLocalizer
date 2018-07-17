################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/marker/arucomarkerdetector.cpp \
../src/marker/arucomarkeridentifier.cpp \
../src/marker/fullmarkerextractor.cpp \
../src/marker/mappingmarkerpredictor.cpp \
../src/marker/markerdetector.cpp \
../src/marker/markerextractor.cpp \
../src/marker/markeridentifier.cpp \
../src/marker/newroimarkerextractor.cpp \
../src/marker/perfectmarkerpredictor.cpp \
../src/marker/roipredictor.cpp \
../src/marker/undistorter.cpp 

OBJS += \
./src/marker/arucomarkerdetector.o \
./src/marker/arucomarkeridentifier.o \
./src/marker/fullmarkerextractor.o \
./src/marker/mappingmarkerpredictor.o \
./src/marker/markerdetector.o \
./src/marker/markerextractor.o \
./src/marker/markeridentifier.o \
./src/marker/newroimarkerextractor.o \
./src/marker/perfectmarkerpredictor.o \
./src/marker/roipredictor.o \
./src/marker/undistorter.o 

CPP_DEPS += \
./src/marker/arucomarkerdetector.d \
./src/marker/arucomarkeridentifier.d \
./src/marker/fullmarkerextractor.d \
./src/marker/mappingmarkerpredictor.d \
./src/marker/markerdetector.d \
./src/marker/markerextractor.d \
./src/marker/markeridentifier.d \
./src/marker/newroimarkerextractor.d \
./src/marker/perfectmarkerpredictor.d \
./src/marker/roipredictor.d \
./src/marker/undistorter.d 


# Each subdirectory must supply rules for building sources it contributes
src/marker/%.o: ../src/marker/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++1y -DRPI -I"/home/pi/fml/FastMarkerLocalizer/src" -O3 -fomit-frame-pointer -pipe -ffast-math -march=armv6j -mfpu=vfp -mfloat-abi=hard -ftree-vectorize -pedantic -Wall -Wextra -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


