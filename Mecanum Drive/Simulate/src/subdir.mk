################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Constants.cpp \
../src/Pickup.cpp \
../src/Robot.cpp \
../src/Switch.cpp 

OBJS += \
./src/Constants.o \
./src/Pickup.o \
./src/Robot.o \
./src/Switch.o 

CPP_DEPS += \
./src/Constants.d \
./src/Pickup.d \
./src/Robot.d \
./src/Switch.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"C:\Users\TechBrick\Desktop\Mecanum Drive/src" -IC:\Users\TechBrick/wpilib/cpp/current/sim/include -I/usr/include -I/usr/include/gazebo-3.1 -I/usr/include/sdformat-2.2 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


