################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/dbw_node.cpp 

OBJS += \
./src/dbw_node.o 

CPP_DEPS += \
./src/dbw_node.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++0x -I/usr/local/include -I/opt/ros/kinetic/include -I/home/student/ROS_Programming_for_Robotics/Zenai/catkin_ws/devel/include -I/usr/include -I/usr/arm-linux-gnueabihf/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


