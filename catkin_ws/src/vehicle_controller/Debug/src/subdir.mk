################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/differential_speed_controller.cpp \
../src/gpio_motor_driver.cpp \
../src/vehicle_controller_node.cpp 

OBJS += \
./src/differential_speed_controller.o \
./src/gpio_motor_driver.o \
./src/vehicle_controller_node.o 

CPP_DEPS += \
./src/differential_speed_controller.d \
./src/gpio_motor_driver.d \
./src/vehicle_controller_node.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++0x -I/usr/local/include -I/opt/ros/kinetic/include -I/home/student/ROS_Programming_for_Robotics/Zenai/catkin_ws/devel/include -I/usr/include -I/usr/arm-linux-gnueabihf/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


