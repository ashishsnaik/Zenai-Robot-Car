/*
 * differential_speed_controller.cpp
 *
 *  Created on: May 21, 2020
 *      Author: Ashish Naik
 */

#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include "../include/vehicle_controller/differential_speed_controller.h"


DifferentialSpeedController::DifferentialSpeedController():
    left_motor_(GpioMotorDriver(GPIO_SERVO_PIN_LFWD, GPIO_SERVO_PIN_LREV, "Left Motor")),
    right_motor_(GpioMotorDriver(GPIO_SERVO_PIN_RFWD, GPIO_SERVO_PIN_RREV, "Right Motor")){

  is_initialized_ = left_motor_.isInitialized() && right_motor_.isInitialized();

}

DifferentialSpeedController::~DifferentialSpeedController() {
  // Clean-up here!
}

void DifferentialSpeedController::doDifferentialSteering(MOTOR_DIRECTION left_motor_direction,
                                                         unsigned int left_motor_dutycycle,
                                                         MOTOR_DIRECTION right_motor_direction,
                                                         unsigned int right_motor_dutycycle) {

  // set left motor direction and speed
  if (left_motor_dutycycle > gpiomotordriver::MAX_DUTYCYCLE) {
    left_motor_dutycycle = gpiomotordriver::MAX_DUTYCYCLE;
  }

  if (left_motor_direction == MOTOR_DIRECTION_FORWARD) {
    left_motor_.runForward(left_motor_dutycycle);
  } else {
    left_motor_.runBackward(left_motor_dutycycle);
  }

  // set right motor direction and speed
  if (right_motor_dutycycle > gpiomotordriver::MAX_DUTYCYCLE) {
    right_motor_dutycycle = gpiomotordriver::MAX_DUTYCYCLE;
  }

  if (right_motor_direction == MOTOR_DIRECTION_FORWARD) {
    right_motor_.runForward(right_motor_dutycycle);
  } else {
    right_motor_.runBackward(right_motor_dutycycle);
  }
}

void DifferentialSpeedController::stopVehicle() {

  ROS_INFO_STREAM("DifferentialSpeedController::stopVehicle(...) called...");
  left_motor_.stop();
  right_motor_.stop();

}
