/*
 * gpio_motor_driver.cpp
 *
 *  Created on: Apr 14, 2020
 *      Author: Ashish Naik
 */

#include "../include/vehicle_controller/gpio_motor_driver.h"

#include <unistd.h>
#include <cstdlib>
#include <ros/ros.h>
#include "pigpio.h"


GpioMotorDriver::GpioMotorDriver(unsigned int forward_pin, unsigned int reverse_pin, string motor_name = "DCMotor") {

  forward_pin_ = forward_pin;
  reverse_pin_ = reverse_pin;
  motor_is_running_ = false;
  name_ = motor_name;

  // initialize the Raspberry Pi GPIO
  if (gpioInitialise() < 0) {
    is_initialized_ = false;
    ROS_ERROR_STREAM("GPIO failed to initialize for " << name_);
  } else {
    is_initialized_ = true;
    // set both the pins to output
    gpioSetMode(forward_pin_, PI_OUTPUT);
    gpioSetMode(reverse_pin_, PI_OUTPUT);
    ROS_INFO_STREAM("GPIO initialized for " << name_);
  }
}

GpioMotorDriver::~GpioMotorDriver() {
  motor_is_running_ = false;
  gpioTerminate();
}

void GpioMotorDriver::runForward(unsigned int duty_cycle, bool use_motor_boost) {

  if (is_initialized_) {
    // stop any backward motion
    gpioPWM(reverse_pin_, 0);

    // use motor boost in case the duty cycle (speed) is below a threshold,
    // so the wheel(s) starts to rotate...
    if (!motor_is_running_ && use_motor_boost && duty_cycle < gpiomotordriver::MOTOR_BOOST_DUTYCYCLE_THRESH) {
      gpioPWM(forward_pin_, gpiomotordriver::MAX_DUTYCYCLE);
      motor_is_running_ = true;
      usleep(gpiomotordriver::MOTOR_BOOST_T_USEC);
    }
    // ...now continue running the motor at the requested duty cycle
    gpioPWM(forward_pin_, duty_cycle);

    ROS_DEBUG_STREAM("Motor " << name_ << " Running forward with Duty Cycle " << duty_cycle);

  } else {
    ROS_ERROR_STREAM("GPIO NOT initialized for " << name_);
  }
}

void GpioMotorDriver::runBackward(unsigned int duty_cycle, bool use_motor_boost) {

  if (is_initialized_) {

    // stop any forward motion
    gpioPWM(forward_pin_, 0);

    // use motor boost in case the duty cycle (speed) is below a threshold,
    // so the wheel(s) starts to rotate...
    if (!motor_is_running_ && use_motor_boost && duty_cycle < gpiomotordriver::MOTOR_BOOST_DUTYCYCLE_THRESH) {
      gpioPWM(reverse_pin_, gpiomotordriver::MAX_DUTYCYCLE);
      motor_is_running_ = true;
      usleep(gpiomotordriver::MOTOR_BOOST_T_USEC);
    }
    // ...now continue running the motor at the requested duty cycle
    gpioPWM(reverse_pin_, duty_cycle);
    ROS_DEBUG_STREAM("Motor " << name_ << " Running forward with Duty Cycle " << duty_cycle);

  } else {
    ROS_ERROR_STREAM("GPIO NOT initialized for " << name_);
  }
}

void GpioMotorDriver::stop() {
  gpioPWM(forward_pin_, 0);
  gpioPWM(reverse_pin_, 0);
  motor_is_running_ = false;
  ROS_DEBUG_STREAM("Motor " << name_ << " Stopped!");
}
