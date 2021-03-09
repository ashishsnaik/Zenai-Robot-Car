/*
 * gpio_motor_driver.h
 *
 *  Created on: Apr 14, 2020
 *      Author: Ashish Naik
 */

#ifndef GPIO_MOTOR_DRIVER_H_
#define GPIO_MOTOR_DRIVER_H_

#include<string>

using std::string;

// global constants in GpioMotorDriver namespace
namespace gpiomotordriver {
  constexpr unsigned int MAX_DUTYCYCLE = 255;
  constexpr unsigned int MOTOR_BOOST_T_USEC = 50000;
  constexpr unsigned int MOTOR_BOOST_DUTYCYCLE_THRESH = 150;
}

class GpioMotorDriver {

private:
  unsigned int forward_pin_;
  unsigned int reverse_pin_;
  bool motor_is_running_;
  bool is_initialized_;
  string name_;

public:
  GpioMotorDriver(unsigned int forward_pin, unsigned int reverse_pin, string motor_name);
  ~GpioMotorDriver();

  inline bool isInitialized() {return is_initialized_;};

  // motor functionality
  void runForward(unsigned int duty_cycle, bool use_motor_boost = true);
  void runBackward(unsigned int duty_cycle, bool use_motor_boost = true);
  void stop();
};


#endif /* GPIO_MOTOR_DRIVER_H_ */
