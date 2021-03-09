/*
 * differential_speed_controller.h
 *
 *  Created on: May 21, 2020
 *      Author: Ashish Naik
 */

#ifndef DIFFERENTIAL_SPEED_CONTROLLER_H_
#define DIFFERENTIAL_SPEED_CONTROLLER_H_

#include "gpio_motor_driver.h"

typedef enum GPIO_SERVO_PIN_ {
  GPIO_SERVO_PIN_LFWD = 22,
  GPIO_SERVO_PIN_LREV = 23,
  GPIO_SERVO_PIN_RFWD = 24,
  GPIO_SERVO_PIN_RREV = 25
} GPIO_SERVO_PIN;

typedef enum MOTOR_DIRECTION_ {
  MOTOR_DIRECTION_REVERSE = 0,
  MOTOR_DIRECTION_FORWARD = 1
} MOTOR_DIRECTION;

class DifferentialSpeedController {

 private:
  GpioMotorDriver left_motor_;
  GpioMotorDriver right_motor_;
  bool is_initialized_;

 public:
  DifferentialSpeedController();
  ~DifferentialSpeedController();

  inline bool isInitialized() {return is_initialized_;};
  void doDifferentialSteering(MOTOR_DIRECTION left_motor_direction,
                              unsigned int left_motor_dutycycle,
                              MOTOR_DIRECTION right_motor_direction,
                              unsigned int right_motor_dutycycle);

  void stopVehicle();
};


#endif /* DIFFERENTIAL_SPEED_CONTROLLER_H_ */
