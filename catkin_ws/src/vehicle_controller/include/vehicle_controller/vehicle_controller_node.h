/*
 * vehicle_controller_node.h
 *
 *  Created on: May 22, 2020
 *      Author: Ashish Naik
 */

#ifndef VEHICLE_CONTROLLER_NODE_H_
#define VEHICLE_CONTROLLER_NODE_H_

#include <ros/ros.h>
#include "differential_speed_controller.h"
#include "zenai_msgs/DifferentialSteering.h"

namespace vehiclecontrollernode {
  constexpr char NODE_NAME[] = "vehicle_controller_node";
  constexpr double VEHICLE_CONTROLLER_RATE_HZ = 200;
  constexpr unsigned QUEUE_SIZE = 1;
  constexpr char DIFFERENTIAL_STEERING_CMD_TOPIC[] = "/vehicle/differential_steering_cmd";
}

class VehicleControllerNode {

 private:
  DifferentialSpeedController differential_speed_controller_;
  ros::Subscriber sub_;
  ros::NodeHandle nh_;
  MOTOR_DIRECTION left_motor_direction_;
  unsigned int left_motor_dutycycle_;
  MOTOR_DIRECTION right_motor_direction_;
  unsigned int right_motor_dutycycle_;

  ros::Rate rate_;

 public:
  VehicleControllerNode();
  ~VehicleControllerNode();

  inline void sleep() {rate_.sleep();};
  inline ros::Rate getRate() {return rate_;};
  inline void setRate(double rate) {rate_=rate;};

  void differentialSteeringMessageCallback(const zenai_msgs::DifferentialSteering::ConstPtr& msg);
  void processDifferentialSteeringMessage();
  void loop();
};


#endif /* VEHICLE_CONTROLLER_NODE_H_ */
