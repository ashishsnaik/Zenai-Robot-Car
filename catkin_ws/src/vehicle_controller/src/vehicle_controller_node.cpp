/*
 * vehicle_controller_node.cpp
 *
 *  Created on: May 22, 2020
 *      Author: Ashish Naik
 */

#include <cstdint>
#include "../include/vehicle_controller/vehicle_controller_node.h"

VehicleControllerNode::VehicleControllerNode(): rate_(vehiclecontrollernode::VEHICLE_CONTROLLER_RATE_HZ) {

  // initialize member variables
  left_motor_direction_ = MOTOR_DIRECTION_FORWARD;
  left_motor_dutycycle_ = 0;
  right_motor_direction_ = MOTOR_DIRECTION_FORWARD;
  right_motor_dutycycle_ = 0;

  // initialize the Subscriber
  sub_ = nh_.subscribe(vehiclecontrollernode::DIFFERENTIAL_STEERING_CMD_TOPIC, // topic
                       vehiclecontrollernode::QUEUE_SIZE, // queue size
                       &VehicleControllerNode::differentialSteeringMessageCallback, // message callback
                       this);

  // spin the node
  loop();
}

VehicleControllerNode::~VehicleControllerNode() {

  left_motor_direction_ = MOTOR_DIRECTION_FORWARD;
  left_motor_dutycycle_ = 0;
  right_motor_direction_ = MOTOR_DIRECTION_FORWARD;
  right_motor_dutycycle_ = 0;

  // stop the vehicle
  differential_speed_controller_.stopVehicle();
}

void VehicleControllerNode::differentialSteeringMessageCallback(const zenai_msgs::DifferentialSteering::ConstPtr& msg) {

  ROS_INFO_STREAM("VehicleControllerNode::DifferentialSteeringMessageCallback (...) called...");

  left_motor_direction_ = (unsigned(msg->left_motor_velocity.direction) == zenai_msgs::MotorVelocity::FORWARD ?
                                                        MOTOR_DIRECTION_FORWARD : MOTOR_DIRECTION_REVERSE);
  left_motor_dutycycle_ = unsigned(msg->left_motor_velocity.dutycycle);

  right_motor_direction_ = (unsigned(msg->right_motor_velocity.direction) == zenai_msgs::MotorVelocity::FORWARD ?
                                                        MOTOR_DIRECTION_FORWARD : MOTOR_DIRECTION_REVERSE);
  right_motor_dutycycle_ = unsigned(msg->right_motor_velocity.dutycycle);

  ROS_INFO_STREAM("Motor Direction/Dutycycle Settings:");
  ROS_INFO_STREAM("Left Motor - direction: " << left_motor_direction_ << " dutycycle: " << left_motor_dutycycle_);
  ROS_INFO_STREAM("Right Motor - direction: " << right_motor_direction_ << " dutycycle: " << right_motor_dutycycle_);

}

void VehicleControllerNode::processDifferentialSteeringMessage() {

  // do differential steering
  if (left_motor_dutycycle_ == 0 && right_motor_dutycycle_ == 0) {
    differential_speed_controller_.stopVehicle();
  } else {
    differential_speed_controller_.doDifferentialSteering(left_motor_direction_, left_motor_dutycycle_,
                                                          right_motor_direction_, right_motor_dutycycle_);
  }

}

void VehicleControllerNode::loop() {

  ROS_INFO_STREAM(vehiclecontrollernode::NODE_NAME << " loop started...");

  while (ros::ok()) {
    processDifferentialSteeringMessage();
    ros::spinOnce();
    sleep();
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, vehiclecontrollernode::NODE_NAME);
  VehicleControllerNode vc_node;
  ROS_INFO_STREAM(vehiclecontrollernode::NODE_NAME << " started...");

  return 0;
}
