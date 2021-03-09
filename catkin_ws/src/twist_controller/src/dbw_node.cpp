/*
 * dbw_node.cpp
 *
 *  Created on: Jun 8, 2020
 *      Author: Ashish Naik
 */


#include "../include/twist_controller/dbw_node.h"

DBWNode::DBWNode() : rate_(dbwnode::DBW_RATE_HZ), dbw_enabled_(false) {

  differential_steering_pub_ = nh_.advertise<zenai_msgs::DifferentialSteering>(
                                dbwnode::DIFFERENTIAL_STEERING_CMD_TOPIC,
                                dbwnode::STEERING_CMD_PUB_QUEUE_SIZE);

  dbw_enabled_sub_ = nh_.subscribe(dbwnode::DBW_ENABLED_TOPIC,
                                   dbwnode::DBW_ENABLED_SUB_QUEUE_SIZE,
                                   &DBWNode::dbwEnabledCallback,
                                   this);

  // spin the node
  loop();
}

DBWNode::~DBWNode() {

}

void DBWNode::dbwEnabledCallback(const std_msgs::Bool::ConstPtr& msg) {
  dbw_enabled_ = msg->data;
//  const char *status_str = ((dbw_enabled_ == true) ? "ENABLED" : "DISABLED");
  ROS_INFO_STREAM("Vehicle Drive-by-wire " << ((dbw_enabled_ == true) ? "ENABLED" : "DISABLED"));

  // TODO: CHECK!! if the dbw is disabled, stop the motor
  if (!dbw_enabled_) {
    ROS_INFO_STREAM("Stopping the vehicle...");
    zenai_msgs::DifferentialSteering msg;
    msg.left_motor_velocity.direction = zenai_msgs::MotorVelocity::FORWARD;
    msg.right_motor_velocity.direction = zenai_msgs::MotorVelocity::FORWARD;
    msg.left_motor_velocity.dutycycle = 0;
    msg.right_motor_velocity.dutycycle = 0;
    publishDifferentialSteeringMessage(msg);
  }
}

void DBWNode::publishDifferentialSteeringMessage(zenai_msgs::DifferentialSteering msg) {
  differential_steering_pub_.publish(msg);
}

void DBWNode::loop() {

  ROS_INFO_STREAM(dbwnode::NODE_NAME << " loop started...");

  zenai_msgs::DifferentialSteering msg;

  msg.left_motor_velocity.direction = zenai_msgs::MotorVelocity::FORWARD;
  msg.right_motor_velocity.direction = zenai_msgs::MotorVelocity::FORWARD;
  msg.left_motor_velocity.dutycycle = 250;
  msg.right_motor_velocity.dutycycle = 250;

  while (ros::ok()) {
    if (dbw_enabled_) {
      publishDifferentialSteeringMessage(msg);
    }
    ros::spinOnce();
    sleep();
  }
}

int main (int argc, char **argv) {

  ros::init(argc, argv, dbwnode::NODE_NAME);
  DBWNode dbw_node;
  ROS_INFO_STREAM(dbwnode::NODE_NAME << " started...");

  return 0;

//  ROS_INFO_STREAM("dbw_node started...");
//
//  ros::init(argc, argv, "dbw_node");
//
//  DBWNode dbw_node;
//
//  zenai_msgs::DifferentialSteering msg;
//  unsigned int l_dutycycle = 200;
//  unsigned int r_dutycycle = 200;
//
//  msg.left_motor_velocity.direction = zenai_msgs::MotorVelocity::FORWARD;
//  msg.left_motor_velocity.dutycycle = l_dutycycle;
//  msg.right_motor_velocity.direction = zenai_msgs::MotorVelocity::FORWARD;
//  msg.right_motor_velocity.dutycycle = r_dutycycle;
//
//
//  while (ros::ok()) {
//
//    dbw_node.publishDifferentialSteeringMessage(msg);
//    ros::spinOnce();
//    dbw_node.sleep();
//
//    if (l_dutycycle < zenai_msgs::MotorVelocity::MAX_DUTY_CYCLE - 5) {
//      l_dutycycle = r_dutycycle += 5;
//      msg.left_motor_velocity.dutycycle = l_dutycycle;
//      msg.right_motor_velocity.dutycycle = r_dutycycle;
//    }
//
//  }
//
//  // stop the vehicle (TODO: Use a different message?)
//  msg.left_motor_velocity.dutycycle = 0;
//  msg.right_motor_velocity.dutycycle = 0;
//  dbw_node.publishDifferentialSteeringMessage(msg);
//
//
//  return 0;
//
}
