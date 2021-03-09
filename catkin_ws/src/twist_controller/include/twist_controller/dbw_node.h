/*
 * dbw_node.h
 *
 *  Created on: Jun 8, 2020
 *      Author: Ashish Naik
 */

#ifndef INCLUDE_TWIST_CONTROLLER_DBW_NODE_H_
#define INCLUDE_TWIST_CONTROLLER_DBW_NODE_H_

#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "zenai_msgs/DifferentialSteering.h"

namespace dbwnode {
  constexpr char NODE_NAME[] = "dbw_node";
  constexpr double DBW_RATE_HZ = 1;
  constexpr unsigned STEERING_CMD_PUB_QUEUE_SIZE = 1;
  constexpr unsigned DBW_ENABLED_SUB_QUEUE_SIZE = 1;
  constexpr char DIFFERENTIAL_STEERING_CMD_TOPIC[] = "/vehicle/differential_steering_cmd";
  constexpr char DBW_ENABLED_TOPIC[] = "/vehicle/dbw_enabled";
}

class DBWNode {

 private:
  ros::NodeHandle nh_;
  ros::Publisher differential_steering_pub_;
  ros::Subscriber dbw_enabled_sub_;
  ros::Rate rate_;

  bool dbw_enabled_;

 public:
  DBWNode();
  ~DBWNode();

  inline void sleep() {rate_.sleep();};
  inline ros::Rate getRate() {return rate_;};
  inline void setRate(double rate) {rate_=rate;};

  void loop();

  void publishDifferentialSteeringMessage(zenai_msgs::DifferentialSteering msg);
  void dbwEnabledCallback(const std_msgs::Bool::ConstPtr& msg);

};

#endif /* INCLUDE_TWIST_CONTROLLER_DBW_NODE_H_ */
