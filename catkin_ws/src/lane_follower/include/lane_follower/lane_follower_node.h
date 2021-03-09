/*
 * lane_follower_node.h
 *
 *  Created on: Jul 15, 2020
 *      Author: Ashish Naik
 */

#ifndef INCLUDE_LANE_FOLLOWER_LANE_FOLLOWER_NODE_H_
#define INCLUDE_LANE_FOLLOWER_LANE_FOLLOWER_NODE_H_

#include <ros/ros.h>
#include "zenai_msgs/LaneInfo.h"

namespace lanefollowernode {
  constexpr char NODE_NAME[] = "lane_follower_node";
  constexpr double LANE_INFO_RATE_HZ = 1;
  // the minimum radius of curvature for the vehicle to start turning
  constexpr double THRESH_RADIUS_OF_CURVATURE_FOR_TURN = 100000.00;
  constexpr unsigned LANE_INFO_PUB_QUEUE_SIZE = 1;
  constexpr char LANE_INFO_TOPIC[] = "/path/lane_info";
}

class LaneFollowerNode {

 private:
  ros::NodeHandle nh_;
  ros::Publisher lane_info_pub_;
  ros::Rate rate_;

  double radius_of_curvature_;

 public:
  LaneFollowerNode();
  ~LaneFollowerNode();

  inline void sleep() {rate_.sleep();};
  inline ros::Rate getRate() {return rate_;};
  inline void setRate(double rate) {rate_=rate;};

  void loop();

  void publishLaneInfoMessage(zenai_msgs::LaneInfo msg);

};


#endif /* INCLUDE_LANE_FOLLOWER_LANE_FOLLOWER_NODE_H_ */
