/*
 * lane_follower_node.cpp
 *
 *  Created on: Jul 20, 2020
 *      Author: Ashish Naik
 */

#include "../include/lane_follower/lane_follower_node.h"

LaneFollowerNode::LaneFollowerNode(): rate_(lanefollowernode::LANE_INFO_RATE_HZ),
                    radius_of_curvature_(lanefollowernode::THRESH_RADIUS_OF_CURVATURE_FOR_TURN){

  lane_info_pub_ = nh_.advertise<zenai_msgs::LaneInfo>(
                    lanefollowernode::LANE_INFO_TOPIC,
                    lanefollowernode::LANE_INFO_PUB_QUEUE_SIZE);

  // spin the node
  loop();
}

LaneFollowerNode::~LaneFollowerNode() {

}

void LaneFollowerNode::loop() {

  ROS_INFO_STREAM(lanefollowernode::NODE_NAME << " loop started...");

  zenai_msgs::LaneInfo msg;

  while (ros::ok()) {

    msg.radius_of_curvature = radius_of_curvature_;

    lane_info_pub_.publish(msg);
    ros::spinOnce();
    sleep();
  }

}

int main(int argc, char **argv) {
  ros::init(argc, argv, lanefollowernode::NODE_NAME);
  LaneFollowerNode lane_follower_node;
  ROS_INFO_STREAM(lanefollowernode::NODE_NAME << " started...");

}

