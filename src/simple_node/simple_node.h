/** Author(s): Kelvin Kang (kelvinkang@cmu.edu)
 *  This file is subject to the terms and conditions defined in the file 'LICENSE',
 *  which is part of this source code package
 */

// always add include guard for all .h files
#pragma once

// std libraries
#include <iostream>
#include <vector>
#include <cmath>

// ROS libraries
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf/tf.h>

// Other includes
#include "myPidController.h"
#include "myVehState.h"

class SimpleNode
{
public:
  SimpleNode();

private:
  // State variables
  MyVehState vehicleState_;
  double currVel_;
  sensor_msgs::Imu currImu_;

  // calculation variables
  int myVar1_, myVar3_;
  double myVar2_, myVar4_;

  // put your most important function and other helper functions here
  void run();
  double calcEuclDist(double x1, double x2, double y1, double y2);

  // ROS and node related variables
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber odomSub_, imuSub_;
  ros::Publisher vehicleCmdPub_, vehicleStatsPub_;
  ros::Timer timer_;
  double timerFreq_;
  geometry_msgs::TwistWithCovarianceStamped vehicleCmdMsg_;
  std_msgs::String vehicleStatsMsg_;

  // Boiler plate ROS functions
  void initRos();
  void timerCallback(const ros::TimerEvent& e);
  void odomCallback(const nav_msgs::Odometry& odomMsg);
  void imuCallback(const sensor_msgs::Imu& imuMsg);
};
