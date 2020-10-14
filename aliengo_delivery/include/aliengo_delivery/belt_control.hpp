
#pragma once

#include "aliengo_delivery/belt_control.hpp"

// CPP
#include <stdio.h>
#include <iostream>
#include <cmath>

// ROS
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "aliengo_state_mach/RoverStateMsg.h"
#include "aliengo_state_mach/RoverActionMsg.h" 
// PCL LIBRARY
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>



namespace aliengo_delivery {

class BeltControl {

 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  BeltControl(ros::NodeHandle nodeHandle_);

  /*!
   * Destructor.
   */
  virtual ~BeltControl();
  void pcCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
  double boxDistanceCalculation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudMsg_);
  void applyForce(double x_);
  void stateCallback(const aliengo_state_mach::RoverStateMsg stateMsg);
  double rate = 10;
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud_;  

 private:
  void updateTimerCallback(const ros::TimerEvent& timerEvent);
  //checks if point cloud is empty
  bool checkEmpty(pcl::PointCloud<pcl::PointXYZ>::Ptr  pc);
  //! ROS node handle.
  ros::NodeHandle& n_;
  ros::Subscriber pcSubscriber_;
  ros::Subscriber stateSubscriber_;
  ros::Publisher actionPublisher_;

  ros::Timer updateTimer_;
  ros::ServiceClient forceClient; 
  bool  pc_available = false, do_once = true, apply_force = false, state_available = false;
  double box_distance = 1, c_signal = 0;
  double prev_error = 0;
  double i_error = 0;
  aliengo_state_mach::RoverStateMsg state ; 
  aliengo_state_mach::RoverActionMsg action ;
};
}  // namespace slope_filter