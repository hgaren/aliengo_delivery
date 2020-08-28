
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
  //point cloud call back used for slope calculation
  void pcCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
  double boxDistanceCalculation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudMsg_);
  void applyForce(double x_);

  double rate = 10;
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud_;  

 private:
  void updateTimerCallback(const ros::TimerEvent& timerEvent);
  //checks if point cloud is empty
  bool checkEmpty(pcl::PointCloud<pcl::PointXYZ>::Ptr  pc);
  //! ROS node handle.
  ros::NodeHandle& n_;
  ros::Subscriber pcSubscriber_;
  ros::Timer updateTimer_;
  ros::ServiceClient forceClient; 
  bool  pc_available = false, do_once = true;
  double prev_error = 0;
  double i_error = 0;
};
}  // namespace slope_filter