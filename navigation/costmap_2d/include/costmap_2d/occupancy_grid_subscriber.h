/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Author: David V. Lu!!
 *********************************************************************/
#ifndef COSTMAP_2D_OCCUPANCY_GRID_SUBSCRIBER_H_
#define COSTMAP_2D_OCCUPANCY_GRID_SUBSCRIBER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>

namespace costmap_2d
{
/**
 * @class OccupancyGridSubscriber
 * @brief A tool to subscribe to visualization data from a Costmap2D
 */
class OccupancyGridSubscriber
{
public:
  /**
   * @brief  Constructor for the OccupancyGridSubscriber
   */
  OccupancyGridSubscriber(ros::NodeHandle * ros_node, std::string topic_name);

  /**
   * @brief  Destructor
   */
  ~OccupancyGridSubscriber();

  int getIndex(int x, int y);

  nav_msgs::OccupancyGrid* getOccupancyGrid(){ return &grid_; }

private:
  void grid_callback(const nav_msgs::OccupancyGridConstPtr& msg);
  void update_callback(const map_msgs::OccupancyGridUpdateConstPtr& msg);

  ros::NodeHandle* node;
  ros::Subscriber costmap_sub_;
  ros::Subscriber costmap_update_sub_;
  nav_msgs::OccupancyGrid grid_;
};
}
#endif
