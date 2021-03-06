#ifndef GRID_LAYER_H_
#define GRID_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include "grid_map_msgs/GridMap.h"
#include <grid_map_ros/grid_map_ros.hpp>

namespace costmap_2d
{

class GridLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  GridLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();
    double robotYaw_;
  double robotX_;
  double robotY_;
 
protected:
    void gridcallback(const grid_map_msgs::GridMap gridmapMsg_);
    ros::Subscriber people_sub_;
private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  grid_map_msgs::GridMap gridmap_;
  bool gridmapAvaliable_;
  double costmapBuffer_[1000];

};
}
#endif