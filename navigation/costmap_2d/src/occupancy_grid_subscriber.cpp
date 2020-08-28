/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Author: David V. Lu!!
 *********************************************************************/
#include <costmap_2d/occupancy_grid_subscriber.h>

namespace costmap_2d
{

OccupancyGridSubscriber::OccupancyGridSubscriber(ros::NodeHandle * ros_node, std::string topic_name) :
    node(ros_node)
{
  costmap_sub_ = ros_node->subscribe<nav_msgs::OccupancyGrid>( topic_name, 1, &OccupancyGridSubscriber::grid_callback, this);
  costmap_update_sub_ = ros_node->subscribe<map_msgs::OccupancyGridUpdate>( topic_name + "_updates", 10, &OccupancyGridSubscriber::update_callback, this );
}

OccupancyGridSubscriber::~OccupancyGridSubscriber()
{
}

int OccupancyGridSubscriber::getIndex(int x, int y){
    int sx = grid_.info.width;
    return y * sx + x;
}

void OccupancyGridSubscriber::grid_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    grid_ = *msg;
}

void OccupancyGridSubscriber::update_callback(const map_msgs::OccupancyGridUpdateConstPtr& msg)
{
    int index = 0;
    for(int y=msg->y; y< msg->y+msg->height; y++){
        for(int x=msg->x; x< msg->x+msg->width; x++){
            grid_.data[ getIndex(x,y) ] = msg->data[ index++ ]; 
        }
    }
}

} // end namespace costmap_2d
