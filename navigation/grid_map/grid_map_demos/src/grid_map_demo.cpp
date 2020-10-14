#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>


using namespace grid_map;

using namespace std;
using namespace ros;
 

boost::shared_ptr<NodeHandle> node_; 

Subscriber pcSubscriber;
Publisher  publisher;

grid_map::GridMap Map;
bool   map_avaliable=false;

void pcCallback_map(const grid_map_msgs::GridMap& message)
{
  // Convert message to map.
  GridMapRosConverter::fromMessage(message, Map);
   
  map_avaliable=true;
}
 
 


int main (int argc, char** argv){
 ros::init (argc, argv, "elevation");
 ros::NodeHandle n;
 pcSubscriber = n.subscribe("/elevation_mapping/elevation_map",   200, pcCallback_map);
 publisher = n.advertise<grid_map_msgs::GridMap>("/deneme_visualization/grid_map_deneme", 1, true);

  
 
 printf("Starting ... \n" );
 ros::Rate r(50);
 while(n.ok()){
   ros::Time time = ros::Time::now();
  if (map_avaliable){
    
    /*for (GridMapIterator it(Map); !it.isPastEnd(); ++it) {
    Position position;
    Map.getPosition(*it, position);

    Map.at("elevation", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();

    //cout<<position.x()<<" "<<position.y()<<" "<< Map.at("elevation", *it)<<endl;
    //map.at("elevation", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
  
    }

    // Publish grid map.
    */


      // Create grid map.
    GridMap map({"elevation"});
    map.setFrameId("odom");
    map.setGeometry(Length(1.2, 2.0), 0.03);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));
     ros::Time time = ros::Time::now();
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      map.at("elevation", *it) =1;
    }

    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    
   }
    ros::spinOnce();
    r.sleep();
  }
  return (0);
}
