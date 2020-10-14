 // CPP
#include <stdio.h>
#include <iostream>
#include <cmath>
// ROS
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include <grid_map_ros/grid_map_ros.hpp>

 
// PCL LIBRARY
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/registration.h>
#include <pcl/features/normal_3d.h>

 
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



using namespace grid_map;

using namespace std;
using namespace ros;
 

boost::shared_ptr<NodeHandle> node_; 

Subscriber pcSubscriber;
Subscriber odomSubscriber;

Publisher pcPublisher;
Publisher marker_pub;
ros::Publisher grid_pub;
 



pcl::PointCloud<pcl::PointXYZ>::Ptr ugv_cloud          (new pcl::PointCloud<pcl::PointXYZ>);

sensor_msgs::PointCloud2 cloud_msg1;
nav_msgs::Odometry odom_ugv;

bool pc_avaliable = false;
bool do_once=true;

double max_slope;
double min_slope;

void pcCallback_ugv(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
 ugv_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
 pcl::fromROSMsg(*laserCloudMsg, *ugv_cloud);
 pc_avaliable=true;
}
 void odomCallback_ugv( const nav_msgs::Odometry msg){
  odom_ugv=msg;
}

void calculate_visualize_slope( pcl::PointCloud<pcl::Normal>::Ptr &msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_msg)
{
     
    visualization_msgs::MarkerArray markerArray;
    GridMap map({"slope"});
    map.setFrameId("base_link");
	map.setGeometry(Length(10, 10), 0.03, Position(0.0, 0.0));

    double size=msg->points.size();
    tf2::Quaternion q_rot;
    int normal_count=0;
    for(int i=0; i<size; i++)
    {  
       //cout<<msg->points[i].normal_x<<endl;
    	if(sqrt( pow(cloud_msg->points[i].x,2)+ pow(cloud_msg->points[i].y,2))<5.0)
    	{
	      if(!isnan(msg->points[i].normal_z ) && !isnan(msg->points[i].normal_y ) && !isnan(msg->points[i].normal_x ))
	      {
			float n_x=msg->points[i].normal_x;
			float n_y=msg->points[i].normal_y;
			float n_z=msg->points[i].normal_z;

			float c_x=odom_ugv.pose.pose.position.x;
			float c_y=odom_ugv.pose.pose.position.y;
			float c_z=odom_ugv.pose.pose.position.z;
			float slope= acos((n_z-c_z)/ sqrt(pow(n_z-c_z,2)+pow(n_y-c_y,2)+ pow(n_x-c_x,2)));
			float max_slope=45*3.14/180;

			Position position;
			position.x()=cloud_msg->points[i].x;
			position.y()=cloud_msg->points[i].y;
			map.atPosition("slope", position) =1-slope/max_slope;
	       }	    
		}
	}

	 for (GridMapIterator it(map); !it.isPastEnd(); ++it) 
	 {
      Position position;
      map.getPosition(*it, position);
      cout<<map.at("slope", *it) ;     
  	 }
	
	cout<<"finished"<<endl;

    ros::Time time = ros::Time::now();


	 // Publish grid map.
	map.setTimestamp(time.toNSec());
	grid_map_msgs::GridMap message;
	GridMapRosConverter::toMessage(map, message);
	grid_pub.publish(message);
    marker_pub.publish(markerArray);
}


void calculate_normals(){
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (ugv_cloud);
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // Use all neighbors in a sphere of radius 30cm
  ne.setRadiusSearch (0.3);
  // Compute the features
  ne.compute (*cloud_normals);
  calculate_visualize_slope(cloud_normals, ugv_cloud);
}





int main (int argc, char** argv){
	ros::init (argc, argv, "slope_calculation");
	ros::NodeHandle n;
	pcSubscriber = n.subscribe("/ugv/octomap_point_cloud_centers",   200, pcCallback_ugv);  ///elevation_map_raw_visualization/elevation_cloud
	odomSubscriber = n.subscribe("/husky_velocity_controller/odom",   200, odomCallback_ugv);
	grid_pub = n.advertise<grid_map_msgs::GridMap>("slope_grid_map", 1, true);

	marker_pub=n.advertise< visualization_msgs::MarkerArray>("marker_slope",1,false);
	 
	
	 

	n.getParam("/slope_calculation/max_slope", max_slope);
	n.getParam("/slope_calculation/min_slope", min_slope);
	 
	printf("Starting Slope Calculation... \n" );
	ros::Rate r(10);
	while(n.ok()){
		if (pc_avaliable){
			calculate_normals(); 
		}
		ros::spinOnce();
		r.sleep();
		}
	return (0);
}