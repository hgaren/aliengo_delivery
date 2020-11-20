/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <quadruped_awareness_controller.h>
#include <tf/tf.h>
#include "aliengo_msgs/LowCmd.h"
#include "aliengo_msgs/LowState.h"
#include "aliengo_msgs/MotorCmd.h"
#include "aliengo_msgs/MotorState.h"
#include "aliengo_msgs/Foots.h"
#include "aliengo_msgs/GaitInfo.h"
#include "geometry_msgs/Point.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>

QuadrupedFootController::QuadrupedFootController(const ros::NodeHandle &node_handle,
                                         const ros::NodeHandle &private_node_handle):
    nh_(node_handle),
    pnh_(private_node_handle),
    body_controller_(base_),
    leg_controller_(base_),
    kinematics_(base_),
    odometry_(base_)
{
    //initializations

	servo_sub[0] = nh_.subscribe("/laikago_gazebo/FR_hip_controller/state", 1, &QuadrupedFootController::FRhipCallback, this);
	servo_sub[1] = nh_.subscribe("/laikago_gazebo/FR_thigh_controller/state", 1, &QuadrupedFootController::FRthighCallback, this);
	servo_sub[2] = nh_.subscribe("/laikago_gazebo/FR_calf_controller/state", 1, &QuadrupedFootController::FRcalfCallback, this);
	servo_sub[3] = nh_.subscribe("/laikago_gazebo/FL_hip_controller/state", 1, &QuadrupedFootController::FLhipCallback, this);
	servo_sub[4] = nh_.subscribe("/laikago_gazebo/FL_thigh_controller/state", 1, &QuadrupedFootController::FLthighCallback, this);
	servo_sub[5] = nh_.subscribe("/laikago_gazebo/FL_calf_controller/state", 1, &QuadrupedFootController::FLcalfCallback, this);
	servo_sub[6] = nh_.subscribe("/laikago_gazebo/RR_hip_controller/state", 1, &QuadrupedFootController::RRhipCallback, this);
	servo_sub[7] = nh_.subscribe("/laikago_gazebo/RR_thigh_controller/state", 1, &QuadrupedFootController::RRthighCallback, this);
	servo_sub[8] = nh_.subscribe("/laikago_gazebo/RR_calf_controller/state", 1, &QuadrupedFootController::RRcalfCallback, this);
	servo_sub[9] = nh_.subscribe("/laikago_gazebo/RL_hip_controller/state", 1, &QuadrupedFootController::RLhipCallback, this);
	servo_sub[10] = nh_.subscribe("/laikago_gazebo/RL_thigh_controller/state", 1, &QuadrupedFootController::RLthighCallback, this);
	servo_sub[11] = nh_.subscribe("/laikago_gazebo/RL_calf_controller/state", 1, &QuadrupedFootController::RLcalfCallback, this);
    

  servo_pub[0] = nh_.advertise<aliengo_msgs::MotorCmd>("/laikago_gazebo/FR_hip_controller/command", 1);
  servo_pub[1] = nh_.advertise<aliengo_msgs::MotorCmd>("/laikago_gazebo/FR_thigh_controller/command", 1);
  servo_pub[2] = nh_.advertise<aliengo_msgs::MotorCmd>("/laikago_gazebo/FR_calf_controller/command", 1);
  servo_pub[3] = nh_.advertise<aliengo_msgs::MotorCmd>("/laikago_gazebo/FL_hip_controller/command", 1);
  servo_pub[4] = nh_.advertise<aliengo_msgs::MotorCmd>("/laikago_gazebo/FL_thigh_controller/command", 1);
  servo_pub[5] = nh_.advertise<aliengo_msgs::MotorCmd>("/laikago_gazebo/FL_calf_controller/command", 1);
  servo_pub[6] = nh_.advertise<aliengo_msgs::MotorCmd>("/laikago_gazebo/RR_hip_controller/command", 1);
  servo_pub[7] = nh_.advertise<aliengo_msgs::MotorCmd>("/laikago_gazebo/RR_thigh_controller/command", 1);
  servo_pub[8] = nh_.advertise<aliengo_msgs::MotorCmd>("/laikago_gazebo/RR_calf_controller/command", 1);
  servo_pub[9] = nh_.advertise<aliengo_msgs::MotorCmd>("/laikago_gazebo/RL_hip_controller/command", 1);
  servo_pub[10] = nh_.advertise<aliengo_msgs::MotorCmd>("/laikago_gazebo/RL_thigh_controller/command", 1);
  servo_pub[11] = nh_.advertise<aliengo_msgs::MotorCmd>("/laikago_gazebo/RL_calf_controller/command", 1);
 

  cmd_vel_subscriber_ = nh_.subscribe("cmd_vel/smooth", 1, &QuadrupedFootController::cmdVelCallback_, this);
  cmd_pose_subscriber_ = nh_.subscribe("/aliengo/ref_odom", 1, &QuadrupedFootController::cmdPoseCallback_, this);
  foot_subscriber_ = nh_.subscribe("/aliengo/ref_foot", 1, &QuadrupedFootController::footCallback_, this);
  gait_subscriber_ = nh_.subscribe("/aliengo/gait_info", 1, &QuadrupedFootController::gaitCallback_, this);
  odomSubscriber_=nh_.subscribe("/aliengo/odometry", 1, &QuadrupedFootController::odomCallback, this);

  //subs elevation map
  gridmapSubscriber_=nh_.subscribe("/traversability_gridmap", 1, &QuadrupedFootController::gridmapCallback, this); ///elevation_mapping_gridmap/local
  markerPublisher_=nh_.advertise<visualization_msgs::MarkerArray>("awareness_foot_marker",1,false);



    velocities_publisher_   = nh_.advertise<nav_msgs::Odometry>("odom/raw", 100);
    foot_publisher_   = nh_.advertise<visualization_msgs::MarkerArray>("foot", 100);

	for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].positionStiffness = 70;
        lowCmd.motorCmd[i*3+0].velocity = 0;
        lowCmd.motorCmd[i*3+0].velocityStiffness = 3;
        lowCmd.motorCmd[i*3+0].torque = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].positionStiffness = 180;
        lowCmd.motorCmd[i*3+1].velocity = 0;
        lowCmd.motorCmd[i*3+1].velocityStiffness = 8;
        lowCmd.motorCmd[i*3+1].torque = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].positionStiffness = 300;
        lowCmd.motorCmd[i*3+2].velocity = 0;
        lowCmd.motorCmd[i*3+2].velocityStiffness = 15;
        lowCmd.motorCmd[i*3+2].torque = 0;
	    }
    for(int i=0; i<12; i++){
        lowCmd.motorCmd[i].position = lowState.motorState[i].position;
    }

    std::string knee_orientation;
    nh_.getParam("gait/pantograph_leg",         gait_config_.pantograph_leg);
    nh_.getParam("gait/max_linear_velocity_x",  gait_config_.max_linear_velocity_x);
    nh_.getParam("gait/max_linear_velocity_y",  gait_config_.max_linear_velocity_y);
    nh_.getParam("gait/max_angular_velocity_z", gait_config_.max_angular_velocity_z);
    nh_.getParam("gait/com_x_translation",      gait_config_.com_x_translation);
    nh_.getParam("gait/com_y_translation",      gait_config_.com_y_translation);
    nh_.getParam("gait/swing_height",           gait_config_.swing_height);
    nh_.getParam("gait/stance_depth",           gait_config_.stance_depth);
    nh_.getParam("gait/stance_duration",        gait_config_.stance_duration);
    nh_.getParam("gait/nominal_height",         gait_config_.nominal_height);
    nh_.getParam("gait/knee_orientation",       knee_orientation);
    nh_.getParam("links_map/base",              base_name_);
    pnh_.getParam("gazebo",                     in_gazebo_);
    gait_config_.knee_orientation = knee_orientation.c_str();
    
    base_.setGaitConfig(gait_config_);
    champ::URDF::loadFromServer(base_, nh_);
    joint_names_ = champ::URDF::getJointNames(nh_);

    node_namespace_ = ros::this_node::getNamespace();
    if(node_namespace_.length() > 1)
    {
        node_namespace_.replace(0, 1, "");
        node_namespace_.push_back('/');
    }
    else
    {
        node_namespace_ = "";
    }
    odom_frame_ = node_namespace_ + "odom";
    base_footprint_frame_ = node_namespace_ + "base";
    base_link_frame_ = node_namespace_ + base_name_;

    loop_timer_ = pnh_.createTimer(ros::Duration(0.0005),
                                   &QuadrupedFootController::controlLoop_,
                                   this);

    odom_data_timer_ = pnh_.createTimer(ros::Duration(0.005),
                                        &QuadrupedFootController::publishVelocities_, 
                                        this);

    foot_position_timer_ = pnh_.createTimer(ros::Duration(0.005),
                                            &QuadrupedFootController::publishFootPositions_, 
                                            this);

    req_pose_.position.z = gait_config_.nominal_height;
}

void QuadrupedFootController::controlLoop_(const ros::TimerEvent& event)
{
   
    // foot pose reference mode
    if(linear_config){
       gait_config_.swing_height = 0.14;
       gait_config_.stance_duration = 0.4;
       gait_config_.nominal_height =   0.35;
      // base_.setGaitConfig(gait_config_);
      // cout <<"linear"<<endl;
    }
    else if(angular_config){
       gait_config_.swing_height = 0.05;
       gait_config_.stance_duration = 0.4;
       gait_config_.nominal_height =  0.35;
       base_.setGaitConfig(gait_config_);
       cout<<"angular"<<endl;
    }
    else if(hybrid_config){
       gait_config_.swing_height = 0.1;
       gait_config_.stance_duration = 0.4;
       gait_config_.nominal_height =  0.35;
       base_.setGaitConfig(gait_config_);
       cout<<"hybrid"<<endl;
    }
    float target_joint_positions[12];

  	//calculates joint positions according to ref orientation and ref velocity
    body_controller_.poseCommand(target_foot_positions_, req_pose_);
    leg_controller_.velocityCommand(target_foot_positions_, req_vel_);
    
    /*
    if(!walking_available && foot_ref_available){
      int ref_id = int(foots.foot[0].id);
      stabilizeCoM_(ref_id,current_foot_positions_);
    	// FR 0 , FL 1, RR 2 , RL 3
 		  target_foot_positions_[ref_id].Translate(foots.foot[0].pose.x,foots.foot[0].pose.y,foots.foot[0].pose.z);
    }

    if(walking_available && !foot_ref_available && !gridmap_available)  {
      //walking mode
      cout<<"walking_mode"<<endl;
      gait_config_.com_x_translation = 0 ;
      gait_config_.com_y_translation = 0 ;
      base_.setGaitConfig(gait_config_);
    }
    */


    if(gridmap_available && odom_available ){
      geometry::Transformation new_foot_positions_[4];
      geometry::Transformation original_foot_positions_[4];

      bool no_warning = true;
      bool contact_none =  false;
      for (int id=0; id<4; id++ ){
        grid_map::Position position;
        grid_map::Position avFootInCircle;
        avFootInCircle.x() = NAN;
        avFootInCircle.y() = NAN;
       

        new_foot_positions_[id].X() = current_foot_positions_[id].X()+target_foot_positions_[id].X();
        new_foot_positions_[id].Y() = current_foot_positions_[id].Y()+target_foot_positions_[id].Y();
        new_foot_positions_[id].Z() = target_foot_positions_[id].Z();
        
        original_foot_positions_[id].X() = current_foot_positions_[id].X()+target_foot_positions_[id].X();
        original_foot_positions_[id].Y() = current_foot_positions_[id].Y()+target_foot_positions_[id].Y();
        original_foot_positions_[id].Z() = target_foot_positions_[id].Z();

        position.x()=new_foot_positions_[id].X()+ odom_.pose.pose.position.x;
        position.y()=new_foot_positions_[id].Y() + odom_.pose.pose.position.y;

        double roll,pitch,yaw;
        tf::Quaternion q(
        odom_.pose.pose.orientation.x,
        odom_.pose.pose.orientation.y,
        odom_.pose.pose.orientation.z,
        odom_.pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

         if(grid_map::checkIfPositionWithinMap(position,inputGridMap_.getLength(),inputGridMap_.getPosition())){
          grid_map::Index index;
          inputGridMap_.getIndex(position,index);    

          if(isnan(inputGridMap_.at("traversability", index)))
            continue;
          if(inputGridMap_.at("traversability", index) < 0.29)
          {          
            no_warning = false;
            cout<<id<<" WARNING"<<endl;
            

            double radius = 0.05;
            position.x() = position.x() + 2*cos(yaw);
            position.y() = position.y() + 2*sin(yaw);


            if (target_foot_positions_[id].Z()>-0.29){
              //stabilizeCoM_(id,current_foot_positions_);  
              //cout<<"elevation: "<<inputGridMap_.at("elevation", index)<<endl;
              new_foot_positions_[id].Z() += 0.05;
              //target_foot_positions_[id].Z() = -0.20;
              contact_none = true;
            }
            else
              contact_none = false;

           
            while(isnan(avFootInCircle.x()) && isnan(avFootInCircle.y())){
              for (grid_map::CircleIterator circleIt(inputGridMap_, position, radius); !circleIt.isPastEnd(); ++circleIt) {
                if (isnan(inputGridMap_.at("traversability", *circleIt)))
                  continue;
                if(inputGridMap_.at("traversability", *circleIt) >= 0.29){
                  inputGridMap_.getPosition(*circleIt, avFootInCircle);
                  new_foot_positions_[id].X() = avFootInCircle.x() -  odom_.pose.pose.position.x ;
                  //new_foot_positions_[id].Y() = avFootInCircle.y() -  odom_.pose.pose.position.y;
          

                 //if(contact_none){
                   // new_foot_positions_[id].X() = original_foot_positions_[id].X();
                  // new_foot_positions_[id].X() = (new_foot_positions_[id].X()+original_foot_positions_[id].X())/2;
                // }
                 
                }
              }
              radius += 0.05;
              if(radius>=0.25){
                cout<<"could not find proper foot placement"<<endl;
                break;
              }
            } 

            
            
          }
        }

        target_foot_positions_[id].X() = new_foot_positions_[id].X() - current_foot_positions_[id].X();
        target_foot_positions_[id].Y() = new_foot_positions_[id].Y() - current_foot_positions_[id].Y();
        target_foot_positions_[id].Z() = new_foot_positions_[id].Z();
        //new_foot_positions_[id].X() = current_foot_positions_[id].X()+target_foot_positions_[id].X();
        //new_foot_positions_[id].Y() = current_foot_positions_[id].Y()+target_foot_positions_[id].Y();
        //new_foot_positions_[id].Z() = target_foot_positions_[id].Z();
        
        cout<<"target: "<<id<<" "<<target_foot_positions_[id].X() <<" "<<target_foot_positions_[id].Y()<<" "<<target_foot_positions_[id].Z()<<endl;
   
      } 
      
      visualizeFootAwareness(new_foot_positions_,original_foot_positions_);
      gridmap_available = false;

    }
 
    


    //converts foot pose to joint angles
    kinematics_.inverse(target_joint_positions, target_foot_positions_);
    //registers current joint positions (feedback coming from joints)
   	for(int i=0; i<12; i++)
    	current_joint_positions_[i]=lowState.motorState[i].position;
    //sends to joint controller for actuating through joint positions 
    moveAllPosition(target_joint_positions, 1);
    //updates base and odom according to feedback
    base_.updateJointPositions(current_joint_positions_);
    base_.getFootPositions(current_foot_positions_);
    odometry_.getVelocities(current_velocities_);
}

void QuadrupedFootController::cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg)
{
    req_vel_.linear.x = msg->linear.x;
    req_vel_.linear.y = msg->linear.y;
    req_vel_.angular.z = msg->angular.z;

    if(req_vel_.linear.x != 0.0 || req_vel_.linear.y != 0.0 || req_vel_.angular.z != 0.0 ){
	    walking_available = true;
	    foot_ref_available = false;
	}
    if(req_vel_.linear.x != 0.0){
        linear_config = false;
        angular_config = false;
        hybrid_config = false;
    }
    if(req_vel_.angular.z != 0.0 ){
        angular_config = true;
        hybrid_config = false;
        linear_config = false;

    }
    if(req_vel_.linear.x != 0.0 && req_vel_.angular.z != 0.0 ){
        angular_config = false;
        hybrid_config = true;
        linear_config = false;
    }
}
 
void QuadrupedFootController::odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    if(!isnan(msg->pose.pose.orientation.x)) {
      odom_ = *msg;
      odom_available=true;
    }
    else{
      ROS_INFO("odometry is empty");
      odom_available=false;
    }
  }
 void QuadrupedFootController::footCallback_(const aliengo_msgs::Foots& foot_msg)
{
 	foots = foot_msg ; 
 	
 	walking_available = false;
    foot_ref_available = true;

}
 void QuadrupedFootController::gaitCallback_(const aliengo_msgs::GaitInfo& gait_msg)
{
	if(gait_msg.max_l_x != 0.0)	
		gait_config_.max_linear_velocity_x = gait_msg.max_l_x;
	if(gait_msg.max_l_y != 0.0)	
		gait_config_.max_linear_velocity_y = gait_msg.max_l_y;
	if(gait_msg.max_a_z != 0.0)	
		gait_config_.max_angular_velocity_z = gait_msg.max_a_z;
	if(gait_msg.com_x_t != 0.0)	
		gait_config_.com_x_translation = gait_msg.com_x_t;
	if(gait_msg.com_y_t != 0.0)	
		gait_config_.com_y_translation = gait_msg.com_y_t;
	if(gait_msg.swing_h != 0.0)	
		gait_config_.swing_height = gait_msg.swing_h;
	if(gait_msg.stance_d != 0.0)	
		gait_config_.stance_duration = gait_msg.stance_d;
	if(gait_msg.nominal_h != 0.0)	
		gait_config_.nominal_height = gait_msg.nominal_h;
	cout<<"stance duration: "<<gait_config_.stance_duration <<"swing height: "<< gait_config_.swing_height<<endl;
	base_.setGaitConfig(gait_config_);
}
  //grid callback is used for general elevation grid map information (size, pose etc.)
void QuadrupedFootController::gridmapCallback( const grid_map_msgs::GridMap msg){
  if(msg.info.resolution != 0.0){
    grid_map::GridMapRosConverter::fromMessage(msg,  inputGridMap_);
    if(!checkGridMapEmpty(inputGridMap_))
      gridmap_available=true;
    else{
      ROS_INFO("gridmap is empty");
      gridmap_available=false;
    }
  }
}

 //checks if gridmap is empty and has desired layer
bool QuadrupedFootController::checkGridMapEmpty(const grid_map::GridMap mapMsg_){ 
  if( mapMsg_.getSize()(0) ==0.0 || mapMsg_.getSize()(1)==0.0)
    return true;
  else 
    return false;
}
void QuadrupedFootController::cmdPoseCallback_(const nav_msgs::Odometry::ConstPtr& msg)
{
    double roll,pitch,yaw;
    tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    req_pose_.orientation.roll = roll ;
    req_pose_.orientation.pitch = pitch ;
    req_pose_.orientation.yaw = yaw;

    req_pose_.position.z = msg->pose.pose.position.z * gait_config_.nominal_height;
    if(req_pose_.position.z < (gait_config_.nominal_height * 0.5))
        req_pose_.position.z = gait_config_.nominal_height * 0.5;
}

void QuadrupedFootController::publishVelocities_(const ros::TimerEvent& event)
{
    ros::Time current_time = ros::Time::now();

    double vel_dt = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;

    //rotate in the z axis
    //https://en.wikipedia.org/wiki/Rotation_matrix
    double delta_heading = current_velocities_.angular.z * vel_dt; 
    double delta_x = (current_velocities_.linear.x * cos(heading_) - current_velocities_.linear.y * sin(heading_)) * vel_dt; //m
    double delta_y = (current_velocities_.linear.x * sin(heading_) + current_velocities_.linear.y * cos(heading_)) * vel_dt; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //calculate robot's heading_ in quaternion angle
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, heading_);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_footprint_frame_;

    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = req_pose_.position.z;
    //robot's heading_ in quaternion
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    odom.twist.twist.linear.x = current_velocities_.linear.x;
    odom.twist.twist.linear.y = current_velocities_.linear.y;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = current_velocities_.angular.z;

    odom.twist.covariance[0] = 0.001;
    odom.twist.covariance[7] = 0.001;
    odom.twist.covariance[35] = 0.001;

    velocities_publisher_.publish(odom);
}
void QuadrupedFootController::stabilizeCoM_(int ref_id, geometry::Transformation *current_foot_positions_)
{
  int count = 0 ; 
  geometry::Transformation supporting_foot_positions_[3];
  double total_x = 0, total_y = 0, total_z = 0 ; 
  for (int id=0; id<4; id++ ){
  if(round(ref_id) != round (id) ){
    //Triangular poligon, supporting foot  
    supporting_foot_positions_[count].X() =  current_foot_positions_[id].X() ; 
    supporting_foot_positions_[count].Y() =  current_foot_positions_[id].Y() ; 
    supporting_foot_positions_[count].Z() =  current_foot_positions_[id].Z() ; 
    count ++;
  }
  //cout<<"current "<<id<<":"<<current_foot_positions_[id].X()<< " "<<current_foot_positions_[id].Y()<< " " <<current_foot_positions_[id].Z()<<endl;
  //cout<<"target "<<id<<":"<<target_foot_positions_[id].X()<< " "<<target_foot_positions_[id].Y()<< " " <<target_foot_positions_[id].Z()<<endl;
  total_x += current_foot_positions_[id].X();
  total_y += current_foot_positions_[id].Y();
  total_z += current_foot_positions_[id].Z();
  }
  //cout<<"ref id: "<<ref_id<<endl;
  geometry_msgs::Point prev_center;
  //previous average center location Rectengular poligon
  prev_center.x =  total_x/4 ; 
  prev_center.y =  total_y/4 ; 
  prev_center.z =  total_z/4 ; 
  //cout<<"prev center: "<<prev_center.x<< " "<<prev_center.y<<" "<<prev_center.z<<endl; 
  geometry_msgs::Point center;
  center = circleCenter(supporting_foot_positions_[0] , supporting_foot_positions_[1] , supporting_foot_positions_[2]);
  //cout <<"target center: "<<center.x << " " << center.y << " "<<endl;
  double x_com_translate =  prev_center.x -  center.x;
  double y_com_translate =  prev_center.y - center.y;

  //cout<<"x_com_trans: "<< x_com_translate<<" y_com_trans: "<< y_com_translate<<endl;
  gait_config_.com_x_translation =  x_com_translate ;
  gait_config_.com_y_translation =  y_com_translate ;
  base_.setGaitConfig(gait_config_);
}

void QuadrupedFootController::visualizeFootAwareness(geometry::Transformation *foot_pos,geometry::Transformation *orig_foot_pos )
{   
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker foot_marker;
    for (int id = 0; id<4; id++){
      if(counter > 100)
        counter = 0;
      foot_marker.header.frame_id = "base";

      foot_marker.type = visualization_msgs::Marker::CUBE;
      foot_marker.action = visualization_msgs::Marker::ADD;
      foot_marker.id = counter;

      foot_marker.pose.position.x = foot_pos[id].X();
      foot_marker.pose.position.y = foot_pos[id].Y();
      foot_marker.pose.position.z = foot_pos[id].Z();
      
      foot_marker.pose.orientation.x = 0.0;
      foot_marker.pose.orientation.y = 0.0;
      foot_marker.pose.orientation.z = 0.0;
      foot_marker.pose.orientation.w = 1.0;

      foot_marker.scale.x = 0.03;
      foot_marker.scale.y = 0.03;
      foot_marker.scale.z = 0.03;

      foot_marker.color.r = 1;
      foot_marker.color.g = 1;
      foot_marker.color.b = 0;
      foot_marker.color.a = 1;
      markerArray.markers.push_back(foot_marker);    
      counter ++;
      foot_marker.type = visualization_msgs::Marker::SPHERE;
      foot_marker.id = counter;
      foot_marker.pose.position.x = orig_foot_pos[id].X();
      foot_marker.pose.position.y = orig_foot_pos[id].Y();
      foot_marker.pose.position.z = orig_foot_pos[id].Z();
      
      foot_marker.pose.orientation.x = 0.0;
      foot_marker.pose.orientation.y = 0.0;
      foot_marker.pose.orientation.z = 0.0;
      foot_marker.pose.orientation.w = 1.0;

      foot_marker.scale.x = 0.05;
      foot_marker.scale.y = 0.05;
      foot_marker.scale.z = 0.05;

      foot_marker.color.r = 0;
      foot_marker.color.g = 0;
      foot_marker.color.b = 1;
      foot_marker.color.a = 1;
      markerArray.markers.push_back(foot_marker);    
      counter ++;

    }
    markerPublisher_.publish(markerArray);

}

visualization_msgs::Marker QuadrupedFootController::createMarker(geometry::Transformation foot_pos, int id, std::string frame_id)
{
    visualization_msgs::Marker foot_marker;

    foot_marker.header.frame_id = frame_id;

    foot_marker.type = visualization_msgs::Marker::SPHERE;
    foot_marker.action = visualization_msgs::Marker::ADD;
    foot_marker.id = id;

    foot_marker.pose.position.x = foot_pos.X();
    foot_marker.pose.position.y = foot_pos.Y();
    foot_marker.pose.position.z = foot_pos.Z();
    
    foot_marker.pose.orientation.x = 0.0;
    foot_marker.pose.orientation.y = 0.0;
    foot_marker.pose.orientation.z = 0.0;
    foot_marker.pose.orientation.w = 1.0;

    foot_marker.scale.x = 0.05;
    foot_marker.scale.y = 0.05;
    foot_marker.scale.z = 0.05;

    foot_marker.color.r = 0.780;
    foot_marker.color.g = 0.082;
    foot_marker.color.b = 0.521;
    foot_marker.color.a = 1;

    return foot_marker;
}


void QuadrupedFootController::publishFootPositions_(const ros::TimerEvent& event)
{
    visualization_msgs::MarkerArray marker_array;
    float robot_height;

    for(size_t i = 0; i < 4; i++)
    {
        geometry::Transformation temp_foot_pos = target_foot_positions_[i];
        //the target foot position is calculated in the hip frame
        //now transform this to base so we can visualize in rviz
        champ::Kinematics::transformToBase(temp_foot_pos, *base_.legs[i]);
        marker_array.markers.push_back(createMarker(temp_foot_pos, i, base_link_frame_));
        robot_height += current_foot_positions_[i].Z();
    }

	if(foot_publisher_.getNumSubscribers())
    {
        foot_publisher_.publish(marker_array);
    }
}
void QuadrupedFootController::FRhipCallback(const aliengo_msgs::MotorState& msg)
{
    start_up = false;
    lowState.motorState[0].mode = msg.mode;
    lowState.motorState[0].position = msg.position;
    lowState.motorState[0].velocity = msg.velocity;
    lowState.motorState[0].torque = msg.torque;
}

void QuadrupedFootController::FRthighCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[1].mode = msg.mode;
    lowState.motorState[1].position = msg.position;
    lowState.motorState[1].velocity = msg.velocity;
    lowState.motorState[1].torque = msg.torque;

}

void QuadrupedFootController::FRcalfCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[2].mode = msg.mode;
    lowState.motorState[2].position = msg.position;
    lowState.motorState[2].velocity = msg.velocity;
    lowState.motorState[2].torque = msg.torque;

}

void QuadrupedFootController::FLhipCallback(const aliengo_msgs::MotorState& msg)
{
    start_up = false;
    lowState.motorState[3].mode = msg.mode;
    lowState.motorState[3].position = msg.position;
    lowState.motorState[3].velocity = msg.velocity;
    lowState.motorState[3].torque = msg.torque;
}

void QuadrupedFootController::FLthighCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[4].mode = msg.mode;
    lowState.motorState[4].position = msg.position;
    lowState.motorState[4].velocity = msg.velocity;
    lowState.motorState[4].torque = msg.torque;
}

void QuadrupedFootController::FLcalfCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[5].mode = msg.mode;
    lowState.motorState[5].position = msg.position;
    lowState.motorState[5].velocity = msg.velocity;
    lowState.motorState[5].torque = msg.torque;
}

void QuadrupedFootController::RRhipCallback(const aliengo_msgs::MotorState& msg)
{
    start_up = false;
    lowState.motorState[6].mode = msg.mode;
    lowState.motorState[6].position = msg.position;
    lowState.motorState[6].velocity = msg.velocity;
    lowState.motorState[6].torque = msg.torque;
}

void QuadrupedFootController::RRthighCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[7].mode = msg.mode;
    lowState.motorState[7].position = msg.position;
    lowState.motorState[7].velocity = msg.velocity;
    lowState.motorState[7].torque = msg.torque;
}

void QuadrupedFootController::RRcalfCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[8].mode = msg.mode;
    lowState.motorState[8].position = msg.position;
    lowState.motorState[8].velocity = msg.velocity;
    lowState.motorState[8].torque = msg.torque;

}

void QuadrupedFootController::RLhipCallback(const aliengo_msgs::MotorState& msg)
{
    start_up = false;
    lowState.motorState[9].mode = msg.mode;
    lowState.motorState[9].position = msg.position;
    lowState.motorState[9].velocity = msg.velocity;
    lowState.motorState[9].torque = msg.torque;
}

void QuadrupedFootController::RLthighCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[10].mode = msg.mode;
    lowState.motorState[10].position = msg.position;
    lowState.motorState[10].velocity = msg.velocity;
    lowState.motorState[10].torque = msg.torque;
}

void QuadrupedFootController::RLcalfCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[11].mode = msg.mode;
    lowState.motorState[11].position = msg.position;
    lowState.motorState[11].velocity = msg.velocity;
    lowState.motorState[11].torque = msg.torque;
}

void QuadrupedFootController::sendServoCmd()
{
    for(int m=0; m<12; m++){
        servo_pub[m].publish(lowCmd.motorCmd[m]);
    }
}

void QuadrupedFootController::moveAllPosition(float* targetPos, double duration)
{
    float pos[12] ,lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].position;
    for(int i=1; i<=duration; i++){
        if(!ros::ok()) break;
        percent = (double)i/duration;
        for(int j=0; j<12; j++){
            lowCmd.motorCmd[j].position = lastPos[j]*(1-percent) + targetPos[j]*percent; 
        }
        sendServoCmd();
    }
}
geometry_msgs::Point QuadrupedFootController::circleCenter(geometry::Transformation A, geometry::Transformation B, geometry::Transformation C) {
    geometry_msgs::Point center;
    center.x =(A.X() + B.X()+ C.X())/3 ;
    center.y =(A.Y() + B.Y()+ C.Y())/3 ;
    return center;
  }