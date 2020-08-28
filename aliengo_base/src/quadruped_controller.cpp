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

#include <quadruped_controller.h>
#include <tf/tf.h>
#include "aliengo_msgs/LowCmd.h"
#include "aliengo_msgs/LowState.h"
#include "aliengo_msgs/MotorCmd.h"
#include "aliengo_msgs/MotorState.h"
  
QuadrupedController::QuadrupedController(const ros::NodeHandle &node_handle,
                                         const ros::NodeHandle &private_node_handle):
    nh_(node_handle),
    pnh_(private_node_handle),
    body_controller_(base_),
    leg_controller_(base_),
    kinematics_(base_),
    odometry_(base_)
{
    //initializations

	servo_sub[0] = nh_.subscribe("/laikago_gazebo/FR_hip_controller/state", 1, &QuadrupedController::FRhipCallback, this);
	servo_sub[1] = nh_.subscribe("/laikago_gazebo/FR_thigh_controller/state", 1, &QuadrupedController::FRthighCallback, this);
	servo_sub[2] = nh_.subscribe("/laikago_gazebo/FR_calf_controller/state", 1, &QuadrupedController::FRcalfCallback, this);
	servo_sub[3] = nh_.subscribe("/laikago_gazebo/FL_hip_controller/state", 1, &QuadrupedController::FLhipCallback, this);
	servo_sub[4] = nh_.subscribe("/laikago_gazebo/FL_thigh_controller/state", 1, &QuadrupedController::FLthighCallback, this);
	servo_sub[5] = nh_.subscribe("/laikago_gazebo/FL_calf_controller/state", 1, &QuadrupedController::FLcalfCallback, this);
	servo_sub[6] = nh_.subscribe("/laikago_gazebo/RR_hip_controller/state", 1, &QuadrupedController::RRhipCallback, this);
	servo_sub[7] = nh_.subscribe("/laikago_gazebo/RR_thigh_controller/state", 1, &QuadrupedController::RRthighCallback, this);
	servo_sub[8] = nh_.subscribe("/laikago_gazebo/RR_calf_controller/state", 1, &QuadrupedController::RRcalfCallback, this);
	servo_sub[9] = nh_.subscribe("/laikago_gazebo/RL_hip_controller/state", 1, &QuadrupedController::RLhipCallback, this);
	servo_sub[10] = nh_.subscribe("/laikago_gazebo/RL_thigh_controller/state", 1, &QuadrupedController::RLthighCallback, this);
	servo_sub[11] = nh_.subscribe("/laikago_gazebo/RL_calf_controller/state", 1, &QuadrupedController::RLcalfCallback, this);
    

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
   

    cmd_vel_subscriber_ = nh_.subscribe("cmd_vel/smooth", 1, &QuadrupedController::cmdVelCallback_, this);
    cmd_pose_subscriber_ = nh_.subscribe("/aliengo/ref_odom", 1, &QuadrupedController::cmdPoseCallback_, this);

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

    loop_timer_ = pnh_.createTimer(ros::Duration(0.005),
                                   &QuadrupedController::controlLoop_,
                                   this);

    odom_data_timer_ = pnh_.createTimer(ros::Duration(0.005),
                                        &QuadrupedController::publishVelocities_, 
                                        this);

    foot_position_timer_ = pnh_.createTimer(ros::Duration(0.005),
                                            &QuadrupedController::publishFootPositions_, 
                                            this);

    req_pose_.position.z = gait_config_.nominal_height;
}

void QuadrupedController::controlLoop_(const ros::TimerEvent& event)
{
    float target_joint_positions[12];
    //calculates joint positions according to ref orientation and ref velocity
    body_controller_.poseCommand(target_foot_positions_, req_pose_);
    leg_controller_.velocityCommand(target_foot_positions_, req_vel_);
    kinematics_.inverse(target_joint_positions, target_foot_positions_);
    //registers current joint positions (feedback coming from joints)
   	for(int i=0; i<12; i++)
    	current_joint_positions_[i]=lowState.motorState[i].position;
    //sends to joint controller for actuating through joint positions 
    moveAllPosition(target_joint_positions, 1);
    //updates base and odom accortding to feedback
    base_.updateJointPositions(current_joint_positions_);
    base_.getFootPositions(current_foot_positions_);
    odometry_.getVelocities(current_velocities_);
}

void QuadrupedController::cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg)
{
    req_vel_.linear.x = msg->linear.x;
    req_vel_.linear.y = msg->linear.y;
    req_vel_.angular.z = msg->angular.z;
}
 

void QuadrupedController::cmdPoseCallback_(const nav_msgs::Odometry::ConstPtr& msg)
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

void QuadrupedController::publishVelocities_(const ros::TimerEvent& event)
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

visualization_msgs::Marker QuadrupedController::createMarker(geometry::Transformation foot_pos, int id, std::string frame_id)
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

void QuadrupedController::publishFootPositions_(const ros::TimerEvent& event)
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
void QuadrupedController::FRhipCallback(const aliengo_msgs::MotorState& msg)
{
    start_up = false;
    lowState.motorState[0].mode = msg.mode;
    lowState.motorState[0].position = msg.position;
    lowState.motorState[0].velocity = msg.velocity;
    lowState.motorState[0].torque = msg.torque;
}

void QuadrupedController::FRthighCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[1].mode = msg.mode;
    lowState.motorState[1].position = msg.position;
    lowState.motorState[1].velocity = msg.velocity;
    lowState.motorState[1].torque = msg.torque;

}

void QuadrupedController::FRcalfCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[2].mode = msg.mode;
    lowState.motorState[2].position = msg.position;
    lowState.motorState[2].velocity = msg.velocity;
    lowState.motorState[2].torque = msg.torque;

}

void QuadrupedController::FLhipCallback(const aliengo_msgs::MotorState& msg)
{
    start_up = false;
    lowState.motorState[3].mode = msg.mode;
    lowState.motorState[3].position = msg.position;
    lowState.motorState[3].velocity = msg.velocity;
    lowState.motorState[3].torque = msg.torque;
}

void QuadrupedController::FLthighCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[4].mode = msg.mode;
    lowState.motorState[4].position = msg.position;
    lowState.motorState[4].velocity = msg.velocity;
    lowState.motorState[4].torque = msg.torque;
}

void QuadrupedController::FLcalfCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[5].mode = msg.mode;
    lowState.motorState[5].position = msg.position;
    lowState.motorState[5].velocity = msg.velocity;
    lowState.motorState[5].torque = msg.torque;
}

void QuadrupedController::RRhipCallback(const aliengo_msgs::MotorState& msg)
{
    start_up = false;
    lowState.motorState[6].mode = msg.mode;
    lowState.motorState[6].position = msg.position;
    lowState.motorState[6].velocity = msg.velocity;
    lowState.motorState[6].torque = msg.torque;
}

void QuadrupedController::RRthighCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[7].mode = msg.mode;
    lowState.motorState[7].position = msg.position;
    lowState.motorState[7].velocity = msg.velocity;
    lowState.motorState[7].torque = msg.torque;
}

void QuadrupedController::RRcalfCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[8].mode = msg.mode;
    lowState.motorState[8].position = msg.position;
    lowState.motorState[8].velocity = msg.velocity;
    lowState.motorState[8].torque = msg.torque;

}

void QuadrupedController::RLhipCallback(const aliengo_msgs::MotorState& msg)
{
    start_up = false;
    lowState.motorState[9].mode = msg.mode;
    lowState.motorState[9].position = msg.position;
    lowState.motorState[9].velocity = msg.velocity;
    lowState.motorState[9].torque = msg.torque;
}

void QuadrupedController::RLthighCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[10].mode = msg.mode;
    lowState.motorState[10].position = msg.position;
    lowState.motorState[10].velocity = msg.velocity;
    lowState.motorState[10].torque = msg.torque;
}

void QuadrupedController::RLcalfCallback(const aliengo_msgs::MotorState& msg)
{
    lowState.motorState[11].mode = msg.mode;
    lowState.motorState[11].position = msg.position;
    lowState.motorState[11].velocity = msg.velocity;
    lowState.motorState[11].torque = msg.torque;
}

void QuadrupedController::sendServoCmd()
{
    for(int m=0; m<12; m++){
        servo_pub[m].publish(lowCmd.motorCmd[m]);
    }
}

void QuadrupedController::moveAllPosition(float* targetPos, double duration)
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
