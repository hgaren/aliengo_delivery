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

#ifndef QUADRUPED_FOOT_CONTROLLER_H
#define QUADRUPED_FOOT_CONTROLLER_H

#include "ros/ros.h"

#include <urdf_loader.h>
//#include <champ_msgs/Joints.h>
//#include <champ_msgs/Pose.h>
//#include <champ_msgs/PointArray.h>

#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_components.h>
#include <body_controller/body_controller.h>
#include <leg_controller/leg_controller.h>
#include <kinematics/kinematics.h>
#include <odometry/odometry.h>
#include <actuator.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <boost/thread.hpp>

#include "aliengo_msgs/LowCmd.h"
#include "aliengo_msgs/LowState.h"
#include "aliengo_msgs/MotorCmd.h"
#include "aliengo_msgs/MotorState.h"
#include "aliengo_msgs/Foots.h"

class QuadrupedFootController
{
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber cmd_vel_subscriber_;
    ros::Subscriber cmd_pose_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber foot_subscriber_;

    ros::Publisher velocities_publisher_;
    ros::Publisher foot_publisher_;


    ros::Subscriber servo_sub[12];
    ros::Publisher servo_pub[12];

    ros::Timer loop_timer_;
    ros::Timer odom_data_timer_;
    ros::Timer foot_position_timer_;

    champ::Velocities req_vel_;
    champ::Pose req_pose_;

    champ::Velocities current_velocities_;
    float current_joint_positions_[12];
    geometry::Transformation current_foot_positions_[4];
    geometry::Transformation target_foot_positions_[4];

    float x_pos_;
    float y_pos_;
    float heading_;
    ros::Time last_vel_time_;

    champ::GaitConfig gait_config_;

    champ::QuadrupedBase base_;
    champ::BodyController body_controller_;
    champ::LegController leg_controller_;
    champ::Kinematics kinematics_;
    champ::Odometry odometry_;
    champ::Actuator actuators_;

    std::vector<std::string> joint_names_;
    std::string base_name_;
    std::string node_namespace_;
    std::string odom_frame_;
    std::string base_footprint_frame_;
    std::string base_link_frame_;

    bool in_gazebo_;
    bool start_up = true;
    bool foot_ref_available = false;
    bool walking_available = false , linear_config = false,angular_config = false, hybrid_config = false;

    void controlLoop_(const ros::TimerEvent& event);
    void publishJoints_(const ros::TimerEvent& event);
    void publishVelocities_(const ros::TimerEvent& event);
    void publishFootPositions_(const ros::TimerEvent& event);

    void cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg);
    void cmdPoseCallback_(const nav_msgs::Odometry::ConstPtr& msg);

    void FRhipCallback(const aliengo_msgs::MotorState& msg);
    void FRthighCallback(const aliengo_msgs::MotorState& msg);
    void FRcalfCallback(const aliengo_msgs::MotorState& msg);
    void FLhipCallback(const aliengo_msgs::MotorState& msg);
    void FLthighCallback(const aliengo_msgs::MotorState& msg);
    void FLcalfCallback(const aliengo_msgs::MotorState& msg);
    void RRhipCallback(const aliengo_msgs::MotorState& msg);
    void RRthighCallback(const aliengo_msgs::MotorState& msg);
    void RRcalfCallback(const aliengo_msgs::MotorState& msg);
    void RLhipCallback(const aliengo_msgs::MotorState& msg);
    void RLthighCallback(const aliengo_msgs::MotorState& msg);
    void RLcalfCallback(const aliengo_msgs::MotorState& msg);
    void sendServoCmd();
    void moveAllPosition(float* targetPos, double duration);
    void odomCallback_(const nav_msgs::Odometry::ConstPtr& msg);
    void footCallback_(const aliengo_msgs::Foots& foot_msg);
    geometry_msgs::Point circleCenter(geometry::Transformation A, geometry::Transformation B, geometry::Transformation C);

    aliengo_msgs::LowCmd lowCmd;
    aliengo_msgs::LowState lowState;
    aliengo_msgs::Foots foots;

    visualization_msgs::Marker createMarker(geometry::Transformation foot_pos, int id, std::string frame_id);

    public:
        QuadrupedFootController(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
};

#endif