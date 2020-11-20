#!/usr/bin/env python
#Gazebo state to tf and odometry 
#hgaren 08.10.2019

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import tf_conversions
import tf2_ros
import geometry_msgs.msg
index=0
index_2=0
def posecallback(data):
  odom =Odometry()
  odom_box = Odometry()
  global index
  global index_2
  index=0
  index_2=0
  for k in range(0,len(data.name)):
    if(data.name[k]=="laikago_gazebo::base"):
      index=k
    if(data.name[k]=="box::my_box"):
      index_2=k
  if(len(data.pose)>1):
    odom.pose.pose=data.pose[index]
    odom.twist.twist=data.twist[index]
    odom.header.stamp=rospy.Time.now()
    odom.header.frame_id="odom"
    odom.child_frame_id="base"

    odom_box.pose.pose=data.pose[index_2]
    odom_box.twist.twist=data.twist[index_2]
    odom_box.header.stamp=rospy.Time.now()
    odom_box.header.frame_id="odom"
    odom_box.child_frame_id="base"

    pub = rospy.Publisher('/aliengo/odometry', Odometry, queue_size=10)
    pub2 = rospy.Publisher('/box/odometry', Odometry, queue_size=10)

    pub.publish(odom)
    pub2.publish(odom_box)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base"
    t.transform.translation.x = odom.pose.pose.position.x
    t.transform.translation.y = odom.pose.pose.position.y
    t.transform.translation.z = odom.pose.pose.position.z

    t.transform.rotation.x = odom.pose.pose.orientation.x
    t.transform.rotation.y = odom.pose.pose.orientation.y
    t.transform.rotation.z = odom.pose.pose.orientation.z
    t.transform.rotation.w = odom.pose.pose.orientation.w

    br.sendTransform(t)

  #print(odom)
  
    
def main():
    rospy.init_node('sim_pose_converter', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", ModelStates, posecallback)
    rate = rospy.Rate(10) # 10hz
    rate.sleep();
    rospy.spin()


if __name__ == '__main__':
    main()
