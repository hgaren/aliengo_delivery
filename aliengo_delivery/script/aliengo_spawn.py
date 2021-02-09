#!/usr/bin/env python

import rospy
import actionlib
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import DeleteModelRequest, SpawnModelRequest,SpawnModel,DeleteModel
import roslib
import os
import time
from tf.transformations import quaternion_from_euler,euler_from_quaternion

unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
box_dropped = True



def callbackodom_aliengo(data):
  orientation_q = data.pose.pose.orientation
  orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
  (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
  if( abs(roll)> 0.5):
    print("Aliengo is Down !")

def callbackodom_box(data):
  print(data.pose.pose.position.z)
  global box_dropped
  if(data.pose.pose.position.z<0.3 and box_dropped):
    print("Box is Down !")
    spawn_box()
    box_dropped = False
    time.sleep(1)

def spawn_box():
 # pauseSim()
  rospy.wait_for_service("/gazebo/spawn_urdf_model")
  file_localition = roslib.packages.get_pkg_dir('aliengo_gazebo') + '/world/my_box.urdf'
  p = os.popen("rosrun xacro xacro " + file_localition)
  xml_string = p.read()
  p.close()
  srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
  srv_delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

  req = DeleteModelRequest()
  name = "box"
  req.model_name = name
  try:
    res = srv_delete_model(name)
  except rospy.ServiceException, e:
    rospy.logdebug("Model %s does not exist in gazebo.", name)

  object_pose = Pose()
  object_pose.position.x = float(-2.283477)
  object_pose.position.y = float(8.013514)
  object_pose.position.z = float(0.6)
  object_pose.orientation.x = float(0.0)
  object_pose.orientation.y = float(0.0)
  object_pose.orientation.z = float(0.0)
  object_pose.orientation.w = float(1.0)
  # spawn new model
  req = SpawnModelRequest()
  req.model_name = name # model name from command line input
  req.model_xml = xml_string
  req.initial_pose = object_pose

  res = srv_spawn_model(req)
 # unpauseSim()


def pauseSim():
    rospy.logdebug("PAUSING service found...")
    paused_done = False
    counter = 0
    while not paused_done and not rospy.is_shutdown():
        if counter < 10:
            try:
                rospy.logdebug("PAUSING service calling...")
                pause()
                paused_done = True
                rospy.logdebug("PAUSING service calling...DONE")
            except rospy.ServiceException as e:
                counter += 1
                rospy.logerr("/gazebo/pause_physics service call failed")

    rospy.logdebug("PAUSING FINISH")

def unpauseSim():
    rospy.logdebug("UNPAUSING service found...")
    unpaused_done = False
    counter = 0
    while not unpaused_done and not rospy.is_shutdown():
        if counter < 10:
            try:
                rospy.logdebug("UNPAUSING service calling...")
                unpause()
                unpaused_done = True
                rospy.logdebug("UNPAUSING service calling...DONE")
            except rospy.ServiceException as e:
                counter += 1
                rospy.logerr("/gazebo/unpause_physics service call failed...Retrying "+str(counter))

    rospy.logdebug("UNPAUSING FiNISH")

if __name__ == '__main__':
  rospy.init_node('aliengo_spawner')


  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
	rospy.Subscriber("/aliengo/odometry", Odometry,  callbackodom_aliengo)
	rospy.Subscriber("/box/odometry", Odometry, callbackodom_box)
	rospy.spin()
	rate.sleep()
