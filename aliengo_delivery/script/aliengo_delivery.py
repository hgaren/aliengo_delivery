#!/usr/bin/env python

import rospy
import actionlib
from aliengo_move import move_handler    
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from aliengo_state_mach.msg import RoverStateMsg,RoverActionMsg
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import DeleteModelRequest, SpawnModelRequest,SpawnModel,DeleteModel
import roslib
import os
import time
stateMsg = RoverStateMsg()
state_able = False
reset_world_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
reset_simulation_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
aliengo_down = True
box_dropped = True
def state_callback(msg):
  global  stateMsg 
  global state_able
  stateMsg = msg 
  state_able = True

    
def callbackodom_aliengo(data):
  orientation_q = data.pose.pose.orientation
  orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
  (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
  global aliengo_down

  if( abs(roll)> 0.5 and aliengo_down):
    print("Aliengo is Down !")
    spawn_robot()
    aliengo_down = False
    time.sleep(1)
    actionMsg.ACTION = actionMsg.ROBOT_FALLS
    action_pub.publish(actionMsg)
    time.sleep(1)

def callbackodom_box(data):
  global box_dropped
  if(data.pose.pose.position.z<0.3 and box_dropped):
    print("Box is Down !")
    spawn_box()
    box_dropped = False
    time.sleep(1)
    actionMsg.ACTION = actionMsg.BOX_FALLS
    action_pub.publish(actionMsg)
    time.sleep(1)


def spawn_robot():
  rospy.wait_for_service("/gazebo/spawn_urdf_model")
  file_localition = roslib.packages.get_pkg_dir('aliengo_description') + '/xacro/robot.xacro'
  p = os.popen("rosrun xacro xacro " + file_localition)
  xml_string = p.read()
  p.close()
  srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
  srv_delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

  req = DeleteModelRequest()
  name = "laikago_gazebo"
  req.model_name = name
  try:
    res = srv_delete_model(name)
  except rospy.ServiceException, e:
    rospy.logdebug("Model %s does not exist in gazebo.", name)

  object_pose = Pose()
  object_pose.position.x = float(-4.418915)
  object_pose.position.y = float(3.5)
  object_pose.position.z = float(1.0)
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

def spawn_box():
 # pauseSim()
  rospy.wait_for_service("/gazebo/spawn_urdf_model")
  file_localition = roslib.packages.get_pkg_dir('aliengo_gazebo') + '/launch/world/my_box.urdf'
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


if __name__ == '__main__':
  rospy.init_node('aliengo_delivery')
  rospy.Subscriber("/smach/output",RoverStateMsg,state_callback)
  rospy.Subscriber("/aliengo/odometry", Odometry,  callbackodom_aliengo)    
  rospy.Subscriber("/box/odometry", Odometry, callbackodom_box)    
  action_pub = rospy.Publisher("/smach/input", RoverActionMsg, queue_size=1)
  actionMsg = RoverActionMsg()
  move_handler = move_handler()

  move_handler.start()
  move_handler.start_timing()

  rate = rospy.Rate(10)
  do_once = True
  while not rospy.is_shutdown():
    if(state_able):
      print(stateMsg.state)
      if(do_once != True and stateMsg.state == stateMsg.IDLE_IDLE_1):
        print("starting first goal, press & enter s ")
        user_input = raw_input()
        if(user_input == 's'):
          actionMsg.ACTION = actionMsg.FIRST_GOAL_STARTED
          action_pub.publish(actionMsg)  

      if(stateMsg.state == stateMsg.WALKING_IDLE_1):
        x = [-5.364326,-5.151546]
        y = [5.051377, 8.126664]
        yaw = [1.555206, 1.582429]
        if(move_handler.movebase_client(x,y,yaw)):
          actionMsg.ACTION = actionMsg.FIRST_GOAL_FINISHED
          action_pub.publish(actionMsg)
          move_handler.start_timing()

       
      if(stateMsg.state == stateMsg.DOCKING_IDLE):
        print(move_handler.robot_docking())
        if(move_handler.robot_docking()):
          actionMsg.ACTION = actionMsg.DOCKING_FINISHED
          action_pub.publish(actionMsg)
          move_handler.start_timing()

      if(stateMsg.state == stateMsg.IDLE_IDLE_3):
        move_handler.start_timing()

      if(stateMsg.state == stateMsg.STANDING_IDLE):
        print(move_handler.robot_standing())
        if(move_handler.robot_standing()):
          actionMsg.ACTION = actionMsg.SECOND_GOAL_STARTED
          action_pub.publish(actionMsg)
          move_handler.start_timing()

      if(stateMsg.state == stateMsg.WALKING_IDLE_2):
        x = [-2.148517,2.344316,2.611152]
        y = [-0.216613,-0.617723,5.532104]
        yaw = [-0.032022,1.456535,1.456535]
        '''
        x = [2.344316,2.611152]
        y = [-0.617723,5.532104]
        yaw = [1.456535,1.456535]
        '''
        if(move_handler.movebase_client(x,y,yaw)):
          actionMsg.ACTION = actionMsg.SECOND_GOAL_FINISHED
          action_pub.publish(actionMsg)
          move_handler.start_timing()

      state_able = False
      do_once =False

    rate.sleep()
