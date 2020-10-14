#!/usr/bin/env python

import rospy
import actionlib
from aliengo_move import move_handler    
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler  
from aliengo_state_mach.msg import RoverStateMsg,RoverActionMsg
 
stateMsg = RoverStateMsg()
state_able = False

def state_callback(msg):
  global  stateMsg 
  global state_able
  stateMsg = msg 
  state_able = True

if __name__ == '__main__':
  rospy.init_node('aliengo_delivery')
  rospy.Subscriber("/smach/output",RoverStateMsg,state_callback)
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
        yaw = [1.582429, 1.582429]
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
        x = [-1.060470]
        y = [-0.478616]
        yaw = [-0.038218]
        if(move_handler.movebase_client(x,y,yaw)):
          actionMsg.ACTION = actionMsg.SECOND_GOAL_FINISHED
          action_pub.publish(actionMsg)
          move_handler.start_timing()

      state_able = False
      do_once =False

    rate.sleep()
