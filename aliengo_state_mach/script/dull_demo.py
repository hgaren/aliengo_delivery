#!/usr/bin/env python
## This is the demo code for state_machine.
import rospy
from std_msgs.msg import String
from status_handler import status_handler
from aliengo_state_mach.msg import RoverActionMsg,RoverStateMsg
 
if __name__ == '__main__':
  rospy.init_node('dull_demo')
  pub = rospy.Publisher('/smach/input', RoverActionMsg, queue_size =1)
  action = RoverActionMsg()
  rate = rospy.Rate(1)
  while not rospy.is_shutdown():

    userInput = raw_input()
    if userInput == '1':  
    	action.ACTION = action.FIRST_GOAL_STARTED
    	pub.publish(action)   
    if userInput == "3":  
    	action.ACTION = action.FIRST_GOAL_FINISHED
    	pub.publish(action)   
    if userInput == "5":  
    	action.ACTION = action.DOCKING_FINISHED
    	pub.publish(action)   
    if userInput == "19":  
    	action.ACTION = action.MOVING_BOX_STARTED
    	pub.publish(action)   
    if userInput == "7":  
    	action.ACTION = action.MOVING_BOX_ENDED
    	pub.publish(action)   
    if userInput == "21":  
    	action.ACTION = action.BOX_IS_ON_THE_ROBOT
    	pub.publish(action)   
    if userInput == "9":  
    	action.ACTION = action.SECOND_GOAL_STARTED
    	pub.publish(action)   
    if userInput == "11":  
    	action.ACTION = action.SECOND_GOAL_FINISHED
    	pub.publish(action)   
    if userInput == '0':  
    	action.ACTION = action.ROBOT_FALLS
    	pub.publish(action)   
    if userInput == '2':  
    	action.ACTION = action.BOX_FALLS
    	pub.publish(action)   

    rate.sleep()
