#!/usr/bin/env python


### THIS IS THE STATE MACHINE FOR ITU ROVER


import rospy
import smach
import smach_ros
from std_msgs.msg import String
from status_handler import status_handler    
from aliengo_state_mach.msg import RoverStateMsg,RoverActionMsg
 
status_handler = status_handler()
actionMsg = RoverActionMsg()

 
        
class IDLE_IDLE_1(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['first_goal_started_1','repeat'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.IDLE_IDLE_1      
    status_handler.publishRoverState(self.stateMsg)
    if(status_handler.action.ACTION  ==  actionMsg.FIRST_GOAL_STARTED):      
      return 'first_goal_started_1'
    return 'repeat'

class WALKING_IDLE_1(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['first_goal_finished_3','robot_falls_0','repeat'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.WALKING_IDLE_1      
    status_handler.publishRoverState(self.stateMsg)  
    if(status_handler.action.ACTION ==  actionMsg.FIRST_GOAL_FINISHED):    
      return 'first_goal_finished_3'
    if (status_handler.action.ACTION ==  actionMsg.ROBOT_FALLS):    
      return 'robot_falls_0'
    return 'repeat'

class DOCKING_IDLE(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['docking_finished_5','robot_falls_0','repeat'])
    self.stateMsg = RoverStateMsg()
  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.DOCKING_IDLE      
    status_handler.publishRoverState(self.stateMsg)  
    if(status_handler.action.ACTION ==  actionMsg.DOCKING_FINISHED):    
      return 'docking_finished_5'
    if (status_handler.action.ACTION ==  actionMsg.ROBOT_FALLS):    
      return 'robot_falls_0'
    return 'repeat'

class IDLE_IDLE_2(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['moving_box_started_19','repeat'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.IDLE_IDLE_2      
    status_handler.publishRoverState(self.stateMsg)
    if(status_handler.action.ACTION ==  actionMsg.MOVING_BOX_STARTED):      
      return 'moving_box_started_19'
    return 'repeat'

class IDLE_WORKING(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['moving_box_ended_7','box_falls_2','repeat'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.IDLE_WORKING      
    status_handler.publishRoverState(self.stateMsg)  
    if(status_handler.action.ACTION ==  actionMsg.MOVING_BOX_ENDED):    
      return 'moving_box_ended_7'
    if (status_handler.action.ACTION ==  actionMsg.BOX_FALLS):    
      return 'box_falls_2'
    return 'repeat'

class IDLE_IDLE_3(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['box_is_on_the_robot_21','repeat'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.IDLE_IDLE_3      
    status_handler.publishRoverState(self.stateMsg)
    if(status_handler.action.ACTION ==  actionMsg.BOX_IS_ON_THE_ROBOT):      
      return 'box_is_on_the_robot_21'
    return 'repeat'

class STANDING_IDLE(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['second_goal_started_9','robot_falls_4','repeat'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.STANDING_IDLE      
    status_handler.publishRoverState(self.stateMsg)  
    if(status_handler.action.ACTION ==  actionMsg.SECOND_GOAL_STARTED):    
      return 'second_goal_started_9'
    if (status_handler.action.ACTION  ==  actionMsg.ROBOT_FALLS):    
      return 'robot_falls_4'
    return 'repeat'

class WALKING_IDLE_2(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['second_goal_finished_11','robot_falls_4','repeat'])
    self.actionMsg = status_handler.action
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.WALKING_IDLE_2      
    status_handler.publishRoverState(self.stateMsg)  
    if(status_handler.action.ACTION  ==  self.actionMsg.SECOND_GOAL_FINISHED):    
      return 'second_goal_finished_11'
    if (status_handler.action.ACTION  ==  self.actionMsg.ROBOT_FALLS):    
      return 'robot_falls_4'
    return 'repeat'

class ROBOT_FAIL_1(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['error_flag_17'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.ROBOT_FAIL_1      
    status_handler.publishRoverState(self.stateMsg)
    return 'error_flag_17'

class ROBOT_FAIL_2(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['error_flag_23'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.ROBOT_FAIL_2      
    status_handler.publishRoverState(self.stateMsg)
    return 'error_flag_23'

class BOX_FAIL(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['spawn_box_15'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.BOX_FAIL      
    status_handler.publishRoverState(self.stateMsg)
    return 'spawn_box_15'
class SUCCES_IDLE(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['succes_flag_13'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.BOX_FAIL      
    status_handler.publishRoverState(self.stateMsg)
    return 'succes_flag_13'
#####################################################################################################################################################
        



def CreateStateMachine():

    #Create the state machine
    sm_rover = smach.StateMachine(outcomes=['DEAD'])
    #Open the container
    with sm_rover:

        smach.StateMachine.add('IDLE_IDLE_1', IDLE_IDLE_1(),
                               transitions={'first_goal_started_1': 'WALKING_IDLE_1','repeat':'IDLE_IDLE_1', })        
        smach.StateMachine.add('ROBOT_FAIL_1', ROBOT_FAIL_1(), 
                               transitions={'error_flag_17':'IDLE_IDLE_1' })

        smach.StateMachine.add('ROBOT_FAIL_2', ROBOT_FAIL_2(), 
                               transitions={'error_flag_23':'IDLE_IDLE_1' })
        smach.StateMachine.add('BOX_FAIL', BOX_FAIL(), 
                               transitions={'spawn_box_15':'IDLE_IDLE_1' })

        smach.StateMachine.add('WALKING_IDLE_1', WALKING_IDLE_1(), 
                               transitions={'first_goal_finished_3':'DOCKING_IDLE', 'robot_falls_0':'ROBOT_FAIL_1','repeat':'WALKING_IDLE_1'})

        smach.StateMachine.add('DOCKING_IDLE', DOCKING_IDLE(), 
                               transitions={'docking_finished_5':'IDLE_IDLE_2', 'robot_falls_0':'ROBOT_FAIL_1','repeat':'DOCKING_IDLE'})     

        smach.StateMachine.add('IDLE_IDLE_2', IDLE_IDLE_2(), 
                               transitions={'moving_box_started_19':'IDLE_WORKING','repeat':'IDLE_IDLE_2'})     

        smach.StateMachine.add('IDLE_WORKING', IDLE_WORKING(), 
                               transitions={'moving_box_ended_7':'IDLE_IDLE_3', 'box_falls_2':'BOX_FAIL','repeat':'IDLE_WORKING'})     

        smach.StateMachine.add('IDLE_IDLE_3', IDLE_IDLE_3(), 
                               transitions={'box_is_on_the_robot_21':'STANDING_IDLE','repeat':'IDLE_IDLE_3'})     

        smach.StateMachine.add('STANDING_IDLE', STANDING_IDLE(), 
                               transitions={'second_goal_started_9':'WALKING_IDLE_2', 'robot_falls_4':'ROBOT_FAIL_2','repeat':'STANDING_IDLE'})     

        smach.StateMachine.add('WALKING_IDLE_2', WALKING_IDLE_2(), 
                               transitions={'second_goal_finished_11':'SUCCES_IDLE', 'robot_falls_4':'ROBOT_FAIL_2','repeat':'WALKING_IDLE_2'})     

        smach.StateMachine.add('SUCCES_IDLE', SUCCES_IDLE(), 
                               transitions={'succes_flag_13':'IDLE_IDLE_1'
                               })
        #Codes for smach viewer
        sis = smach_ros.IntrospectionServer('rover_state_machine', sm_rover, '/ROVER_SM_ROOT')
        sis.start()

        outcome = sm_rover.execute()
        sis.stop()

def main():
    #Init,pubs and subs
    rospy.init_node("state_machine")
    global status_handler
    status_handler.start()
    CreateStateMachine()


if __name__ == '__main__':
    main()
