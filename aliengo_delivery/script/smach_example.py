#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

# define state Foo
class Idle_conveyor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moving_box_starts'])

    def execute(self, userdata):
        rospy.loginfo('Executing state IDLE')
        return 'moving_box_starts'
   
       


class Working(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moving_box_ends', 'box_dropped'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Working')
        if self.counter < 2:
            self.counter += 1
            return 'moving_box_ends'
        else:
            return 'box_dropped'


class Breakdown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['respawned','error_flag'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Initialize')
        if self.counter < 2:
            self.counter += 1
            return 'respawned'
        else:
            return 'error_flag'

class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Initialize')
        return 'start'

class Idle_robot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['first_goal_starts','moving_box_ends'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Initialize')
        if self.counter < 2:
            self.counter += 1
            return 'first_goal_starts'
        else:
            return 'moving_box_ends' 
class Moving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['first_goal_reached','second_goal_reached'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Initialize')
        if self.counter < 2:
            self.counter += 1
            return 'first_goal_reached'
        else:
            return 'second_goal_reached' 
class Docking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['docking_ends'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Initialize')
        return 'first_goal_reached'
class Stand_up(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['second_goal_starts'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Initialize')
        return 'second_goal_starts'
            
def main():
    rospy.init_node('smach_example_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome5','outcome6','outcome7','outcome8' ])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('Initialize', Initialize(),
                               transitions={'start':'Conveyor_Belt'})

        # Create the sub SMACH state machine
        sm_conveyor = smach.StateMachine(outcomes=['Fail','Succes'])

        sm_robot = smach.StateMachine(outcomes=['Fail','Succes','Box_Placed'])


        # Open the container
        with sm_conveyor:

            # Add states to the container
            smach.StateMachine.add('Idle_conveyor', Idle_conveyor(), 
                                   transitions={'moving_box_starts':'Working'})

            smach.StateMachine.add('Breakdown', Breakdown(), 
                                   transitions={'respawned':'Idle_conveyor', 
                                                'error_flag':'Fail'})
            smach.StateMachine.add('Working', Working(), 
                                   transitions={'moving_box_ends':'Succes', 
                                                'box_dropped':'Breakdown'})

        smach.StateMachine.add('Conveyor_Belt', sm_conveyor,
                               transitions={'Fail':'outcome5', 'Succes': 'outcome6'})
        
        # Open the container
        with sm_robot:

            # Add states to the container
            smach.StateMachine.add('Idle_robot', Idle_robot(), 
                                   transitions={'first_goal_starts':'Moving' , 'Box_Placed': 'Stand_up'})
 
            # Add states to the container
            smach.StateMachine.add('Moving', Moving(), 
                                   transitions={'first_goal_reached':'Docking' , 'second_goal_reached':'Idle_robot' })
                      # Add states to the container
            smach.StateMachine.add('Docking', Docking(), 
                                   transitions={'docking_ends':'Idle_robot'  })
            smach.StateMachine.add('Stand_up', Stand_up(), 
                                   transitions={'second_goal_starts':'Moving'  })

        smach.StateMachine.add('Robot', sm_robot,
                               transitions={'Fail':'outcome7', 'Succes': 'outcome8'})

        
    # Execute SMACH plan
    #outcome = sm_top.execute()
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
