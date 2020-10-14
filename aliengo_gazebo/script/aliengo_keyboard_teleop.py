#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
import rospy

from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler  
from nav_msgs.msg import Odometry
from aliengo_msgs.msg import Foot, Foots, GaitInfo
import sys, select, termios, tty

msg = """
Control AlienGO
---------------------------
Moving around:
w/s : +/- x liner vel 
a/d : +/- y liner vel 
q/e : +/- z angular vel   
r/f : +/- z pose 
t/g : +/- roll pose
y/h : +/- pitch pose
u/j : =/- yaw pose
m : change individual foot pose 
c : change gait config dynamically
z/x : increase/decrease axis speed
space key: force stop
CTRL-C to quit
"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .2
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('aliengo_teleop')
    pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
    pub1 = rospy.Publisher('/aliengo/ref_odom', Odometry, queue_size=10)
    pub2 = rospy.Publisher('/aliengo/ref_foot', Foots, queue_size=10)
    pub3 = rospy.Publisher('/aliengo/gait_info', GaitInfo, queue_size=10)

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0.2
    control_turn = 0
    twist = Twist()

    roll=0
    pitch=0
    yaw=0

    q = quaternion_from_euler(roll, pitch, yaw)
    odom =Odometry()
    odom.pose.pose.position.x = 0.0
    odom.pose.pose.position.y = 0.0
    odom.pose.pose.position.z = 1.0
    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]
    odom.header.stamp=rospy.Time.now()
    odom.header.frame_id="odom"

    is_twist=False
    is_odom=False
    try:
        print msg
        print vels(speed,turn)
        while not rospy.is_shutdown():
            key = getKey()
            
            
            if key == 'w':
               twist.linear.x = control_speed ; twist.linear.y = 0 ;  twist.angular.z = 0
               q = quaternion_from_euler(0, 0, 0)
               odom.pose.pose.orientation.x = q[0]
               odom.pose.pose.orientation.y = q[1]
               odom.pose.pose.orientation.z = q[2]
               odom.pose.pose.orientation.w = q[3]
               is_twist = True; is_odom = False
            elif key == 's':
               twist.linear.x = -control_speed ; twist.linear.y = 0 ;  twist.angular.z = 0
               q = quaternion_from_euler(0, 0, 0)
               odom.pose.pose.orientation.x = q[0]
               odom.pose.pose.orientation.y = q[1]
               odom.pose.pose.orientation.z = q[2]
               odom.pose.pose.orientation.w = q[3]
               is_twist = True; is_odom = False

            if key == 'q':
               twist.angular.z = control_speed*2.5 ; twist.linear.y = 0 ; twist.linear.x = 0
               q = quaternion_from_euler(0, 0, 0)
               odom.pose.pose.orientation.x = q[0]
               odom.pose.pose.orientation.y = q[1]
               odom.pose.pose.orientation.z = q[2]
               odom.pose.pose.orientation.w = q[3]
               is_twist = True; is_odom = False
            elif key == 'e':
               twist.angular.z = -control_speed*2.5 ; twist.linear.y = 0 ; twist.linear.x = 0
               q = quaternion_from_euler(0, 0, 0)
               odom.pose.pose.orientation.x = q[0]
               odom.pose.pose.orientation.y = q[1]
               odom.pose.pose.orientation.z = q[2]
               odom.pose.pose.orientation.w = q[3]
               is_twist = True; is_odom = False


            elif key == 'a':
               twist.linear.y = control_speed ; twist.linear.x = 0 ;  twist.angular.z = 0
               q = quaternion_from_euler(0, 0, 0)
               odom.pose.pose.orientation.x = q[0]
               odom.pose.pose.orientation.y = q[1]
               odom.pose.pose.orientation.z = q[2]
               odom.pose.pose.orientation.w = q[3]
               is_twist = True; is_odom = False

            elif key == 'd':
               twist.linear.y = -control_speed ; twist.linear.x = 0 ;  twist.angular.z = 0
               q = quaternion_from_euler(0, 0, 0)
               odom.pose.pose.orientation.x = q[0]
               odom.pose.pose.orientation.y = q[1]
               odom.pose.pose.orientation.z = q[2]
               odom.pose.pose.orientation.w = q[3]
               is_twist = True; is_odom = False

            elif key == 'z':
               control_speed = control_speed+ 0.1
               q = quaternion_from_euler(0, 0, 0)
               odom.pose.pose.orientation.x = q[0]
               odom.pose.pose.orientation.y = q[1]
               odom.pose.pose.orientation.z = q[2]
               odom.pose.pose.orientation.w = q[3]
               if control_speed > 2 : 
                  control_speed=2
               print "speed: " + str(control_speed)
            elif key == 'x':
               control_speed=control_speed -0.1
               q = quaternion_from_euler(0, 0, 0)
               odom.pose.pose.orientation.x = q[0]
               odom.pose.pose.orientation.y = q[1]
               odom.pose.pose.orientation.z = q[2]
               odom.pose.pose.orientation.w = q[3]
               if control_speed < 0.1 : 
                  control_speed=0.05
               print "speed: "+ str(control_speed)
            elif key == ' ':
                twist.linear.y = 0 ; twist.linear.x = 0 ;  twist.angular.z = 0
                is_twist = True; is_odom = True
                q = quaternion_from_euler(0, 0, 0)
                odom.pose.pose.orientation.x = q[0]
                odom.pose.pose.orientation.y = q[1]
                odom.pose.pose.orientation.z = q[2]
                odom.pose.pose.orientation.w = q[3]


            elif key == 'r':
              odom.pose.pose.position.z = odom.pose.pose.position.z+0.2 
              print "z: " + str( odom.pose.pose.position.z)
              is_twist = False; is_odom = True


            elif key == 'f':
              odom.pose.pose.position.z = odom.pose.pose.position.z-0.2
              print "z: " + str( odom.pose.pose.position.z)
              is_twist = False; is_odom = True

            elif key == 't':
              roll=roll+0.1
              if(roll>3.1):
                roll=3.1
              q = quaternion_from_euler(roll, pitch,yaw)
              odom.pose.pose.orientation.x = q[0]
              odom.pose.pose.orientation.y = q[1]
              odom.pose.pose.orientation.z = q[2]
              odom.pose.pose.orientation.w = q[3]
              print "roll: " + str(roll)
              is_twist = False; is_odom = True

            elif key == 'g':
              roll=roll-0.1
              if(roll<-3.1):
                roll=-3.1
              q = quaternion_from_euler(roll, pitch,yaw)
              odom.pose.pose.orientation.x = q[0]
              odom.pose.pose.orientation.y = q[1]
              odom.pose.pose.orientation.z = q[2]
              odom.pose.pose.orientation.w = q[3]
              print "roll: " + str(roll)
              is_twist = False; is_odom = True

            elif key == 'y':
              pitch=pitch+0.1
              if(pitch>3.1):
                pitch=3.1
              q = quaternion_from_euler(roll, pitch,yaw)
              odom.pose.pose.orientation.x = q[0]
              odom.pose.pose.orientation.y = q[1]
              odom.pose.pose.orientation.z = q[2]
              odom.pose.pose.orientation.w = q[3]
              print "pitch: " + str(pitch)
              is_twist = False; is_odom = True

            elif key == 'h':
              pitch=pitch-0.1
              if(pitch<-3.1):
                pitch=-3.1
              q = quaternion_from_euler(roll, pitch,yaw)
              odom.pose.pose.orientation.x = q[0]
              odom.pose.pose.orientation.y = q[1]
              odom.pose.pose.orientation.z = q[2]
              odom.pose.pose.orientation.w = q[3]
              print "pitch: " + str(pitch)
              is_twist = False; is_odom = True

            elif key == 'u':
              yaw=yaw+0.1
              if(yaw>3.1):
                yaw=3.1
              q = quaternion_from_euler(roll, pitch,yaw)
              odom.pose.pose.orientation.x = q[0]
              odom.pose.pose.orientation.y = q[1]
              odom.pose.pose.orientation.z = q[2]
              odom.pose.pose.orientation.w = q[3]
              print "yaw: " + str(yaw)
              is_twist = False; is_odom = True

            elif key == 'j':
              yaw=yaw-0.1
              if(yaw<-3.1):
                yaw=-3.1
              q = quaternion_from_euler(roll, pitch,yaw)
              odom.pose.pose.orientation.x = q[0]
              odom.pose.pose.orientation.y = q[1]
              odom.pose.pose.orientation.z = q[2]
              odom.pose.pose.orientation.w = q[3]
              print "yaw: " + str(yaw)
              is_twist = False; is_odom = True

            elif key == 'm': 
              print "FR 0 , FL 1, RR 2 , RL 3"
              foots = Foots ()
              foots.header.stamp=rospy.Time.now()
              foots.header.frame_id="odom"
              f = Foot()
              f.pose.x = 0.0
              f.pose.y = 0.0
              f.pose.z = 0.0
              '''
              #FR
              f.id = 0.0
              foots.foot.append(f)
              #FL
              f.id = 1.0
              foots.foot.append(f)
              #RR
              f.id = 2.0
              foots.foot.append(f)
              #RL
              f.id = 3.0
              foots.foot.append(f)
              '''
              foots.foot.append(f)

              id_ = input("Exit e, Foot id 0-1-2-3: ")
              x_ = input("X translation: ")
              y_ = input("Y translation: ")
              z_ = input("Z translation: ")

              print "foot id: " + str(id_) + " Translation X-Y-Z: "  + str(x_) + ","+ str(y_) + ","+ str(z_) 
              
              foots.foot[0].id = id_
              foots.foot[0].pose.x = x_
              foots.foot[0].pose.y = y_
              foots.foot[0].pose.z = z_
              
              pub2.publish(foots)
              
            elif (key == '\x03'):
                break
            elif key == 'c': 
              newkey = input("max_l_x 0 , max_l_y 1, max_a_z 2 , com_x_t 3, com_y_t 4, swing_h 5, stance_d 6, nominal_h 7: ") 
              gaitinfo = GaitInfo()
              if(newkey == 0):
                val_ = input("max_l_x: ")
                gaitinfo.max_l_x = val_
              if(newkey == 1):
                val_ = input("max_l_y: ")
                gaitinfo.max_l_y = val_
              if(newkey == 2):
                val_ = input("max_a_z: ")
                gaitinfo.max_a_z = val_
              if(newkey == 3):
                val_ = input("com_x_t: ") 
                gaitinfo.com_x_t = val_                           
              if(newkey == 4):
                val_ = input("com_y_t: ")
                gaitinfo.com_y_t = val_
              if(newkey == 5):
                val_ = input("swing_h: ")
                gaitinfo.swing_h = val_
              if(newkey == 6):
                val_ = input("stance_d: ")
                gaitinfo.stance_d = val_
              if(newkey == 7):
                val_ = input("nominal_h: ")
                gaitinfo.nominal_h = val_
              pub3.publish(gaitinfo)
            if (is_twist):
              pub.publish(twist)
            if (is_odom):
              pub1.publish(odom)

 
    finally:
        twist = Twist()
        twist.linear.x = 0;  twist.linear.y = 0 ;  twist.angular.z = 0
        pub.publish(twist)
        q = quaternion_from_euler(roll, pitch, yaw)
        odom =Odometry()
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 1.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.header.stamp=rospy.Time.now()
        odom.header.frame_id="odom"
        pub1.publish(odom)


termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
