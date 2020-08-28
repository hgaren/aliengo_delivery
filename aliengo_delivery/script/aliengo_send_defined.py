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


if __name__=="__main__":
  rospy.init_node('aliengo_teleop')
  pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
  pub1 = rospy.Publisher('/aliengo/ref_odom', Odometry, queue_size=10)
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
  control_speed = 0.5
  is_twist=False
  is_odom=False  
  count = 0
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():  
    if count <3:
      starting = rospy.Time.now().to_sec()
      count = count +1;

    key = ' '
    now = rospy.Time.now().to_sec()
    sec = now - starting
    print( sec )
    if(sec<8):
      key = 'q'#q
    elif(sec>=8 and sec<14):
      key = 'w'#w
    elif(sec>=14 and sec<22):
      key = 'e' #e
    elif(sec>=22 and sec<24):
      key = 'w' #e
    else:
      key = ' '
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
    elif key == 'q':
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
    elif key == 'r':
      odom.pose.pose.position.z = odom.pose.pose.position.z+0.2 
      print "z: " + str( odom.pose.pose.position.z)
      is_twist = False; is_odom = True


    elif key == 'f':
      odom.pose.pose.position.z = odom.pose.pose.position.z-0.2
      print "z: " + str( odom.pose.pose.position.z)
      is_twist = False; is_odom = True

    elif key == 'y':
      pitch=pitch+0.05
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
      pitch=pitch-0.05
      if(pitch<-3.1):
        pitch=-3.1
      q = quaternion_from_euler(roll, pitch,yaw)
      odom.pose.pose.orientation.x = q[0]
      odom.pose.pose.orientation.y = q[1]
      odom.pose.pose.orientation.z = q[2]
      odom.pose.pose.orientation.w = q[3]
      print "pitch: " + str(pitch)
      is_twist = False; is_odom = True
    elif key == ' ':
     twist.linear.x = 0 ; twist.linear.y = 0 ;  twist.angular.z = 0
     q = quaternion_from_euler(0, 0, 0)
     odom.pose.pose.orientation.x = q[0]
     odom.pose.pose.orientation.y = q[1]
     odom.pose.pose.orientation.z = q[2]
     odom.pose.pose.orientation.w = q[3]
     is_twist = True; is_odom = True
    elif (key == '\x03'):
        break
    if (is_twist):
      pub.publish(twist)
    if (is_odom):
      pub1.publish(odom)
    rate.sleep()
  
