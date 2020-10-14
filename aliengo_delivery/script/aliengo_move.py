
import roslib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler  
from nav_msgs.msg import Odometry

class move_handler:
  def __init__(self):
    self.twist = Twist()
    self.roll=0
    self.pitch=0
    self.yaw=0
    self.q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
    self.odom =Odometry()
    self.odom.pose.pose.position.x = 0.0
    self.odom.pose.pose.position.y = 0.0
    self.odom.pose.pose.position.z = 1.0
    self.odom.pose.pose.orientation.x = self.q[0]
    self.odom.pose.pose.orientation.y = self.q[1]
    self.odom.pose.pose.orientation.z = self.q[2]
    self.odom.pose.pose.orientation.w = self.q[3]
    self.odom.header.frame_id="odom"
    self.is_twist=False
    self.is_odom=False  
    self.control_speed = 0.1
  def start(self):
    self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
    self.pub1 = rospy.Publisher('/aliengo/ref_odom', Odometry, queue_size=10)
  def start_timing(self):
    self.starting = rospy.Time.now().to_sec()

  def robot_idle(self):
    self.input_move(' ')
    if (self.is_twist):
      self.pub.publish(self.twist)
    if (self.is_odom):
      self.pub1.publish(self.odom)
  
  def robot_docking(self):
    now = rospy.Time.now().to_sec()
    sec = now - self.starting
    key = ' '
    print(sec)
    if(sec<1):
      key = 'f' 
    if(sec>=1 and sec < 2):
      key = ' ' 
    if(sec>2):
      return True
    self.input_move(key)
    if (self.is_twist):
      self.pub.publish(self.twist)
    if (self.is_odom):
      self.pub1.publish(self.odom)
    return False
  def robot_standing(self):
    now = rospy.Time.now().to_sec()
    sec = now - self.starting
    key = ' '
    print(sec)
    if(sec<1):
      key = 'r' 
    if(sec>=1 and sec<3):
      key = '-'
    if(sec>=3 and sec<7):
      key = 's'
    if(sec>=7):
      return True
    self.input_move(key)
    if (self.is_twist):
      self.pub.publish(self.twist)
    if (self.is_odom):
      self.pub1.publish(self.odom)
    return False
  def movebase_client(self,x,y,yaw):
    print("waiting move_base server")
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    print("finds move_base server")
    n_goals = 0
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    print("move_base started")
    while (n_goals<len(x)):
        print("Current Goal: "+ str(n_goals) + "Total Goal: " + str(len(x)))
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x[n_goals]
        goal.target_pose.pose.position.y =  y[n_goals]
        q = quaternion_from_euler(0, 0, yaw[n_goals])
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
         
        client.send_goal(goal)
        wait = client.wait_for_result()
        n_goals += 1
    client.send_goal(goal)
    wait = client.wait_for_result()
    return client.get_result()

  def input_move(self,data):
    if data == 'w':
      self.twist.linear.x =self.control_speed ; self.twist.linear.y = 0 ;  self.twist.angular.z = 0
      self.q = quaternion_from_euler(0, 0, 0)
      self.odom.pose.pose.orientation.x = self.q[0]
      self.odom.pose.pose.orientation.y = self.q[1]
      self.odom.pose.pose.orientation.z = self.q[2]
      self.odom.pose.pose.orientation.w = self.q[3]
      self.is_twist = True; self.is_self.odom = False
    elif data== 's':
     self.twist.linear.x = -self.control_speed ; self.twist.linear.y = 0 ;  self.twist.angular.z = 0
     self.q = quaternion_from_euler(0, 0, 0)
     self.odom.pose.pose.orientation.x = self.q[0]
     self.odom.pose.pose.orientation.y = self.q[1]
     self.odom.pose.pose.orientation.z = self.q[2]
     self.odom.pose.pose.orientation.w = self.q[3]
     self.is_twist = True; self.is_odom = False
    elif data== 'q':
     self.twist.angular.z = self.control_speed*2.5 ; self.twist.linear.y = 0 ; self.twist.linear.x = 0
     self.q = quaternion_from_euler(0, 0, 0)
     self.odom.pose.pose.orientation.x = self.q[0]
     self.odom.pose.pose.orientation.y = self.q[1]
     self.odom.pose.pose.orientation.z = self.q[2]
     self.odom.pose.pose.orientation.w = self.q[3]
     self.is_twist = True; self.is_odom = False
    elif data== 'e':
     self.twist.angular.z = -self.control_speed*2.5 ; self.twist.linear.y = 0 ; self.twist.linear.x = 0
     self.q = quaternion_from_euler(0, 0, 0)
     self.odom.pose.pose.orientation.x = self.q[0]
     self.odom.pose.pose.orientation.y = self.q[1]
     self.odom.pose.pose.orientation.z = self.q[2]
     self.odom.pose.pose.orientation.w = self.q[3]
     self.is_twist = True; self.is_odom = False
    elif data== 'a':
     self.twist.linear.x = 0 ; self.twist.linear.y = self.control_speed ;  self.twist.angular.z = 0
     self.q = quaternion_from_euler(0, 0, 0)
     self.odom.pose.pose.orientation.x = self.q[0]
     self.odom.pose.pose.orientation.y = self.q[1]
     self.odom.pose.pose.orientation.z = self.q[2]
     self.odom.pose.pose.orientation.w = self.q[3]
     self.is_twist = True; self.is_odom = False
    elif data== 'r':
      self.odom.pose.pose.position.z =self.odom.pose.pose.position.z+0.05 
      print "z: " + str(self.odom.pose.pose.position.z)
      self.is_twist = False; self.is_odom = True
    elif data== 'f':
      self.odom.pose.pose.position.z =self.odom.pose.pose.position.z-0.05
      print "z: " + str(self.odom.pose.pose.position.z)
      self.is_twist = False; self.is_odom = True

    elif data== ' ':
     self.twist.linear.x = 0 ; self.twist.linear.y = 0 ;  self.twist.angular.z = 0
     self.q = quaternion_from_euler(0, 0, 0)
     self.odom.pose.pose.orientation.x = self.q[0]
     self.odom.pose.pose.orientation.y = self.q[1]
     self.odom.pose.pose.orientation.z = self.q[2]
     self.odom.pose.pose.orientation.w = self.q[3]
     self.is_twist = True; self.is_odom = True
    elif data== '-':
     self.twist.linear.x = 0 ; self.twist.linear.y = 0 ;  self.twist.angular.z = 0
     self.q = quaternion_from_euler(0, 0, 0)
     self.odom.pose.pose.orientation.x = self.q[0]
     self.odom.pose.pose.orientation.y = self.q[1]
     self.odom.pose.pose.orientation.z = self.q[2]
     self.odom.pose.pose.orientation.w = self.q[3]
     self.is_twist = True; self.is_odom = True
  

