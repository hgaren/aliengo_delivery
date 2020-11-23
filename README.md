# Instructions: Package Delivery Scenario in Multi-Floor Environment using State Machine

In paper, the supervisory control of a Discrete Event System (DES) is analyzed to construct autonomous package delivery system. 
The delivery system includes legged robot in order to autonomously navigate uneven indoor terrain and a conveyor belt for transporting the package to  the legged robot. 
The aim of the paper  is using  theory  of supervisory  control  of DES  to supervise and control  machine’s state and event and  ensure robots autonomously collaborate.
By applying the theory, we show collaboration of two individual robots to deliver goods in multi-floor environment
The obtained results from the theory of supervisory control is implemented and verified in simulation environment.
In this code, four legged robot Aliengo and a conveyor belt are implemented in Gazebo environment to collaborate and deliver goods in destinated area. 
We implement Theory of Supervisory Control of Discrete Events-States (DES)  using ROS-Smach library. 



* State Machine library (Smach) is obtained from 
https://github.com/ros/executive_smach
 
* Aliengo Gazebo is obtained from 
https://github.com/unitreerobotics/aliengo_ros and https://github.com/unitreerobotics/laikago_ros

* Aliengo's Gait generator and Balancing Control is obtained from 
https://github.com/chvmp/champ.git

* Note that at this branch, local awarness framework doesnt use in stair climbing application,instead CHAMP lib is used to generate foot placement for climbing (even though it may not be safe for real application). For more information about local awarness for safe foot placement, you can check out elevation mapping and traversability analysis.
## Dependencies:
Requirment:
* Ubuntu 16.04 or 18.04
* ROS kinetic  or melodic
* Gazebo8 or above
```
ros-melodic-gazebo8-ros 
ros-melodic-gazebo8-ros-control
ros-melodic-gazebo8-ros-pkgs
ros-melodic-gazebo8-ros-dev
```
ROS related packages for simulation
```
sudo apt-get install ros-melodic-controller-manager ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-velocity-controllers ros-melodic-position-controllers ros-melodic-robot-controllers ros-melodic-robot-state-publisher
```


## Building ROS workspace:

make a workspace folder (for example: aliengo_delivery_ws) and copy src file into here.
* `mkdir ~/aliengo_delivery_ws/src`
* `cd ~/aliengo_delivery_ws/src`

compile
* `catkin_make`

if compiling gives error, to make sure all ros related packages are installed, go to learning_ws
* `source devel/setup.bash`
* `rosdep install aliengo_navigation`<br>
* `rosdep install aliengo_delivery`<br>
* `rosdep install aliengo_state_mach`<br>
* `rosdep install aliengo_gazebo`<br>
* `rosdep install laikago_controller`<br>

Proposed delivery framework which includes three main block; DES, Conveyor Belt and Legged Robot

![](docs/framework.png?raw=true )

* Framework of our DES implementation on Autonomus Package Delivery


Visualization of multi-floor environment and two collaborative robots
![](docs/scenario.png?raw=true )
![](docs/robots_2.png?raw=true )

* Simulation  Environment  in  Gazebo:  (Up-Left)  indoor-unevenenvironment  which  includes  stair, (Bottom  Left)  Quadrupedal  robot  which a package carrier  is  mounted on it’s  back,  (Bottom  Right)  Conveyor  belt  which  moves box to robot’s back


### aliengo_base
includes foot placement controller
### aliengo_config
includes configs of gaits
### aliengo_navigation:
includes 2D path planing algorithm (move_base) 
### aliengo_description:
including mesh, urdf and xacro files of quadrupedal robot named Aliengo A1
### aliengo_gazebo:
Spawns aliengo robot, conveyor belt and a box in Gazebo enviornment (normal_aliengo.launch) 
### laikago_controller:
Laikago's default joint controller, subscribes joint commands via topics
### laikago_msgs:
Laikago's default msgs


## How to Execute The Scenario
Open first terminal;
* `roscore`

Open second terminal; (opens gazebo and rviz while spawning Aliengo, conveyor and a box)
 
* `roslaunch aliengo_gazebo normal_aliengo.launch`

Open third terminal; (opens state machine)
 
* `rosrun aliengo_state_mach state_machine.py`

Open fourth terminal; (opens state-event controller) and press s to start

* `rosrun aliengo_delivery aliengo_delivery.py`

Additionally to visualize states and events 
* `rosrun smach_viewer smach_viewer.py `

 

Gazebo Snapshots of Autonmous Delivery Scenario

![](docs/stages.png?raw=true )
* Snapshot  of  Autonomous  Package  Delivery  Scenario:(Up-Right)initial Position , (Up-Left) first goal reached, (Bottom-Left) climbing to thestairs, (Bottom-Right) second goal reached


State Machine Snapshots of Autonmous Delivery Scenario (DES is obtained from Theory of Supervisory Control)
![](docs/state_machine.png?raw=true )
* States and Events in ROS-Smach Implementation 


## Results

Robot is able to pick-up package and delivery one floor above by climbing stair.
![](docs/scenario.gif?raw=true)
* Video Result of Implemening State Machine considering all CONTRALLABLE states and events 


Robots are able to return initial starting state if robot is falling or box is falling
![](docs/uncontrollable.gif?raw=true)
* Video Result of Implemening State Machine considering all UNCONTRALLABLE states and events 



