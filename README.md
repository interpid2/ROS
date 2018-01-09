![FERIT](https://www.ferit.unios.hr/new-images/ferit-web-f-200.png)

#### Faculty of electrical engeneering, computer science and information technology Osijek, Croatia 
# Controlling ABB IRB 2400L industrial robotic arm using ROS (Robotics Operating System)  

Used OS: Ubuntu 16.04 LTS - Xenial Xerus  
Used ROS: Kinetic Kame LTS  
Used programming language: Python 2.7  

Current features:
* initialize info node  
* control robotic arm using  
    * end point space + trajectory planning
    * end point space + constrainted trajectory planning
    * joint space  

Git repo files:
* abblib - package (folder) for control  
* example files  
    * abbTestFile.py - example for using info node  
    * abbCmd.py - example for control  

### 1. Using info node  
Function jointsInfo creates new node named __*abb_jointListener*__ and subscribes to the __*/joint_states*__ topic to get the current joint angles. Angles displayed are in degrees. Node can be runned as a normal node or as anonymous node. If it's run as anonymous node then ROS will append a unique id at the end of the node name.

Usage:  
```python
    def jointsInfo(printoutRate=0.5, anonym=False)  
```
* printoutRate
    * print rate to terminal in Hz (Refresh time)
    * default: 0.5
* anonym
    * do you want to run the node as anonymous node
    * default: False  

Example:  

```python
from abblib import abbCtrl
abbCtrl.jointsInfo(0.5,True)
```

### 2. Controlling the robot
Functions for arm control are implemented in abbRobot class. Object of the class abbRobot does not initialize new node so it should be initialized by the user.

#### 2.1 Controlling the robot in end point space
##### 2.1.1 Controlling with no constraints
Function __*move2Point*__ moves the robot in end effector point space using MoveIt! package for trajectory planning. Because this function uses MoveIt! package, if the given point can not be reached it will print out "No solutions found. Execution not attempted."  

Usage:
```python
def move2Point(points, eAngles=[[0,0,0]], ax='sxyz', end_effector='link_6')
```
* points  - list of lists that contain x,y,z coordinates in meters
* eAngles
    * list of lists that contain alpha,beta and gamma values of euler angles in radians
    * default: [[0,0,0]]
    * if multiple points are passed and only one list of euler angles then those angles are going to be used for all points
* ax
    * specify convention to use for euler angles
    * default: 'sxyz'
* end_effector
    * link whose point you want to move
    * default: link_6

Aditional notes:  
Convention for the parameter ax is specified with a 4 letter string. First letter describes what frame of reference should be. It can be 's'tatic or 'r'otating frame. Remaining characters define order of axis rotation.  

Example - move end effector point to (1,0,1) then (1,1,1) then (1,-1,1) with euler angles (0,0,0). Because euler angles are all 0 for all points then eAngles parameter does not have to be passed in.

```python
from abblib.abbCtrl import abbRobot
import rospy as rp

rp.init_node('abbMove_Main')
robot=abbRobot()
pts=[[1,0,1],[1,1,1],[1,-1,1]]
robot.move2Point(pts)
```

##### 2.1.2 Controlling the robot with end effector constrainted
Function _**cartesian2Point**_ moves the robot in end effector point space in a straight line using MoveIt! package for trajectory planning. Euler angles specify the direction of end effector in which it will be constrained. Trajectory planning can fail because of 2 reasons: generated path contains too much points (>100) or path found is incomplete. If generated path has too much points, consider increasing the resolution parameter. If path found is incomplete then end point is most probably out of range for the constraint.

Usage:  
```python
def cartesian2Point(self, points, eAngles, ax='sxyz',resolution=0.01, jumpStep=0,end_effector='link_6'):
```
* points
    * list of lists that contain x,y,z coordinates
* eAngles
    * list that contains a,b,g euler angles for constraint
* ax
    * specify convention to use for euler angles
    * default: 'sxyz'
* resolution
    * maximum distance between 2 generated points
    * default: 0.01
* jumpStep
    * maximum jump distance between 2 generated points in joint space
    * default: 0
* end_effector
    * link whose point you want to move
    * default: link_6  

Aditional notes:  
Parameter _jumpStep_ describes highest possible movement in joint space. If joint value changes for a higher value than _jumpStep_ then trajectory generated is truncated to a point just before the jump. If _jumpStep_ is equal to 0 then jump detection is disabled.


Example - move end effector point to (1,0,1) then (1,1,1) then (1,-1,1) with euler angles (0,0,0). Move the end effector with default convention, resolution and jumpStep.

```python
from abblib.abbCtrl import abbRobot
import rospy as rp

rp.init_node('abbMove_Main')
robot=abbRobot()
pts=[[1,0,1],[1,1,1],[1,-1,1]]
robot.cartesian2Point(pts,[0,0,0])
```

#### 2.2 Controlling the robot in joint space
Function __*jointAction*__ uses an ROS action to move robot in joint space. While moving, function will print out current desired angles in degrees. Care should be taken when using this function because angles could be out of reach. In that case, FlexPendant will throw an error and stop the RAPID program when joints are close to singularity  while function will return message "_Invalid joints_".

Usage:
```python
def jointAction(jointAngles)
```
* jointAngles
    * list of lists that contain 6 angles in radians in order from joint_1 to joint_6

Example - move to joint values (0,0,0,0,0,0) then to (-pi,0,0,0,0,0) then to (0,pi,-pi,0,0,0) 

```python
from abblib.abbCtrl import abbRobot
import rospy as rp
from math import pi

rp.init_node('abbMove_Main')
robot=abbRobot()
angles=[[0,0,0,0,0,0],[-pi,0,0,0,0,0],[0,pi,-pi,0,0,0]]
robot.jointAction(angles)
```