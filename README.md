## Faculty of electrical engeneering, computer science and information technology Osijek, Croatia 
# Controlling ABB IRB 2400L industrial robotic arm using ROS (Robotics Operating System)  

Used OS: Ubuntu 16.04 LTS - Xenial Xerus  
Used ROS: Kinetic Kame LTS  
Used programming language: Python 2.7  

Current features:
* initialize info node  
* control robotic arm using  
    * end point space + trajectory planning  
    * joint space  

Git repo files:
* abblib - package (folder) for control  
* example files  
    * abbTestFile.py - example for using info node  
    * abbCmd.py - example for control  

### Using info node  
Function jointsInfo creates new node named _**abb_jointListener**_ and subscribes to the _**/joint_states**_ topic to get the current joint angles. Angles displayed are in degrees. Node can be runned as a normal node or as anonymous node. If it's run as anonymous node then ROS will append a unique id at the end of the node name.

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

### Controlling the robot
Functions for arm control are implemented in abbRobot class. Making an object does not initialize new node so it should be initialized by the user.

#### Controlling the robot in end point space
Function move2Point moves the robot in end effector point space using MoveIt! package for trajectory planning.  

Usage:
```python
def move2Point(points, eAngles=[[0,0,0]], ax='sxyz', end_effector='link_6')
* points  - list of lists that contain x,y,z coordinates
* eAngles
    * list of lists that contain a,b,g euler angles
    * default [[0,0,0]]
    * if multiple points are passed and only one list of euler angles then those angles are going to be used for all points
* ax
    * specify convention to use for euler angles
    * default: 'sxyz'
* end_effector
    * link whose point you want to move
    * default link_6
```