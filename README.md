Controlling ABB IRB 2400L industrial robotic arm using ROS (Robotics Operating System)
Faculty of electrical engeneering, computer science and information technology Osijek, Croatia

Used OS: Ubuntu 16.04 LTS - Xenial Xerus
Used ROS: Kinetic Kame LTS
Used programming language: Python 2.7

Current features:
    - initialize info node
    - control robotic arm using
        - end point space + trajectory planning
        - joint space

Git repo files:
    - abblib - package (folder) for control
    - example files
        - abbTestFile.py - example for using info node
        - abbCmd.py - example for control

Using info node
Function jointsInfo creates new node named "abb_jointListener" and subscribes to the "/joint_states" topic get the current joint angles. Angles displayed are in degrees. Node can be runned as a normal node or as anonymous node. If its run as anonymous node then ROS will append a unique id at the end of the node name.

Usage:
    def jointsInfo(printoutRate=0.5, anonym=False)
    - printoutRate - print rate to terminal in Hz (Refresh time)
                   - defult: 0.5
    - anonym - do you want to run the node as anonymous node
             - default: False