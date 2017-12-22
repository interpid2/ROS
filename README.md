Controlling ABB IRB 2400L industrial robotic arm using ROS (Robotics Operating System)\n
Faculty of electrical engeneering, computer science and information technology Osijek, Croatia\n

Used OS: Ubuntu 16.04 LTS - Xenial Xerus\n
Used ROS: Kinetic Kame LTS\n
Used programming language: Python 2.7\n

Current features:\n
    - initialize info node\n
    - control robotic arm using\n
        - end point space + trajectory planning\n
        - joint space\n

Git repo files:\n
    - abblib - package (folder) for control\n
    - example files\n
        - abbTestFile.py - example for using info node\n
        - abbCmd.py - example for control\n

Using info node\n
Function jointsInfo creates new node named "abb_jointListener" and subscribes to the "/joint_states" topic get the current joint angles. Angles displayed are in degrees. Node can be runned as a normal node or as anonymous node. If its run as anonymous node then ROS will append a unique id at the end of the node name.

Usage:\n
    def jointsInfo(printoutRate=0.5, anonym=False)\n
    - printoutRate - print rate to terminal in Hz (Refresh time)\n
                   - defult: 0.5\n
    - anonym - do you want to run the node as anonymous node\n
             - default: False\n