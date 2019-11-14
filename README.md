# heron_software

This is a ROS package.

**Requirements :**
- Ubuntu 16.04 LTS 
- ROS kinetic
- catkin tools

## Getting Started !

First clone this repository in your catkin workspace :
    
    cd catkin_ws
    catkin_ws git clone <url>

Then build using catkin tools

    catkin build

Finally source

    source /devel/setup.bash

Now you are ready for next section.

## Launch
You can easily launch all nodes required to use the Robot by typing in a terminal :

    roslaunch heron heronController.launch

Then simply power on the Xbox controller and you can start moving the base around !

---------------------------

## ROS Nodes

Here is a list of the nodes :
- drive
- odom
- controller
- joy_node (from external package xbox_controller)

### drive
- **Sets up the communication** between the computer and the physical **RoboteQ drivers**. 
- Sends **commands** to control the **motors** via the drivers.
- Gets **feedbacks** from **encoders** and publishes them. 

> Based on the API provided by RoboteQ the drivers manufacturer.

### odom
**Computes** and **publishes** odometry over ROS using tf.

### controller
Translates inputs from **Xbox controller** to Twist to make the robot move.

*Usefull command :*

    rosnode info <node>

---------------------------
## Usefull ROS Topics

Topics are defined under a **namespace**. If you have a fleet of robots that operate on one **ROS_MASTER**, you will have those topics preceded by the name of each robot.

> **For example :** My robot is named Heron01
The following topics will be `/Heron01/...`

|Topic|message type|Publisher Node|Subscriber Node|
|---|:---|:---|:---|
|`cmd_vel`|*Twist*|controller|drive|
|`odom`|*Odometry*| odom|
|`sensor_encs`|custom|drive|odom|
|`joy`|*Joy*|joy_node (from external package)|controller|

*Usefull commands :*

    rostopic list
    rostopic echo <topic>
    rostopic pub <topic> <message type> <data>

