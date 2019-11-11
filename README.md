# heron_software

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

> drive is used to **setup the communication** between the computer and the physical **RoboteQ drivers**. AND to send commands to control the motors.

> odom is used to **compute** and **publish** odometry over ROS using tf

> controller translate inputs from **Xbox controller** to Twist for the robot to move.

Usefull command :

    rosnode info

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

Usefull commands:

    rostopic list
    rostopic echo <topic>
    rostopic pub <topic> <message type> <data>

