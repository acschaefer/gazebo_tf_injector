# perfect_control

![perfect_control logo](perfect_control.gif)

## What is the purpose of perfect_control?

This ROS package contains a single node, which makes sure the configuration of
the robot in the Gazebo simulator always exactly represents the desired
configuration published via ROS `sensor_msgs/JointState` messages. It bypasses
the usual control pipeline, which consists of feeding the desired configuration
to joint controllers and letting them control the physical model of the robot in
the simulator. 

The perfect_control package is particularly useful if you want to use Gazebo as
a visualization tool, without relying on its physics engine.

## How to use perfect_control?

First, make sure [Gazebo](http://gazebosim.org/) and [ROS](http://www.ros.org/)
are installed, as well as the ROS packages that let both of them communicate.
Instructions on how to install them can be found
[here](http://gazebosim.org/tutorials?tut=ros_installing).

Second, publish the desired joint states on a topic of type
`sensor_msgs/JointState` and load the robot model into Gazebo.

In order to start the perfect_control node, you need to tell it which robot to
control and where the desired configuration of the robot is published. This
information is passed to the node in the form of private ROS parameters. See the
following examples:

* Control a robot named "R2D2" by listening to sensor_msgs/JointState messages
    on topic "/R2D2/joints".

    ```bash
    $ rosrun perfect_control perfect_control _robot:=R2D2 _topic:=/R2D2/joints
    ```

* Control a robot named "baxter" by listening to the topic "/joint_state".

    ```bash
    $ rosrun perfect_control perfect_control _robot:=baxter _topic:=/joint_state
    ```

The perfect_control node will now listen to joint state messages on the given
topic and adjust the configuration of the given robot in Gazebo by calling the
service `/gazebo/set_model_configuration`.

## Note

Depending on the physics engine selected in Gazebo, updating the configuration
of the model while the physics are running may cause instabilities. If that
happens, start Gazebo in stopped mode, i.e. with the physics engine paused.
