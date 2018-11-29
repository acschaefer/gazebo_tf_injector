# gazebo_tf_injector

![gazebo_tf_injector logo](gazebo_tf_injector.gif)

## What is the purpose of gazebo_tf_injector?

Usually, when you want to move a robot in a Gazebo world, you have to attach motors to its joints, set up a controller for each motor, and then tune the controllers.
This package is meant for all who want to move a robot in Gazebo without worrying about the controllers, by simply mimicing the transformations in the ROS transformation tree.
This might be interesting when you already know the poses of the individual robot joints and you want to simulate camera images or laser scans.

## How to use gazebo_tf_injector?

First, make sure [Gazebo](http://gazebosim.org/) and [ROS](http://www.ros.org/)
are installed, as well as the ROS packages that let both of them communicate.
Instructions on how to install them can be found
[here](http://gazebosim.org/tutorials?tut=ros_installing).

Second, build this package, as described in the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials/BuildingPackages).

Finally, launch the [example script](launch/example.launch) that ships with this package:
```bash
roslaunch gazebo_tf_injector example.launch
```

The result looks like the animation at the top of this page.

Lastly, adapt the [example script](launch/example.launch) to your needs.
