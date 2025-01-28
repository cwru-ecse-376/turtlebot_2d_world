# The `turtlebot_2d_world` ROS Package

The `turtlebot_2d_world` package provides a 3D version of the STDR default `sparse_objects` map.

## Usage

To launch a Turtlebot 2 in a 3D version of the STDR default environment, use the following command:

``` bash
ROBOT_INITIAL_POSE="-x 1 -y 2 -z 0" roslaunch turtlebot_2d_world turtlebot_stdr_gazebo.launch
```

