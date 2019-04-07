# turtlebot_moving_obstacles

### Prerequisites

- Ubuntu 18.04, ros melodic. Older versions may work.
- ```sudo apt install python-wstool```

### Compilation
- create a catkin workspace say ```~/catkin_ws``` and clone this repo in src folder
- ```cd ~/catkin_ws/src && wstool init .```
- ```wstool merge -t . turtlebot_moving_obstacles/.rosinstall```
- ```wstool update .```
- Install deps using ```rosdep install --from-paths . --ignore-src --rosdistro=ROSDISTRO```
- Finally compile using ```catkin b --save-config --cmake-args -DCMAKE_BUILD_TYPE=Release```
- Set environment variable for model ```export TURTLEBOT3_MODEL=waffle```

### Simulation environment
The simulation enviroment is a 10m x 10m world with box obstacles. In some of the cases below objects are moving and this is made possible by implementing a gazebo model plugin [here](src/box_animator.cc) and using it in the gazebo world file as below

```
<plugin name="push_animate" filename="libbox_animator.so">
        <time_interval>40.0</time_interval>
        <num_positions>4</num_positions>
        <height>0.2</height>
        <pitch>-1.57</pitch>
        <roll>0</roll>
        <positions>1 0 0  3 -3 -1.57  0 -6 -3.14 -3 -3 -4.71  1 0 -6.28</positions>
</plugin>
```

Here the ```time_interval``` is the interval between each pose, ```num_positions``` is the total number of unique positions, ```height, pitch , roll``` is the fixed Z value, roll and pitch of the positions and ```positions``` are all positions in ```x, y, yaw```. The initial position is then added at the end making it 5 positions and 4 unique positions in the animation.

### Localization

Map based localization is used based on laser and RGBD sensor readings. Cartographer is used to create the map by launching ```roslaunch turtlebot_moving_obstacles slam_static_obstacles.launch``` . In situations where the box obstacles are not needed, this is cleared from the map and all maps needed are created in the ```config``` folder.

AMCL is then used for localization within the map. 

### Navigation in a static environment

The simulation can be run using ```roslaunch turtlebot_moving_obstacles nav_static_obstacles.launch```. A nav goal can be set from rviz to visualize navigation to the goal. This is shown in video below.


The objective here is to enable the robot to navigate in a static and known environment. ```move_base``` is used as the navigation framework and the created map serves as the prior information of all obstacles and will be part of the global costmap. This enables the robot to navigate to its goal even if the sensors are not able to view all obstacles. The respective config files used for ```move_base``` can be found [here](https://github.com/aswinthomas/turtlebot3/tree/master/turtlebot3_navigation/param)

[navfn](https://wiki.ros.org/navfn) is used as the global planner and [dwa_local_planner](https://wiki.ros.org/dwa_local_planner) is used as the local planner. The local planner params can be found [here](https://github.com/aswinthomas/turtlebot3/blob/master/turtlebot3_navigation/param/dwa_local_planner_params_waffle.yaml)

[![Nav in static obstacles](https://img.youtube.com/vi/riDBhGZQAwY/0.jpg)](https://www.youtube.com/watch?v=riDBhGZQAwY)

### Navigation in a dynamic environment

The simulation can be run using ```roslaunch turtlebot_moving_obstacles nav_dynamic_obstacles.launch```. A nav goal can be set from rviz to visualize navigation to the goal. This is shown in video below.

The objective here is to enable navigation in a dynamic environment. The static box objects are now animated using the animator plugin mentioned above.

The framework used is similar to that of the previous step, except that [teb_local_planner](https://wiki.ros.org/teb_local_planner) ishe config file can be found [here](https://github.com/aswinthomas/turtlebot3/blob/master/turtlebot3_navigation/param/teb_local_planner_params.yaml)

[![Nav in dynamic obstacles](https://img.youtube.com/vi/SoZExEGgNP4/0.jpg)](https://www.youtube.com/watch?v=SoZExEGgNP4)


### AR code follower

The simulation can be run using ```roslaunch turtlebot_moving_obstacles nav_ar_follow.launch```. This is shown in video below.

The objective here is to follow an AR code moving in the simulation. An Aruco code model is created in the ```models``` folder and animated similar to the box above. For the Aruco marker detection, [tuw_aruco](https://wiki.ros.org/tuw_aruco) is used and this publishes out markers in the camera view.

A simple follower node [here](src/aruco_follower.cc) subscribes to the detected aruco markers and provides simple P control commands to move the robot towards it. 

[![Nav in dynamic obstacles](https://img.youtube.com/vi/1gM4WWvazS0/0.jpg)](https://www.youtube.com/watch?v=1gM4WWvazS0)
