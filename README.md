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

### Navigation in a static environment
The simulation enviroment is a 10m x 10m world with cube obstacles.

A map must be initially created by launching ```roslaunch turtlebot_moving_obstacles nav_static_obstacles.launch```. This has already been created in the config folder. 

move_base is used as the navigation framework and the created map serves as the prior information of all obstacles and will be part of the global costmap. This enables the robot to navigate to its goal even if the sensors are not able to view all obstacles. 

After launching ```roslaunch turtlebot_moving_obstacles nav_static_obstacles.launch``` , a nav goal can be set from rviz. This is shown in video below.

[![Nav in static obstacles](https://img.youtube.com/vi/riDBhGZQAwY/0.jpg)](https://www.youtube.com/watch?v=riDBhGZQAwY)

### Navigation in a dynamic environment
[![Nav in dynamic obstacles](https://img.youtube.com/vi/SoZExEGgNP4/0.jpg)](https://www.youtube.com/watch?v=SoZExEGgNP4)