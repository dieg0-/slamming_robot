# Slamming Robot
## Robotics Software Engineer Nanodegree Program, Udacity
### Project: Map My World

This project demonstrates the usage of RTAB-Map, an implementation of the Graph-SLAM algorithm. A simulated robot equipped with a laser scanner and an RGB-D camera is used for this purpose.

![Real Environment](/images/gazebo_environment.png)

### Usage:
1. Run ``roslaunch slamming_robot world.launch``. This starts RVIZ and Gazebo with the described robot.

2. Run ``roslaunch slamming_robot mapping.launch``. This initializes the RTAB-Map ROS package to enable mapping.

3. Run ``roslaunch slamming_robot teleop.launch`` to start teleoperation. Follow the instructions on the screen output to navigate the robot through the environment. Once the environment is fully mapped, terminate the nodes.

4. The map file (by default __rtabmap.db__) is stored under the __.ros__ directory. Rename it and move it to the desired location.

The following images show an example of a robot mapping the environment as well as the route taken to fully map it:

![Robot mapping](/images/robot_mapping.png)
![Path taken](/images/robot_route_to_map.png)

#### Localization
1. To perform localization, follow the same first three steps, but instead of the mapping launch file, run the localization launch file: ``roslaunch slamming_robot localization.launch``. By default, it uses a map stored under __slamming_robot/maps/my_rtabbed_apartment.db__. 

