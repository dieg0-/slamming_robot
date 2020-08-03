# Self Localizing Robot
## Robotics Software Engineer Nanodegree Program, Udacity
### Project: Where Am I?

This project demonstrates the usage of the Adaptive Monte Carlo Localization (AMCL) ROS Package, through a custom robot designed via URDF. 

![AMCL-initial](/images/initial_estimate.png)
![AMCL-estimate](/images/robot_localizing_1.png)

#### Usage:
1. Run ``roslaunch self_localizing_robot world.launch``. This starts RVIZ and Gazebo with a custom robot equipped with a camera and a laser scanner.
2. Run ``roslaunch self_localizing_robot amcl.launch``. This starts the map server and loads the corresponding map. Additionally, it starts the navigation stack for the robot, enables teleoperation, and runs the Adaptive Monte Carlo Localization.
3. By default, teleoperation is activated within the amcl.launch file. On the corresponding terminal, use the keyboard to navigate the robot around the map. The green arrows are the localization estimates of the particle filter.
4. The AMCL parameters are already tuned and should work fine for the given map and robot. For further tuning run ``rosrun rqt_reconfigure rqt_reconfigure`` and go to the **amcl** tab.

#### Maps:

The available maps are found under __/self_localizing_robot/maps/__:
1. **my_apartment_map** has been converted using the [pgm_map_creator](https://github.com/udacity/pgm_map_creator) package.
2. **my_gmapped_apartment** has been created using [gmapping](http://wiki.ros.org/gmapping), it is set as default and currently show better results as it contains a more accurate representation of objects such as tables according to what the laser scanner actually sees.