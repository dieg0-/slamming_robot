
# Robot Ball Chaser

This repository contains two packages, which combined simulate a robot capable of chasing a white ball through the environment.

## Usage

1. Start the robot simulation: ``roslaunch my_robot world.launch``
2. Start the ball recognition and motion control: `` roslaunch ball_chaser ball_chaser.launch``
3. Use the Translation Mode Tool within Gazebo to move the white ball around. The robot can move forward or rotate left and right according to the position of the ball. Inspect the specifications of the camera in the corresponding **_plugins.gazebo** file under **my_robot/urdf/**. If the ball is outside of the camera range, it will not be recognized and the motion will not be triggered.

![Ball-chasing Robot](/images/ball_chaser.png)

