# ros-robotics

The repo consists of robotics projects.

All the projects are done in ROS Kinetic environment(http://wiki.ros.org/kinetic).
The robot simulator software used is Stage(http://wiki.ros.org/stage). 

Lab1:Robot Control
- This is simple pursuer-evader project, where one robot follows the another using its coordinate transforms w.r.t world.
- This consists of simple controller node for the robot which commands the motion of the robot at 2m/s, using laser range
finder(180 degrees) attached to the robot in order to avoid obstacles, a tf- transform node which maps the coordinate
and guides the pursuer robot.
- The pursuer stops within a certain threshold when it moves too close to the evader in order to avoid collision between 
two robots.


Lab2:Perception and Motion planning 
- This project deals with the perception and motion planning module in the robot autonomy.
- The first part of the project uses RANSAC(https://en.wikipedia.org/wiki/Random_sample_consensus) inorder to to output 
a set of lines seen by the robot identifying the obstacles in view.
![screen_3](https://user-images.githubusercontent.com/34932185/51089873-1d65c500-1742-11e9-98ab-cf56b0c63877.png)
- The second half of the project implements Bug2 algorithm for navigation of the robot from source to destination in 
the world view. The robot will mainly be in two states- GOAL_SEEK and WALL_FOLLOW. As soon as the 
robot perceives wall in front of it, changes its state from GOAL_SEEK to WALL_FOLLOW where it starts following the wall, 
upto the end of the wall.

Lab4:Bayes Filter

-  



