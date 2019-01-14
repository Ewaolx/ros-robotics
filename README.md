# ros-robotics

The repo consists of robotics projects.

All the projects are done in ROS Kinetic environment(http://wiki.ros.org/kinetic).
The robot simulator software used is Stage(http://wiki.ros.org/stage). 

Lab1: Robot Control
- This is simple pursuer-evader project, where one robot follows the another using its coordinate transforms w.r.t world.
- This consists of simple controller node for the robot which commands the motion of the robot at 2m/s, using laser range
finder(180 degrees) attached to the robot in order to avoid obstacles, a tf- transform node which maps the coordinate
and guides the pursuer robot.
- The pursuer stops within a certain threshold when it moves too close to the evader in order to avoid collision between 
two robots.


Lab2: Perception and Motion planning 
- This project deals with the perception and motion planning module in the robot autonomy.
- The first part of the project uses RANSAC(https://en.wikipedia.org/wiki/Random_sample_consensus) in order to to output 
a set of lines seen by the robot through laser scanner sensor identifying the obstacles in view.

![screen_3](https://user-images.githubusercontent.com/34932185/51089873-1d65c500-1742-11e9-98ab-cf56b0c63877.png)
- The second half of the project implements Bug2 algorithm for navigation of the robot from source to destination in 
the world view. The robot will mainly be in two states- GOAL_SEEK and WALL_FOLLOW. As soon as the 
robot perceives wall in front of it, changes its state from GOAL_SEEK to WALL_FOLLOW where it starts following the wall, 
upto the end of the wall.

Lab4: Bayes Filter
- This project deals with the localization module of the robot autonomy.
- It uses the grid localization which is a variant of discrete Bayes Localization
- The map is discretized into an occupancy grid, where each cell in the grid represents the probability of robot in that
particular cell at a given timestep.
- The grid cells with maximum probabilities at each step, characterize the robot’s trajectory. Grid Localization runs in two iterative steps —Motion and Observation.
- The robot's motion commands and its perception at each time step is read from the ROSbag file. There are total 6 landmarks
in the map at discrete locations.
- In order to follow same unit transformation for the cell location and robot's location, the robot coordinates in 2D and
its heading angle is discretized.
- For the motion step, the odometry model is used which consists of- rotation_1,translation and rotation_2.
- The noise for both motion and observation steps are added from Gaussian distribution.
- The final output after 178 motion and observation timesteps consists of trajectory(trajectory.txt) of the robot, indicating
the location of robot in the world at that particular timestep.

Lab5: A* Planning
- This project deals with the path-planning module of the robot autonomy.
- It implements the A*(https://en.wikipedia.org/wiki/A*_search_algorithm) which find a optimal route from a starting point to goal by avoiding obstacles in its way.
- The map.txt represents the map- 0 indicating free space, while 1 indicating occupied.
- The euclidean distance is used as heurisitic cost between the goal and starting point.
- Once the path is formed, the robot moves from starting point to end following that path.


# Usage

- Drop the folder into catkin workspace in `src/` folder.
- From catkin workspace do `catkin_make --only-pkg-with-deps --folder_name` to build the code in workspace.
- Run using `roslaunch folder_name launch_file.launch` in order to launch the program.
- In order to install Stage, do `sudo apt-get install ros-kinetic-stage-ros`.











