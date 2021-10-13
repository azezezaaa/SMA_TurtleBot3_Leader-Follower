# SMA_TurtleBot3_Leader-Follower
This repo contains the code for a project on using TurtleBot3 for a leader-follower navigation system. This repo was used as lab practical in MIAR.

# Launch 3 turtlebots and operate them independently.
## (1) Install prerequisites:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_applications.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs.git
$ cd .. && catkin_make
$ source devel/setup.bash
$ cd src/ 
```
## (2) Download the package.
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/azezezaaa/SMA_TurtleBot3_Leader-Follower
```
## (3) Update the modified content: 
```
$ chmod +x ~/catkin_ws/src/SMA_TurtleBot3_Leader-Follower/script/sma_fleet.py
$ cd ~/catkin_ws && catkin_make && source devel/setup.bash
$ rospack profile
```
## (4) Launch turtlebots simulation:
```
$ roslaunch SMA_TurtleBot3_Individual-Navigation simulation.launch
```
## (5) Run Teleoperation:
Open another terminal and write: 
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
