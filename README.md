# ISCI techman_robot ROS repo

Fork from techman_robot repo

**maintainer: Ren Hao, Yu Shien,yhtsai , Yueh Chuan**

environment: ubuntu14.04 ROS indigo gazebo7

##Prerequest
Install gazebo 7 
[realsense gazebo tutorial](https://paper.dropbox.com/doc/realsense-gazebo-DzoV2B9uNwcnfQbYvvvgL)


## compile package:

this is ROS catkin_ws 

```
git clone https://github.com/ISCI-NCTU/techman_robot.git  techman_robot

cd techman_robot
```

edit below in environment.sh to your user name

```cmake=
vim environment.sh
```
source /home/<YOUR-USER-NAME>/Documents/techman_robot/devel/setup.bash

ex:

source /home/techman/Documents/techman_robot/devel/setup.bash

save and exit.

```
source environment.sh 

catkin_make
```

## How to use:

```cmake=
cd ~/techman_robot
source environment.sh
```

####moveIt!

```
roslaunch tm700_moveit_config tm700_moveit_planning_execution.launch
```

####gazebo + realsense

terminal 1.
```
roslaunch realsense_gazebo_plugin realsense.launch
```
terminal 2.
```
roslaunch tm_gazebo tm700.launch
```




