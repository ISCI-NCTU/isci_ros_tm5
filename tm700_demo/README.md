#tm700_demo
a simple test move using TM5_700 robot

## Overview
* installation
* usage
* turorial
    * create a Moveit package for an industrial robot
    * use Moveit API to moveit!

## Installation
First check the [*techman_robot*](https://github.com/kentsai0319/techman_robot) package is installed  
Then clone the repository into the src/ folder of the same catkin workspace. It should look like  /path/to/your/catkin_workspace/src/tm700_demo.  
Make sure to source the correct setup file according to your workspace hierarchy, then use ```catkin_make``` to compile.  

## Usage

1. To bring up moveit environment  
    in simulation mode, run:  
    ```
    roslaunch tm700_demo_moveit_config tm700_demo_moveit_planning_execution.launch
    
    ```
    with real robot, run:  
    ```
    roslaunch tm700_demo_moveit_config tm700_demo_moveit_planning_execution.launch sim:=False robot_ip:=192.168.0.10
    ```
    > ## __Warning!__  
    If you are running real robot, remember that you should always have your hands on the red E-Stop button in case there is something in the way or anything unexpected happens.  
    
2. To run the simple test move, open a new terminal and run:  
    
    ```
    rosrun tm700_demo_test tm700_demo_test
    ```

## Tutorial
### create a Moveit package for an industrial robot
see [*create a Moveit package for an industrial robot*](http://wiki.ros.org/Industrial/Tutorials/Create_a_MoveIt_Pkg_for_an_Industrial_Robot).  
the example urdf:    
/path/to/your/catkin_workspace/src/tm700_demo/tm700_demo_test/urdf/tm700_demo.xacro.  
It bases on robot model in *techman\_robot/tm\_description/* and adds a floor in the planning context.

### use Moveit API to moveit!
see [Moveit tutorial](http://docs.ros.org/indigo/api/moveit_tutorials/html/).  
the example code:  
/path/to/your/catkin_workspace/src/tm700_demo/tm700_demo_test/src/tm700_demo_test.cpp.  
it is a ros node call Moveit API and moves the robot to predefined positions.
