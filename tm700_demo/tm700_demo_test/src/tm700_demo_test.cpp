/*********************************************************************
 * tm700_demo_node.cpp
 *
 * Copyright 2016 Copyright 2016 Techman Robot Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************
 *
 * Author: Yun-Hsuan Tsai
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

#include "tm_msgs/SetIO.h"
//#include "tm_msgs/SetIORequest.h"
//#include "tm_msgs/SetIOResponse.h"



//get current joint values of TM5
void get_current_joint_values(moveit::planning_interface::MoveGroup& group,
                              moveit::planning_interface::MoveGroup::Plan& plan
														){
		//if(!ros::ok()) return false;
		bool success = false;

		std::vector<double> joint_value;
		joint_value = group.getCurrentJointValues();

		for(int i = 0; i<joint_value.size(); i++){
			joint_value[i] = joint_value[i]*180/M_PI;
			printf("Joint %d: %lf\n",i+1, joint_value[i]);
		}
}

bool try_move_to_named_target(moveit::planning_interface::MoveGroup& group,
			      moveit::planning_interface::MoveGroup::Plan& plan,
			      const std::string& target_name,
			      unsigned int max_try_times = 1
			     ) {
  if(!ros::ok()) return false;
  bool success = false;

  for(unsigned int i = 0; i < max_try_times; i++) {

    group.setNamedTarget(target_name);

    if(group.move()) {
      success = true;
      break;
    }
    else {
      if(!ros::ok()) break;
      sleep(1);
    }
  }
  return success;
}

bool try_move_to_joint_target(moveit::planning_interface::MoveGroup& group,
			      moveit::planning_interface::MoveGroup::Plan& plan,
			      const std::vector<double>& joint_target,
			      unsigned int max_try_times = 1
			     ) {
  if(!ros::ok()) return false;
  bool success = false;

  for(unsigned int i = 0; i < max_try_times; i++) {

    group.setJointValueTarget(joint_target);

    if(group.move()) {
      success = true;
      break;
    }
    else {
      if(!ros::ok()) break;
      sleep(1);
    }
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tm700_demo_test");
  ros::NodeHandle node_handle;
  ros::ServiceClient set_io_client = node_handle.serviceClient<tm_msgs::SetIO>("tm_driver/set_io");
  tm_msgs::SetIO io_srv;
  io_srv.request.fun = 2;//tm_msgs::SetIORequest::FUN_SET_EE_DIGITAL_OUT;
  io_srv.request.ch = 0;
  io_srv.request.value = 0.0;

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sleep(1);

  // Setup
  // ^^^^^
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup group("manipulator");
  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // (Optional) Create a publisher for visualizing plans in Rviz.
  //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  //moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  moveit::planning_interface::MoveGroup::Plan my_plan;

  set_io_client.call(io_srv);

  group.setPlanningTime(30.0);

  //try_move_to_named_target(group, my_plan, "home", 100);

  std::vector<double> joint_target_1;
  std::vector<double> joint_target_2;

  joint_target_1.assign(6, 0.0f);
  joint_target_1[0] = 0.0174533*( 30.0);
  joint_target_1[1] = 0.0174533*( 15.0);
  joint_target_1[2] = 0.0174533*(105.0);
  joint_target_1[3] = 0.0174533*(-30.0);
  joint_target_1[4] = 0.0174533*( 90.0);
  joint_target_1[5] = 0.0174533*( 30.0);

  joint_target_2.assign(6, 0.0f);
  joint_target_2[0] = 0.0174533*(-30.0);
  joint_target_2[1] = 0.0174533*(-15.0);
  joint_target_2[2] = 0.0174533*( 90.0);
  joint_target_2[3] = 0.0174533*(-75.0);
  joint_target_2[4] = 0.0174533*(120.0);

  int step = 0;

  while (ros::ok()) {
    switch (step) {
      case 0: //gripper off
	io_srv.request.fun = 2;
	io_srv.request.ch = 0;
	io_srv.request.value = 0.0;
	if (set_io_client.call(io_srv))
	  ROS_INFO("Set IO success");
	else
	  ROS_WARN("Failed to call service set_io");
	break;

      case 1: //move to ready
	ROS_INFO("move...");
	//try_move_to_named_target(group, my_plan, "ready", 100);
	break;

      case 2: //light on
/*
	io_srv.request.fun = 2;
	io_srv.request.ch = 3;
	io_srv.request.value = 1.0;
*/
	if (set_io_client.call(io_srv))
	  ROS_INFO("Set IO success");
	else
	  ROS_WARN("Failed to call service set_io");
	break;

      case 3: //wait 3 sec
	sleep(3);
	break;

      case 4: //light off
	io_srv.request.fun = 2;
	io_srv.request.ch = 3;
	io_srv.request.value = 0.0;
	if (set_io_client.call(io_srv))
	  ROS_INFO("Set IO success");
	else
	  ROS_WARN("Failed to call service set_io");
	break;

      case 5: //move 1
	//ROS_INFO("move...");
	//try_move_to_joint_target(group, my_plan, joint_target_1, 100);
	ROS_INFO("Read Joints Values");
	get_current_joint_values(group, my_plan);
	break;

      case 6: //gripper on
	io_srv.request.fun = 2;
	io_srv.request.ch = 0;
	io_srv.request.value = 1.0;
	if (set_io_client.call(io_srv))
	  ROS_INFO("Set IO success");
	else
	  ROS_WARN("Failed to call service set_io");
	break;

      case 7: //wait 1 sec
	sleep(1);
	break;

      case 8: //move 2...
	ROS_INFO("move...");
	//try_move_to_joint_target(group, my_plan, joint_target_2, 100);
	break;
    }
    step = (step + 1) % 9;
  }
  return 0;
}
