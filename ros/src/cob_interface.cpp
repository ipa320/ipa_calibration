/****************************************************************
 *
 * Copyright (c) 2015
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: squirrel
 * ROS stack name: squirrel_calibration
 * ROS package name: robotino_calibration
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Marc Riedlinger, email:marc.riedlinger@ipa.fraunhofer.de
 *
 * Date of creation: September 2017
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <calibration_interface/cob_interface.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <actionlib/client/simple_action_client.h>

CobInterface::CobInterface(ros::NodeHandle nh, bool do_arm_calibration) :
				CustomInterface(nh)
{
	std::cout << "\n========== CobInterface Parameters ==========\n";

	node_handle_.param<std::string>("camera_joint_controller_command", camera_joint_controller_command_, "");
	std::cout << "camera_joint_controller_command: " << camera_joint_controller_command_ << std::endl;
	camera_joint_controller_ = node_handle_.advertise<std_msgs::Float64MultiArray/*trajectory_msgs::JointTrajectory*/>(camera_joint_controller_command_, 1, false);

	node_handle_.param<std::string>("camera_joint_state_topic", camera_joint_state_topic_, "");
	std::cout << "camera_joint_state_topic: " << camera_joint_state_topic_ << std::endl;
	camera_state_ = node_handle_.subscribe<sensor_msgs::JointState>(camera_joint_state_topic_, 0, &CobInterface::cameraStateCallback, this);

	if ( do_arm_calibration )
	{
		node_handle_.param<std::string>("arm_left_command", arm_left_command_, "");
		std::cout << "arm_left_command: " << arm_left_command_ << std::endl;
		arm_left_controller_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(arm_left_command_, 1, false);

		node_handle_.param<std::string>("arm_left_state_topic", arm_left_state_topic_, "");
		std::cout << "arm_left_state_topic: " << arm_left_state_topic_ << std::endl;
		arm_left_state_ = node_handle_.subscribe<sensor_msgs::JointState>(arm_left_state_topic_, 0, &CobInterface::armLeftStateCallback, this);

		node_handle_.param<std::string>("arm_right_command", arm_right_command_, "");
		std::cout << "arm_right_command: " << arm_right_command_ << std::endl;
		arm_right_controller_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(arm_right_command_, 1, false);

		node_handle_.param<std::string>("arm_right_state_topic", arm_right_state_topic_, "");
		std::cout << "arm_right_state_topic: " << arm_right_state_topic_ << std::endl;
		arm_right_state_ = node_handle_.subscribe<sensor_msgs::JointState>(arm_right_state_topic_, 0, &CobInterface::armRightStateCallback, this);
	}
	else
	{
		node_handle_.param<std::string>("base_velocity_command", base_velocity_command_, "");
		std::cout << "base_velocity_command: " << base_velocity_command_ << std::endl;
		base_velocity_controller_ = node_handle_.advertise<geometry_msgs::Twist>(base_velocity_command_, 1, false);
	}

	// /arm_left/joint_group_position_controller/command
	// /arm_right/joint_group_position_controller/command
	// /base/velocity_smoother/command

	ROS_INFO("CobInterface initialized.");
}

CobInterface::~CobInterface()
{
}

// Callbacks
void CobInterface::cameraStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

}

void CobInterface::armLeftStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

}

void CobInterface::armRightStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

}
// End callbacks

void CobInterface::assignNewRobotVelocity(geometry_msgs::Twist newVelocity)
{

}

void CobInterface::assignNewCameraAngles(std_msgs::Float64MultiArray newAngles)
{

}

std::vector<double>* CobInterface::getCurrentCameraState()
{
	return 0;
}

void CobInterface::assignNewArmJoints(std_msgs::Float64MultiArray newJointConfig)
{

}

std::vector<double>* CobInterface::getCurrentArmState()
{
	return 0;
}







