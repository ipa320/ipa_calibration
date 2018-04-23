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
 * Date of creation: February 2017
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

#include <calibration_interface/robotino_interface.h>


RobotinoInterface::RobotinoInterface(ros::NodeHandle nh, bool do_arm_calibration) :
				CustomInterface(nh), camera_state_current_(2, 0.), arm_state_current_(0)
{
	std::cout << "\n========== RobotinoInterface Parameters ==========\n";

	// Adjust here: Add all needed code in here to let robot move itself, its camera and arm.
	node_handle_.param<std::string>("pan_controller_command", pan_controller_command_, "");
	std::cout << "pan_controller_command: " << pan_controller_command_ << std::endl;
	node_handle_.param<std::string>("tilt_controller_command", tilt_controller_command_, "");
	std::cout << "tilt_controller_command: " << tilt_controller_command_ << std::endl;
	pan_controller_ = node_handle_.advertise<std_msgs::Float64>(pan_controller_command_, 1, false);
	tilt_controller_ = node_handle_.advertise<std_msgs::Float64>(tilt_controller_command_, 1, false);

	node_handle_.param<std::string>("camera_joint_state_topic", camera_joint_state_topic_, "");
	std::cout << "camera_joint_state_topic: " << camera_joint_state_topic_ << std::endl;
	node_handle_.param<std::string>("pan_joint_name", pan_joint_name_, "");
	std::cout << "pan_joint_name: " << pan_joint_name_ << std::endl;
	node_handle_.param<std::string>("tilt_joint_name", tilt_joint_name_, "");
	std::cout << "tilt_joint_name: " << tilt_joint_name_ << std::endl;

	camera_joint_state_sub_ = node_handle_.subscribe<sensor_msgs::JointState>(camera_joint_state_topic_, 0, &RobotinoInterface::cameraJointStateCallback, this);

	if (do_arm_calibration)
	{
		node_handle_.param<std::string>("arm_joint_controller_command", arm_joint_controller_command_, "");
		std::cout << "arm_joint_controller_command: " << arm_joint_controller_command_ << std::endl;
		arm_joint_controller_ = node_handle_.advertise<std_msgs::Float64MultiArray>(arm_joint_controller_command_, 1, false);

		node_handle_.param<std::string>("arm_state_topic", arm_state_topic_, "");
		std::cout << "arm_state_topic: " << arm_state_topic_ << std::endl;
		arm_state_ = node_handle_.subscribe<sensor_msgs::JointState>(arm_state_topic_, 0, &RobotinoInterface::armStateCallback, this);
	}
	else
	{
		node_handle_.param<std::string>("base_controller_topic_name", base_controller_topic_name_, "");
		std::cout << "base_controller_topic_name: " << base_controller_topic_name_ << std::endl;
		base_controller_ = node_handle_.advertise<geometry_msgs::Twist>(base_controller_topic_name_, 1, false);
	}

	ROS_INFO("RobotinoInterface initialized.");
}

RobotinoInterface::~RobotinoInterface()
{
}


// CAMERA CALIBRATION INTERFACE

//Callbacks - User defined
void RobotinoInterface::cameraJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(camera_joint_state_data_mutex_);
	if (camera_state_current_.size() >= 2)
	{
		for (size_t i=0; i<msg->name.size(); ++i)
		{
			const std::string& name = msg->name[i];
			if (name.compare(pan_joint_name_)==0)
				camera_state_current_[0] = msg->position[i];
			if (name.compare(tilt_joint_name_)==0)
				camera_state_current_[1] = msg->position[i];
		}
	}
}

void RobotinoInterface::armStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(arm_state_data_mutex_);
	arm_state_current_ = new sensor_msgs::JointState;
	*arm_state_current_ = *msg;
}
// End Callbacks

void RobotinoInterface::assignNewRobotVelocity(geometry_msgs::Twist new_velocity) // Spin and move velocities
{
	// Adjust here: Assign new robot velocity here
	base_controller_.publish(new_velocity);
}

void RobotinoInterface::assignNewCameraAngles(std_msgs::Float64MultiArray new_angles)
{
	// Adjust here: Assign new camera angles
	std_msgs::Float64 angle;
	angle.data = new_angles.data[0];
	pan_controller_.publish(angle);
	angle.data = new_angles.data[1];
	tilt_controller_.publish(angle);
}

std::vector<double>* RobotinoInterface::getCurrentCameraState()
{
	boost::mutex::scoped_lock lock(camera_joint_state_data_mutex_);
	return &camera_state_current_;
}
// END CALIBRATION INTERFACE


// ARM CALIBRATION INTERFACE
void RobotinoInterface::assignNewArmJoints(std_msgs::Float64MultiArray new_joint_config)
{
	// Adjust here: Assign new joints to your robot arm
	arm_joint_controller_.publish(new_joint_config);
}

std::vector<double>* RobotinoInterface::getCurrentArmState()
{
	boost::mutex::scoped_lock lock(arm_state_data_mutex_);
	return &arm_state_current_->position;
}
// END




