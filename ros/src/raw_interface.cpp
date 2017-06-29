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

#include <robotino_calibration/raw_interface.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>


#include <actionlib/client/simple_action_client.h>


RAWInterface::RAWInterface(ros::NodeHandle nh, bool bArmCalibration) :
				CalibrationInterface(nh), arm_state_current_(0)
{
	std::cout << "\n========== RAWInterface Parameters ==========\n";

	// Adjust here: Add all needed code in here to let robot move itself, its camera and arm.
	node_handle_.param<std::string>("camera_joint_controller_command", camera_joint_controller_command_, "/torso/joint_trajectory_controller/command");
	std::cout << "camera_joint_controller_command: " << camera_joint_controller_command_ << std::endl;
	camera_joint_controller_ = node_handle_.advertise<std_msgs::Float64MultiArray/*trajectory_msgs::JointTrajectory*/>(camera_joint_controller_command_, 1, false);

	node_handle_.param<std::string>("camera_state_command", camera_state_command_, "/torso/joint_states");
	std::cout << "camera_state_command: " << camera_state_command_ << std::endl;
	camera_state_ = node_handle_.subscribe<sensor_msgs::JointState>(camera_state_command_, 0, &RAWInterface::cameraStateCallback, this);

	camera_state_current_.resize(2);

	if ( bArmCalibration )
	{
		node_handle_.param<std::string>("arm_joint_controller_command", arm_joint_controller_command_, "/arm/joint_trajectory_controller/command");
		std::cout << "arm_joint_controller_command: " << arm_joint_controller_command_ << std::endl;
		arm_joint_controller_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(arm_joint_controller_command_, 1, false);

		node_handle_.param<std::string>("arm_state_command", arm_state_command_, "/arm/joint_states");
		std::cout << "arm_state_command: " << arm_state_command_ << std::endl;
		arm_state_ = node_handle_.subscribe<sensor_msgs::JointState>(arm_state_command_, 0, &RAWInterface::armStateCallback, this);
	}
	else
	{
		node_handle_.param<std::string>("base_controller_topic_name", base_controller_topic_name_, "/cmd_vel");
		std::cout << "base_controller_topic_name: " << base_controller_topic_name_ << std::endl;
		base_controller_ = node_handle_.advertise<geometry_msgs::Twist>(base_controller_topic_name_, 1, false);
	}

	ROS_INFO("RAWInterface initialized.");
}

RAWInterface::~RAWInterface()
{
}


// CAMERA CALIBRATION INTERFACE

//Callbacks - User defined
void RAWInterface::cameraStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(pan_tilt_joint_state_data_mutex_);
	camera_state_current_[0] = msg->position[0];
	camera_state_current_[1] = msg->position[1];
}


/*void RAWInterface::panTiltJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	//ROS_INFO("Old style controller state received.");
	boost::mutex::scoped_lock lock(pan_tilt_joint_state_data_mutex_);

	pan_joint_state_current_ = msg->position[0];
	tilt_joint_state_current_ = msg->position[1];
}*/

void RAWInterface::armStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(arm_state_data_mutex_);
	arm_state_current_ = new sensor_msgs::JointState;
	*arm_state_current_ = *msg;
}
// End Callbacks

void RAWInterface::assignNewRobotVelocity(geometry_msgs::Twist newVelocity) // Spin and move velocities
{
	// Adjust here: Assign new robot velocity here
	base_controller_.publish(newVelocity);
}

void RAWInterface::assignNewCamaraPanAngle(std_msgs::Float64 newPan)
{
	// Adjust here: Assign new camera pan angle here
}

void RAWInterface::assignNewCamaraTiltAngle(std_msgs::Float64 newTilt)
{
	// Adjust here: Assign new camera tilt angle here
}

void RAWInterface::assignNewCameraAngles(std_msgs::Float64MultiArray newAngles)
{
	// Adjust here: Assign new camera tilt angle here
	/*trajectory_msgs::JointTrajectoryPoint jointTrajPoint;
	trajectory_msgs::JointTrajectory jointTraj;

	jointTraj.joint_names = {"torso_pan_joint", "torso_tilt_joint"};
	jointTrajPoint.positions.insert(jointTrajPoint.positions.end(), newAngles.data.begin(), newAngles.data.end());
	jointTrajPoint.time_from_start = ros::Duration(1);
	jointTraj.points.push_back(jointTrajPoint);
	jointTraj.header.stamp = ros::Time::now();*/

	camera_joint_controller_.publish(newAngles/*jointTraj*/);
}

/*double RAWInterface::getCurrentCameraTiltAngle()
{
	boost::mutex::scoped_lock lock(pan_tilt_joint_state_data_mutex_);
	return tilt_joint_state_current_;
}

double RAWInterface::getCurrentCameraPanAngle()
{
	boost::mutex::scoped_lock lock(pan_tilt_joint_state_data_mutex_);
	return pan_joint_state_current_;
}*/

std::vector<double>* RAWInterface::getCurrentCameraState()
{
	return &camera_state_current_;
}
// END CALIBRATION INTERFACE


// ARM CALIBRATION INTERFACE
void RAWInterface::assignNewArmJoints(std_msgs::Float64MultiArray newJointConfig)
{
	// Adjust here: Assign new joints to your robot arm
	trajectory_msgs::JointTrajectoryPoint jointTrajPoint, currentPoint;
	trajectory_msgs::JointTrajectory jointTraj;

	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("arm", true);
	control_msgs::FollowJointTrajectoryActionGoal goal;



	jointTraj.joint_names = {"arm_elbow_joint", "arm_shoulder_lift_joint", "arm_shoulder_pan_joint", "arm_wrist_1_joint", "arm_wrist_2_joint", "arm_wrist_3_joint"};
	jointTrajPoint.positions.insert(jointTrajPoint.positions.end(), newJointConfig.data.begin(), newJointConfig.data.end());
	jointTrajPoint.time_from_start = ros::Duration(1);
	currentPoint.positions.insert(currentPoint.positions.end(), arm_state_current_->position.begin(), arm_state_current_->position.end());
	jointTraj.points.push_back(currentPoint);
	jointTraj.points.push_back(jointTrajPoint);
	jointTraj.header.stamp = ros::Time::now();

	goal.goal.trajectory = jointTraj;
	ac.sendGoal(goal);
	//arm_joint_controller_.publish(jointTraj); // RAW3-1
}

std::vector<double>* RAWInterface::getCurrentArmState()
{
	boost::mutex::scoped_lock lock(arm_state_data_mutex_);
	return &arm_state_current_->position;
}
// END




