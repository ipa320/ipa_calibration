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
 * Date of creation: January 2017
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

/* This class represents the interface between your robot and the calibration code.
 * Adjust the interface functions so that the calibration code will work correctly with your robot environment.
*/

#include <robotino_calibration/calibration_interface.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

CalibrationInterface::CalibrationInterface()
{
}

CalibrationInterface::CalibrationInterface(ros::NodeHandle nh, bool bArmCalibration) : node_handle_(nh)
{
	std::cout << "\n========== CalibrationInterface Parameters ==========\n";

	// Adjust here: Add all needed code in here to let robot move itself, its camera and arm.

	if ( bArmCalibration )
	{
		node_handle_.param<std::string>("arm_joint_controller_command", arm_joint_controller_command_, "");
		std::cout << "arm_joint_controller_command: " << arm_joint_controller_command_ << std::endl;
		//arm_joint_controller_ = node_handle_.advertise<std_msgs::Float64MultiArray>(arm_joint_controller_command_, 1, false); // Robotino
		arm_joint_controller_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(arm_joint_controller_command_, 1, false);  // RAW3-1
	}
	else
	{

	}

	ROS_INFO("CalibrationInterface initialized.");
}

CalibrationInterface::~CalibrationInterface()
{
}


// CAMERA CALIBRATION INTERFACE
void CalibrationInterface::assignNewRobotVelocity(geometry_msgs::Twist newVelocity)
{
	// Adjust here: Assign new robot velocity here
}

void CalibrationInterface::assignNewCamaraPanAngle(std_msgs::Float64 newPan)
{
	// Adjust here: Assign new camera pan angle here
}

void CalibrationInterface::assignNewCamaraTiltAngle(std_msgs::Float64 newTilt)
{
	// Adjust here: Assign new camera tilt angle here
}
// END


// ARM CALIBRATION INTERFACE
void CalibrationInterface::assignNewArmJoints(std_msgs::Float64MultiArray newJointConfig)
{
	// Adjust here: Assign new joints to your robot arm
	trajectory_msgs::JointTrajectoryPoint jointTrajPoint;
	trajectory_msgs::JointTrajectory jointTraj;

	jointTrajPoint.positions.insert(jointTrajPoint.positions.end(), newJointConfig.data.begin(), newJointConfig.data.end());
	jointTraj.points.push_back(jointTrajPoint);

	//arm_joint_controller_.publish(newJointConfig); // Robotino
	arm_joint_controller_.publish(jointTraj); // ROW3-1
}
// END



