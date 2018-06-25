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
 * Date of creation: June 2018
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


#include <calibration_interface/camera_arm_type.h>


CameraArmType::CameraArmType()
{
}

CameraArmType::~CameraArmType()
{
}

void CameraArmType::initialize(ros::NodeHandle nh, IPAInterface* calib_interface)
{
	CalibrationType::initialize(nh, calib_interface);  // call parent

	std::cout << "\n========== Camera-Arm Parameters ==========\n";

	node_handle_.param("arm_dof", arm_dof_, 0);
	std::cout << "arm_dof: " << arm_dof_ << std::endl;

	if ( arm_dof_ < 1 )
	{
		std::cout << "Error: Invalid arm_dof: " << arm_dof_ << ". Setting arm_dof to 1." << std::endl;
		arm_dof_ = 1;
	}

	node_handle_.param("max_angle_deviation", max_angle_deviation_, 0.5);
	std::cout << "max_angle_deviation: " << max_angle_deviation_ << std::endl;

	std::vector<double> temp;
	node_handle_.getParam("robot_configurations", temp);
	const int num_params = arm_dof_ + camera_dof_;

	if ( temp.size() % num_params != 0 || temp.size() < 3*num_params )
	{
		ROS_ERROR("The robot_configurations vector should contain at least 3 configurations with %d values each.", num_params);
		return;
	}
	const int num_configs = temp.size()/num_params;

	for ( int i=0; i<num_configs; ++i )
	{
		std::vector<double> angles;
		for ( int j=0; j<arm_dof_; ++j )
		{
			angles.push_back(temp[num_params*i + j]);
		}
		arm_configurations_.push_back(angles);

		angles.clear();
		for ( int j=arm_dof_; j<num_params; ++j ) // camera_dof_ iterations
		{
			angles.push_back(temp[num_params*i + j]);
		}
		camera_configurations_.push_back(angles);
	}

	// Display configurations
	std::cout << "arm configurations:" << std::endl;
	for ( int i=0; i<arm_dof_; ++i )
	{
		for ( int j=0; j<arm_configurations_[i].size(); ++j )
			std::cout << arm_configurations_[i][j] << "/t";
		std::cout << std::endl;
	}
	std::cout << "camera configurations:" << std::endl;
	for ( int i=0; i<camera_dof_; ++i )
	{
		for ( int j=0; j<camera_configurations_[i].size(); ++j )
			std::cout << camera_configurations_[i][j] << "/t";
		std::cout << std::endl;
	}
}

bool CameraArmType::moveRobot(int config_index)
{
	bool result = CalibrationType::moveRobot(config_index);  // call parent

	if ( result )
	{
		result &= moveArm(arm_configurations_[config_index]);
	}

	return result;
}

bool CameraArmType::moveArm(const std::vector<double>& arm_configuration)
{
	std_msgs::Float64MultiArray new_joint_config;
	new_joint_config.data.resize(arm_configuration.size());

	for ( int i=0; i<new_joint_config.data.size(); ++i )
		new_joint_config.data[i] = arm_configuration[i];

	std::vector<double> cur_state = *calibration_interface_->getCurrentArmState();
	if ( cur_state.size() != arm_configuration.size() )
	{
		ROS_ERROR("Size of target arm configuration and count of arm joints do not match! Please adjust the yaml file.");
		return false;
	}

	// Ensure that target angles and current angles are not too far away from one another to avoid collision issues!
	for ( size_t i=0; i<cur_state.size(); ++i )
	{
		double delta_angle = 0.0;

		do
		{
			cur_state = *calibration_interface_->getCurrentArmState();
			delta_angle = arm_configuration[i] - cur_state[i];

			while (delta_angle < -CV_PI)
				delta_angle += 2*CV_PI;
			while (delta_angle > CV_PI)
				delta_angle -= 2*CV_PI;

			if ( fabs(delta_angle) > max_angle_deviation_ )
			{
				ROS_WARN("%d. target angle exceeds max allowed deviation of %f!\n"
						 "Please move the arm manually closer to the target position to avoid collision issues. Waiting 5 seconds...", (int)(i+1), max_angle_deviation_);
				std::cout << "Current arm state: ";
				for ( size_t j=0; j<cur_state.size(); ++j )
					std::cout << cur_state[j] << (j<cur_state.size()-1 ? "\t" : "\n");
				std::cout << "Target arm state: ";
				for ( size_t j=0; j<cur_state.size(); ++j )
					std::cout << arm_configuration[j] << (j<cur_state.size()-1 ? "\t" : "\n");
				ros::Duration(5).sleep();
				ros::spinOnce();
			}
		} while ( fabs(delta_angle) > max_angle_deviation_ && ros::ok() );
	}

	//arm_joint_controller_.publish(new_joint_config);
	calibration_interface_->assignNewArmJoints(new_joint_config);

	//Wait for arm to move
	if ( (*calibration_interface_->getCurrentArmState()).size() > 0 )
	{
		Timer timeout;
		while (timeout.getElapsedTimeInSec()<10.0) //Max. 10 seconds to reach goal
		{

			boost::mutex::scoped_lock(arm_state_data_mutex_);
			cur_state = *calibration_interface_->getCurrentArmState();
			std::vector<double> difference(cur_state.size());
			for (int i = 0; i<cur_state.size(); ++i)
				difference[i] = arm_configuration[i]-cur_state[i];

			double length = std::sqrt(std::inner_product(difference.begin(), difference.end(), difference.begin(), 0.0)); //Length of difference vector in joint space

			if ( length < 0.025 ) //Close enough to goal configuration (~1° deviation allowed)
			{
				std::cout << "Arm configuration reached, deviation: " << length << std::endl;
				break;
			}

			ros::spinOnce();
		}

		if ( timeout.getElapsedTimeInSec()>=10.0 )
		{
			ROS_WARN("Could not reach following arm configuration in time:");
			for (int i = 0; i<arm_configuration.size(); ++i)
				std::cout << arm_configuration[i] << "\t";
			std::cout << std::endl;
		}
	}
	else
		ros::Duration(1).sleep();

	ros::spinOnce();
	return true;
}
