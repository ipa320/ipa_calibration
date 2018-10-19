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
#include <calibration_interface/ipa_interface.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <robotino_calibration/time_utilities.h>


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

	// arms list format: [arm_name, DoF-count, max_delta_angle]
	std::vector<std::string> arms_list;
	node_handle_.getParam("arms_list", arms_list);
	std::cout << "arms_list:" << std::endl;
	for ( int i=0; i<arms_list.size(); i+=3 )
	{
		arm_description arm_desc;
		arm_desc.arm_name_ = arms_list[i];
		arm_desc.dof_count_ = std::stoi(arms_list[i+1]);
		arm_desc.max_delta_angle_ = fabs(std::stod(arms_list[i+2]));

		if ( arm_desc.dof_count_ < 1 )
		{
			ROS_WARN("Invalid DoF count %d for arm %s, skipping arm.", arm_desc.dof_count_, arm_desc.arm_name_.c_str());
			continue;
		}

		arms_.push_back(arm_desc);
		std::cout << arms_list[i] << ": DoF " << arms_list[i+1] << ", max delta angle: " << arms_list[i+2] << std::endl;
	}

	if ( arms_.empty() )
	{
		ROS_ERROR("Arms list empty, check yaml file");
		return;
	}

	// Read in arm data
	for ( int i=0; i<arms_.size(); ++i )
	{
		std::vector<double> arm_data;  // raw data that will be read in
		node_handle_.getParam((arms_[i].arm_name_+"_configs"), arm_data);
		const int arm_dof = arms_[i].dof_count_;

		if ( !arm_data.empty() )  // configs has been found
		{
			if ( arm_dof == 0 || arm_data.size() % arm_dof != 0 )
			{
				ROS_ERROR("%s_configs vector has wrong size, arm DoF %d", arms_[i].arm_name_.c_str(), arm_dof);
				return;
			}

			for ( int j=0; j<arm_data.size(); j+=arm_dof )
			{
				std::vector<double> values;
				values.reserve(arm_dof);
				for ( int k=j; k<j+arm_dof; ++k )
					values.push_back(arm_data[k]);

				arms_[i].configurations_.push_back(values);
			}
		}
		else  // remove entry
		{
			ROS_WARN("No configurations has been generated for arm %s, removing arm.", arms_[i].arm_name_.c_str());
			arms_.erase(arms_.begin()+i);
			--i;
		}
	}

	// Compute maximum configuration count over all entities (cameras and arms)
	max_configuration_count = 0;
	for ( int i=0; i<cameras_.size(); ++i )  // get max count of configs over all cameras
	{
		int cam_configs = cameras_[i].configurations_.size();

		if ( cam_configs > max_configuration_count )
			max_configuration_count = cam_configs;
	}

	int total_arms_configs = 0;
	for ( int i=0; i<arms_.size(); ++i )  // get max count of configs over all arms and total count
	{
		int arm_configs = arms_[i].configurations_.size();
		total_arms_configs += arm_configs;

		if ( arm_configs > max_configuration_count )
			max_configuration_count = arm_configs;
	}

	// Display arm configurations
	for ( int i=0; i<arms_.size(); ++i )
	{
		std::cout << arms_[i].arm_name_ << " configurations:" << std::endl;
		for ( int j=0; j<arms_[i].configurations_.size(); ++j )
		{
			for ( int k=0; k<arms_[i].configurations_[j].size(); ++k )
				std::cout << arms_[i].configurations_[j][k] << "\t";
			std::cout << std::endl;
		}
		std::cout << std::endl;
	}

	if ( total_arms_configs <= 0 )
	{
		ROS_WARN("Invalid total configuration count for arms: %d", total_arms_configs);
		return;
	}

	initialized_ = true;
}

bool CameraArmType::moveRobot(int config_index)
{
	// Cameras move only when their assigned arms have finished moving
	// How detect when an arm has finished moving?
	// Camera moves -> arm moves in whole field of view of camera (corners, middle, etc), after that, camera moves and arm does same procedure again

	bool result = CalibrationType::moveRobot(config_index);  // call parent

	// Move arms
	if ( result )
	{
		for ( int i=0; i<arms_.size(); ++i )  // move one arm after the other
		{
			if ( config_index >= arms_[i].configurations_.size() )  // this arm has finished, continue with next
				continue;

			for ( short j=0; j<NUM_MOVE_TRIES; ++j )
			{
				unsigned short error_code = moveArm(arms_[i], arms_[i].configurations_[config_index]);

				if ( error_code == MOV_NO_ERR ) // Exit loop, as successfully executed move
				{
					break;
				}
				else if ( error_code == MOV_ERR_SOFT ) // Retry last failed move
				{
					ROS_WARN("CameraArmType::moveRobot: Could not execute moveRobot, (%d/%d) tries.", i+1, NUM_MOVE_TRIES);
					if ( i<NUM_MOVE_TRIES-1 )
					{
						ROS_INFO("CameraArmType::moveRobot: Trying again in 2 sec.");
						ros::Duration(2.f).sleep();
					}
					else
						ROS_WARN("CameraArmType::moveRobot: Skipping arm configuration %d of %s.", config_index, arms_[i].arm_name_.c_str());
				}
				else
				{
					ROS_FATAL("CameraArmType::moveRobot: Exiting calibration.");
					throw std::exception();
				}
			}
		}
	}

	return result;
}

bool CameraArmType::moveCameras(int config_index)
{
	for ( int i=0; i<cameras_.size(); ++i )  // move one camera after the other
	{
		if ( config_index >= cameras_[i].configurations_.size() )  // this camera has finished, continue with next
			continue;

		for ( short j=0; j<NUM_MOVE_TRIES; ++j )
		{
			unsigned short error_code = moveCamera(cameras_[i], cameras_[i].configurations_[config_index]);

			if ( error_code == MOV_NO_ERR ) // Exit loop, as successfully executed move
			{
				break;
			}
			else if ( error_code == MOV_ERR_SOFT ) // Retry last failed move
			{
				ROS_WARN("CalibrationType::moveRobot: Could not execute moveCamera, (%d/%d) tries.", i+1, NUM_MOVE_TRIES);
				if ( i<NUM_MOVE_TRIES-1 )
				{
					ROS_INFO("CalibrationType::moveRobot: Trying again in 2 sec.");
					ros::Duration(2.f).sleep();
				}
				else
					ROS_WARN("CalibrationType::moveRobot: Skipping camera configuration %d of %s.", config_index, cameras_[i].camera_name_.c_str());
			}
			else
			{
				ROS_FATAL("CalibrationType::moveRobot: Exiting calibration.");
				throw std::exception();
			}
		}
	}

	return true;
}

unsigned short CameraArmType::moveArm(const arm_description &arm, const std::vector<double>& arm_configuration)
{
	std_msgs::Float64MultiArray new_joint_config;
	new_joint_config.data.resize(arm_configuration.size());

	unsigned short error_code = MOV_NO_ERR;
	const std::string &arm_name = arm.arm_name_;

	for ( int i=0; i<new_joint_config.data.size(); ++i )
		new_joint_config.data[i] = arm_configuration[i];

	ros::spinOnce();
	std::vector<double> cur_state = *calibration_interface_->getCurrentArmState(arm_name);

	if ( cur_state.empty() )
	{
		ROS_ERROR("Can't retrieve state of current arm %s.", arm_name.c_str());
		return MOV_ERR_FATAL;
	}

	if ( cur_state.size() != arm_configuration.size() )
	{
		ROS_ERROR("Size of target arm configuration and count of arm joints do not match! Please adjust the yaml file.");
		return MOV_ERR_FATAL;
	}

	// Ensure that target angles and current angles are not too far away from one another to avoid collision issues!
	const double &max_delta_angle = arm.max_delta_angle_;
	if ( max_delta_angle > 0.f )
	{
		int bad_index = -1;
		if ( checkForMaxDeltaAngle(cur_state, arm_configuration, max_delta_angle, bad_index) )
		{
			if ( bad_index == -1 )
			{
				ROS_ERROR("Size of target arm configuration and count of arm joints do not match! Please adjust the yaml file.");
				return MOV_ERR_FATAL;
			}
			else
			{
				ROS_WARN("Angle number %d in target configuration of arm %s exceeds max allowed deviation %f!\n"
						 "Please move the arm manually closer to the target position to avoid collision issues.", bad_index, arm_name.c_str(), max_delta_angle);
				std::cout << "Current arm state: ";
				for ( size_t j=0; j<cur_state.size(); ++j )
					std::cout << cur_state[j] << (j<cur_state.size()-1 ? "\t" : "\n");
				std::cout << "Target arm state: ";
				for ( size_t j=0; j<arm_configuration.size(); ++j )
					std::cout << arm_configuration[j] << (j<arm_configuration.size()-1 ? "\t" : "\n");
			}

			return MOV_ERR_SOFT;
		}
	}

	calibration_interface_->assignNewArmJoints(arm_name, new_joint_config);

	// wait for arm to arrive at goal
	const double start_time = time_utilities::getSystemTimeSec();
	while ( time_utilities::getTimeElapsedSec(start_time) < 10.f ) //Max. 10 seconds to reach goal
	{
		ros::Rate(20).sleep(); // No need to iterate through this every tick
		ros::spinOnce();
		cur_state = *calibration_interface_->getCurrentArmState(arm_name);

		double norm = 0.f;
		for (int i = 0; i<cur_state.size(); ++i)
		{
			double error = arm_configuration[i]-cur_state[i];
			norm += error*error;  // sum over squared errors
		}
		norm = std::sqrt(norm);  // take root

		if ( norm < 0.025 ) //Close enough to goal configuration
		{
			std::cout << "Arm configuration reached, deviation: " << norm << std::endl;
			break;
		}
	}

	if ( time_utilities::getTimeElapsedSec(start_time) >= 10.f )
	{
		ROS_WARN("Could not reach following arm configuration in time:");
		for (int i = 0; i<arm_configuration.size(); ++i)
			std::cout << arm_configuration[i] << "\t";
		std::cout << std::endl;

		return MOV_ERR_SOFT;
	}

	return error_code;
}

std::string CameraArmType::getString()
{
	return "camera_arm";
}
