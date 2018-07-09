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

#include <calibration_interface/calibration_type.h>
#include <calibration_interface/ipa_interface.h>
#include <robotino_calibration/timer.h>
#include <robotino_calibration/transformation_utilities.h>
#include <numeric>  // std::inner_product
#include <std_msgs/Float64MultiArray.h>


CalibrationType::CalibrationType() :
					calibration_interface_(0), initialized_(false), camera_dof_(0)
{

}

void CalibrationType::initialize(ros::NodeHandle nh, IPAInterface* calib_interface)
{
	node_handle_ = nh;
	//transform_listener_ = nh;
	calibration_interface_ = calib_interface;

	std::cout << "\n========== CalibrationType Parameters ==========\n";

	node_handle_.param("camera_dof", camera_dof_, 0);
	std::cout << "camera_dof: " << camera_dof_ << std::endl;
	if ( camera_dof_ < 1 )
	{
		std::cout << "Error: Invalid camera_dof: " << camera_dof_ << ". Setting camera_dof to 1." << std::endl;
		camera_dof_ = 1;
	}

	node_handle_.getParam("uncertainties_list", uncertainties_list_);

	if ( uncertainties_list_.empty() )
	{
		ROS_WARN("Uncertainties list is empty... returning uninitialized.");
		return;
	}

	if ( uncertainties_list_.size() % 6 != 0 )
		ROS_WARN("Size of uncertainsties_list is not a factor of 6: [parent frame, child frame, last parent-branch frame, last child-branch frame, parent marker, child marker]");

	std::cout << "uncertainties list: " << std::endl;
	for ( int i=0; i<uncertainties_list_.size(); i+=6 )
	{
		std::cout << (int(i/4)+1) << ". Uncertainty: From " << uncertainties_list_[i] << " [over " << uncertainties_list_[i+2] << " to parent marker: " << uncertainties_list_[i+4]
				<< "] to " << uncertainties_list_[i+1] << " [over " << uncertainties_list_[i+3] << " to child marker: " << uncertainties_list_[i+5] << "]" << std::endl;
	}

	initialized_ = true;
}

CalibrationType::~CalibrationType()
{
}

bool CalibrationType::moveRobot(int config_index)
{
	if ( !initialized_ )
	{
		ROS_WARN("CalibrationType::moveRobot - Not inizialized yet, no movement allowed!");
		return false;
	}

	return moveCamera(camera_configurations_[config_index]);
	// Does not make too much sense here for now, as it only returns false in case of an dimension error
	/*for ( short i=0; i<NUM_MOVE_TRIES; ++i )
	{
		if ( !moveCamera(camera_configurations_[config_index]) )
		{
			ROS_ERROR("RobotCalibration::moveRobot: Could not execute moveCamera, (%d/%d) tries.", i+1, NUM_MOVE_TRIES);
			if ( i<NUM_MOVE_TRIES-1 )
			{
				ROS_INFO("RobotCalibration::moveRobot: Trying again in 1 sec.");
				ros::Duration(1.f).sleep();
			}
			else
				ROS_WARN("RobotCalibration::moveRobot: Skipping camera configuration %d.", config_index);
		}
		else
			break;
	}*/
}

bool CalibrationType::moveCamera(const std::vector<double> &cam_configuration)
{
	std_msgs::Float64MultiArray angles;
	angles.data.resize(cam_configuration.size());

	for ( int i=0; i<angles.data.size(); ++i )
		angles.data[i] = cam_configuration[i];

	std::vector<double> cur_state = *calibration_interface_->getCurrentCameraState();
	if ( cur_state.size() != cam_configuration.size() )
	{
		ROS_ERROR("Size of target camera configuration and count of camera joints do not match! Please adjust the yaml file.");
		return false;
	}

	calibration_interface_->assignNewCameraAngles(angles);

	// wait for pan tilt to arrive at goal position
	if ( cur_state.size() > 0 )
	{
		Timer timeout;
		while (timeout.getElapsedTimeInSec()<10.0)
		{
			ros::Rate(20).sleep(); // No need to iterate through this every tick
			cur_state = *calibration_interface_->getCurrentCameraState();
			std::vector<double> difference(cur_state.size());
			for (int i = 0; i<cur_state.size(); ++i)
				difference[i] = cam_configuration[i]-cur_state[i];

			double length = std::sqrt(std::inner_product(difference.begin(), difference.end(), difference.begin(), 0.0)); //Length of difference vector in joint space

			if ( length < 0.02 ) //Close enough to goal configuration
			{
				std::cout << "Camera configuration reached, deviation: " << length << std::endl;
				break;
			}

			ros::spinOnce();
		}

		if ( timeout.getElapsedTimeInSec() >= 10.0 )
		{
			ROS_WARN("Could not reach following camera configuration in time:");
			for (int i = 0; i<cam_configuration.size(); ++i)
				std::cout << cam_configuration[i] << "\t";
			std::cout << std::endl;
		}
	}
	else
	{
		ros::Duration(1).sleep();
	}

	ros::spinOnce();
	return true;
}

int CalibrationType::getConfigurationCount()
{
	return camera_configurations_.size();
}

void CalibrationType::getUncertainties(std::vector<std::string> &uncertainties_list)
{
	uncertainties_list = uncertainties_list_;
}
