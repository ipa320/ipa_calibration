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
#include <robotino_calibration/time_utilities.h>
#include <robotino_calibration/transformation_utilities.h>
#include <numeric>  // std::inner_product
#include <std_msgs/Float64MultiArray.h>


CalibrationType::CalibrationType() :
					calibration_interface_(0), initialized_(false), configuration_count_(0), cameras_count_(0)
{

}

void CalibrationType::initialize(ros::NodeHandle nh, IPAInterface* calib_interface)
{
	node_handle_ = nh;
	//transform_listener_ = nh;
	calibration_interface_ = calib_interface;

	std::cout << "\n========== CalibrationType Parameters ==========\n";

	// cameras list format: [camera_name, DoF-count]
	std::vector<std::string> cameras_list;
	node_handle_.getParam("cameras_list", cameras_list);
	std::cout << "cameras_list:" << std::endl;
	for ( int i=0; i<cameras_list.size(); i+=2 )
	{
		camera_description cam_desc;
		cam_desc.camera_name_ = cameras_list[i];
		cam_desc.dof_count_ = std::stoi(cameras_list[i+1]);
		cameras_.push_back(cam_desc);
		std::cout << cameras_list[i] << ": DOF " << cameras_list[i+1] << std::endl;
	}

	bool use_range = false;
	node_handle_.param("use_range", use_range, false);
	std::cout << "use_range: " << use_range << std::endl;

	if ( cameras_.size() <= 0 )
	{
		ROS_WARN("Invalid cameras size: %d", cameras_.size());
		return;
	}

	const int cameras_count = cameras_.size();

	if ( use_range )
	{
		for ( int i=0; i<cameras_count; ++i )
		{
			std::vector<double> cam_range;
			node_handle_.getParam("camera_ranges_"+std::to_string(i+1), cam_range);
			const int camera_dof = cameras_[i].dof_count_;
			if ( cam_range.size() % 3 != 0 || cam_range.size() != 3*camera_dof )
			{
				ROS_ERROR("The camera range vector has the wrong size, each DOF needs three entries (start,step,stop)");
				std::cout << "size: " << cam_range.size() << ", needed: " << (3*camera_dof) << std::endl;
				return;
			}

			std::vector< std::vector<double> > cam_ranges;
			for ( int i=0; i<camera_dof; i++ )
			{
				std::vector<double> range;
				for ( int j=0; j<3; j++ )
				{
					range.push_back(cam_range[3*i + j]);
				}
				cam_ranges.push_back(range);
			}

			for ( int i=0; i<camera_dof; ++i )
			{
				if ( cam_ranges[i][0] == cam_ranges[i][2] || cam_ranges[i][1] == 0. )  // fix invalid entries (infinite loop)
					cam_ranges[i][1] = 1.0;
			}

			std::vector< std::vector<double> > param_vector;  // Contains all possible values of all camera joints
			param_vector.reserve(camera_dof);
			for ( int i=0; i<camera_dof; ++i )
			{
				std::vector<double> values;  // temporary container that stores all values of currently processed parameter
				for ( double value=cam_ranges[i][0]; value<=cam_ranges[i][2]; value+=cam_ranges[i][1] )
					values.push_back(value);
				param_vector.push_back(values);
			}

			if ( !generateConfigs(param_vector, cameras_[i].configurations_) )
				return;
		}
	}
	else
	{
		for ( int i=0; i<cameras_count; ++i )  // go through all cameras and save their respective configurations that have been entered by the user manually
		{
			std::vector<double> cam_configs;
			node_handle_.getParam("camera_configs_"+std::to_string(i+1), cam_configs);
			const int camera_dof = cameras_[i].dof_count_;

			if ( cam_configs.size() % camera_dof != 0 )
			{
				ROS_ERROR("camera_configs_%d for %s vector has the wrong size, camera dof %d", (i+1), cameras_[i].camera_name_.c_str(), camera_dof);
				return;
			}

			for ( int j=0; j<cam_configs.size(); j+=camera_dof )
			{
				std::vector<double> values;
				values.reserve(camera_dof);
				for ( int k=j; k<j+camera_dof; ++k )
					values.push_back(cam_configs[k]);

				cameras_[i].configurations_.push_back(values);
			}
		}
	}

	for ( int i=0; i<cameras_.size(); ++i )
	{
		if ( configuration_count_ == 0 )
			configuration_count_ = cameras_[i].configurations_.size();
		else
			configuration_count_ *= cameras_[i].configurations_.size();
	}

	if ( configuration_count_ <= 0 )
	{
		ROS_WARN("Invalid configuration_count for cameras: %d", configuration_count_);
		return;
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

	for ( int i=0; i<cameras_.size(); ++i )
	{
		for ( int j=0; j<cameras_[i].configurations_.size(); ++j )
		{
			if ( cameras_[i].configurations_[j].order_index_ == config_index )
			{
				bool success = moveCamera(cameras_[i].camera_name_, cameras_[i].configurations_[j].config_);

				if ( !success )
					ROS_WARN("CalibrationType::moveRobot - Could not move camera %s at index %d", cameras_[i].camera_name_.c_str(), config_index);
			}
		}
	}

	return true;
}

bool CalibrationType::moveCamera(const std::string &camera_name, const std::vector<double> &cam_configuration)
{
	std_msgs::Float64MultiArray angles;
	angles.data.resize(cam_configuration.size());

	for ( int i=0; i<angles.data.size(); ++i )
		angles.data[i] = cam_configuration[i];

	std::vector<double> cur_state = *calibration_interface_->getCurrentCameraState(camera_name);
	if ( cur_state.size() != cam_configuration.size() )
	{
		ROS_ERROR("Size of target camera configuration and count of camera joints do not match! Please adjust the yaml file.");
		return false;
	}

	calibration_interface_->assignNewCameraAngles(camera_name, angles);

	// wait for pan tilt to arrive at goal position
	if ( cur_state.size() > 0 )
	{
		const double start_time = time_utilities::getSystemTimeSec();
		while ( time_utilities::getTimeElapsedSec(start_time) < 10.f )
		{
			ros::Rate(20).sleep(); // No need to iterate through this every tick
			cur_state = *calibration_interface_->getCurrentCameraState(camera_name);
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

		if ( time_utilities::getTimeElapsedSec(start_time) >= 10.f )
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
	return configuration_count_;
}

void CalibrationType::getUncertainties(std::vector<std::string> &uncertainties_list)
{
	uncertainties_list = uncertainties_list_;
}

// Create robot_configurations grid. Do this by pairing every parameter value with every other parameter value
// E.g. param_1={f}, param_2={d,e}, param_3={a,b,c}  <-- stored in param_vector
// Resulting configs:
// (param_1  param_2  param_3)
//  f        d        a
//  f        d        b
//  f        d        c
//  f        e        a
//  f        e        b
//  f        e        c
bool generateConfigs(const std::vector< std::vector<double> > &param_vector, std::vector< std::vector<double> > &configs)
{
	int num_configs = 0;
	const int num_params = param_vector.size();
	for ( int i=0; i<num_params; ++i )
	{
		if ( num_configs == 0 )
			num_configs = param_vector[i].size();
		else
			num_configs *= param_vector[i].size();
	}

	if ( num_configs == 0 || num_params == 0 )
	{
		ROS_ERROR("No configuration can be generated! Num. configs: %d, num. params per config: %d", num_configs, num_params);
		return false;
	}

	std::vector<double> temp;
	temp.resize(num_params);
	configs.resize(num_configs, temp);

	int repetition_pattern = 0; // Describes how often a value needs to be repeated, look at example param_2, d and e are there three times
	for ( int i=num_params-1; i>=0; --i ) // Fill robot_configurations_ starting from last parameter in param_vector
	{
		int counter = 0;
		for ( int j=0; j<num_configs; ++j )
		{
			if ( repetition_pattern == 0 ) // executed initially
				counter = j % param_vector[i].size(); // repeat parameters over and over again
			else if ( j > 0 && (j % repetition_pattern == 0) )
				counter = (counter+1) % param_vector[i].size();

			configs[j][i] = param_vector[i][counter];
		}

		if ( repetition_pattern == 0 )
			repetition_pattern = param_vector[i].size();
		else
			repetition_pattern *= param_vector[i].size();
	}

	return true;
}
