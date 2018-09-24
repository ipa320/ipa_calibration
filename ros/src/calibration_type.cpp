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
					calibration_interface_(0), initialized_(false), total_configuration_count_(0),
					current_camera_counter_(0), mapped_camera_index_(0), cameras_done_(false)
{

}

void CalibrationType::initialize(ros::NodeHandle nh, IPAInterface* calib_interface)
{
	node_handle_ = nh;
	//transform_listener_ = nh;
	calibration_interface_ = calib_interface;

	std::cout << "\n========== CalibrationType Parameters ==========\n";

	// cameras list format: [camera_name, DoF-count, max_delta_angle]
	std::vector<std::string> cameras_list;
	node_handle_.getParam("cameras_list", cameras_list);
	std::cout << "cameras_list:" << std::endl;
	for ( int i=0; i<cameras_list.size(); i+=3 )
	{
		camera_description cam_desc;
		cam_desc.camera_name_ = cameras_list[i];
		cam_desc.dof_count_ = std::stoi(cameras_list[i+1]);
		cam_desc.max_delta_angle_ = fabs(std::stod(cameras_list[i+2]));

		if ( cam_desc.dof_count_ < 1 )
		{
			ROS_WARN("Invalid DoF count %d for camera %s, skipping camera.", cam_desc.dof_count_, cam_desc.camera_name_.c_str());
			continue;
		}

		cameras_.push_back(cam_desc);
		std::cout << cameras_list[i] << ": DoF " << cameras_list[i+1] << ", max delta angle: " << cameras_list[i+2] << std::endl;
	}

	if ( cameras_.empty() )
	{
		ROS_ERROR("Cameras list empty, check yaml file");
		return;
	}

	// Read in camera data
	for ( int i=0; i<cameras_.size(); ++i )
	{
		std::vector<double> cam_data;  // raw data that will be read in: can either be configs or ranges
		node_handle_.getParam((cameras_[i].camera_name_+"_configs"), cam_data);
		const int camera_dof = cameras_[i].dof_count_;

		if ( !cam_data.empty() )  // configs has been found
		{
			if ( camera_dof == 0 || cam_data.size() % camera_dof != 0 )
			{
				ROS_ERROR("%s_configs vector has wrong size, camera DoF %d", cameras_[i].camera_name_.c_str(), camera_dof);
				return;
			}

			for ( int j=0; j<cam_data.size(); j+=camera_dof )
			{
				std::vector<double> values;
				values.reserve(camera_dof);
				for ( int k=j; k<j+camera_dof; ++k )
					values.push_back(cam_data[k]);

				cameras_[i].configurations_.push_back(values);
			}
		}
		else  // no configs found, search for ranges
		{
			node_handle_.getParam((cameras_[i].camera_name_+"_ranges"), cam_data);

			if ( cam_data.empty() )
			{
				ROS_WARN("Neither configs nor ranges data has found for camera %s, removing camera.", cameras_[i].camera_name_.c_str());
				cameras_.erase(cameras_.begin()+i);
				--i;
				continue;
			}

			if ( cam_data.size() % 3 != 0 || cam_data.size() != 3*camera_dof )
			{
				ROS_ERROR("%s_ranges vector has the wrong size, each DoF needs three entries (start,step,stop), camera DoF: %d", cameras_[i].camera_name_.c_str(), cameras_[i].dof_count_);
				std::cout << "size: " << cam_data.size() << ", needed: " << (3*camera_dof) << std::endl;
				return;
			}

			std::vector< std::vector<double> > cam_ranges;
			for ( int i=0; i<camera_dof; i++ )
			{
				std::vector<double> range;
				for ( int j=0; j<3; j++ )
				{
					range.push_back(cam_data[3*i + j]);
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

			std::cout << "Generated " << (int)cameras_[i].configurations_.size() << " configurations for " << cameras_[i].camera_name_ <<  "." << std::endl;

			if ( cameras_[i].configurations_.empty() )
			{
				ROS_WARN("No configurations has been generated for camera %s, removing camera.", cameras_[i].camera_name_.c_str());
				cameras_.erase(cameras_.begin()+i);
				--i;
			}
		}
	}

	for ( int i=0; i<cameras_.size(); ++i )  // compute total configuration count (only for cameras here)
		total_configuration_count_ += cameras_[i].configurations_.size();

	if ( total_configuration_count_ <= 0 )
	{
		ROS_WARN("Invalid total configuration count for cameras: %d", total_configuration_count_);
		return;
	}

	// Display camera configurations
	for ( int i=0; i<cameras_.size(); ++i )
	{
		std::cout << cameras_[i].camera_name_ << " configurations:" << std::endl;
		for ( int j=0; j<cameras_[i].configurations_.size(); ++j )
		{
			for ( int k=0; k<cameras_[i].configurations_[j].size(); ++k )
				std::cout << cameras_[i].configurations_[j][k] << "\t";
			std::cout << std::endl;
		}
		std::cout << std::endl;
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

	if ( cameras_done_ )  // reset flag after it was set the last call
		cameras_done_ = false;

	int current_count = cameras_[current_camera_counter_].configurations_.size();  // count of current configurations of current camera

	if ( config_index > 0 && mapped_camera_index_ % current_count == 0 )  // cameras move one after an other
	{
		current_camera_counter_ = ( (current_camera_counter_+1 >= cameras_.size()) ? 0 : current_camera_counter_+1 );  // when previous camera has iterated through all its configs, do same with next camera
		mapped_camera_index_ = 0;

		if ( current_camera_counter_ == 0 )  // set for one call, reset on next call
			cameras_done_ = true;
	}

	for ( short i=0; i<NUM_MOVE_TRIES; ++i )
	{
		unsigned short error_code = moveCamera(cameras_[current_camera_counter_], cameras_[current_camera_counter_].configurations_[mapped_camera_index_]);

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
				ROS_WARN("CalibrationType::moveRobot: Skipping camera configuration %d of %s.", mapped_camera_index_, cameras_[current_camera_counter_].camera_name_.c_str());
		}
		else
		{
			ROS_FATAL("CalibrationType::moveRobot: Exiting calibration.");
			throw std::exception();
		}
	}

	++mapped_camera_index_;
	return true;
}

unsigned short CalibrationType::moveCamera(const camera_description &camera, const std::vector<double> &cam_configuration)
{
	std_msgs::Float64MultiArray angles;
	angles.data.resize(cam_configuration.size());

	unsigned short error_code = MOV_NO_ERR;
	const std::string &camera_name = camera.camera_name_;

	for ( int i=0; i<angles.data.size(); ++i )
		angles.data[i] = cam_configuration[i];

	ros::spinOnce();
	std::vector<double> cur_state = *calibration_interface_->getCurrentCameraState(camera_name);
	if ( cur_state.size() != cam_configuration.size() )
	{
		ROS_ERROR("Size of target camera configuration and count of camera joints do not match! Please adjust the yaml file.");
		return MOV_ERR_FATAL;
	}

	// check whether each joint passes the max delta angle test
	const double &max_delta_angle = camera.max_delta_angle_;
	if ( max_delta_angle > 0.f )
	{
		int bad_index = -1;
		if ( checkForMaxDeltaAngle(cur_state, cam_configuration, max_delta_angle, bad_index) )
		{
			if ( bad_index == -1 )
			{
				ROS_ERROR("Size of target camera configuration and count of camera joints do not match! Please adjust the yaml file.");
				return MOV_ERR_FATAL;
			}
			else
			{
				ROS_WARN("Angle number %d in target configuration of camera %s exceeds max allowed deviation %f!\n"
						 "Please move the camera manually closer to the target position to avoid collision issues.", bad_index, camera_name.c_str(), max_delta_angle);
				std::cout << "Current camera state: ";
				for ( size_t j=0; j<cur_state.size(); ++j )
					std::cout << cur_state[j] << (j<cur_state.size()-1 ? "\t" : "\n");
				std::cout << "Target camera state: ";
				for ( size_t j=0; j<cam_configuration.size(); ++j )
					std::cout << cam_configuration[j] << (j<cam_configuration.size()-1 ? "\t" : "\n");
			}

			return MOV_ERR_SOFT;
		}
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
				std::cout << camera_name << " configuration reached, deviation: " << length << std::endl;
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

	return error_code;
}

int CalibrationType::getConfigurationCount()
{
	return total_configuration_count_;
}

void CalibrationType::getUncertainties(std::vector<std::string> &uncertainties_list)
{
	uncertainties_list = uncertainties_list_;
}

// Create configuration vector out of several parameters (stored in param_vector) by pairing every parameter value with every other parameter value (like a grid)
// E.g. param_1={f}, param_2={d,e}, param_3={a,b,c}  <-- each stored in param_vector
// Resulting configs:
// (param_1  param_2  param_3)
//  f        d        a
//  f        d        b
//  f        d        c
//  f        e        a
//  f        e        b
//  f        e        c
bool CalibrationType::generateConfigs(const std::vector< std::vector<double> > &param_vector, std::vector< std::vector<double> > &configs)
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

	// preallocate memory
	std::vector<double> temp;
	temp.resize(num_params);  // filled with zeros
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

bool CalibrationType::checkForMaxDeltaAngle(const std::vector<double> &state, const std::vector<double> &target, const double max_angle, int &bad_idx)
{
	if ( state.size() != target.size() )  // should never happen, but just to make this safer
		return false;

	for ( int i=0; i<state.size(); ++i )
	{
		double delta_angle = target[i] - state[i];

		while (delta_angle < -CV_PI)
			delta_angle += 2*CV_PI;
		while (delta_angle > CV_PI)
			delta_angle -= 2*CV_PI;

		if ( fabs(delta_angle) > max_angle )
		{
			bad_idx = i;
			return false;
		}
	}

	return true;
}
