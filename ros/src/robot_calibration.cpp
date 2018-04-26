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
 * Date of creation: October 2016
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


#include <robotino_calibration/robot_calibration.h>
#include <robotino_calibration/timer.h>
#include <robotino_calibration/transformation_utilities.h>
#include <std_msgs/Float64MultiArray.h>

//Exception
#include <exception>
#include <tf/exceptions.h>

// File writing
#include <sstream>
#include <fstream>

// Boost
#include <boost/filesystem.hpp>


RobotCalibration::RobotCalibration(ros::NodeHandle nh, CalibrationInterface* interface) :
	node_handle_(nh), transform_listener_(nh), calibrated_(false), calibration_interface_(interface)
{
	// load parameters
	std::cout << "\n========== RobotCalibration Parameters ==========\n";

	// hack to fix tf::waitForTransform throwing error that transforms do not exist when now() == 0 at startup
	ROS_INFO("Waiting for TF listener to initialize...");
	Timer timeout;
	while ( timeout.getElapsedTimeInSec() < 10.f )
	{
		if ( ros::Time::now().isValid() )
			break;

		ros::Duration(0.2f).sleep();
	}
	ROS_INFO("End waiting for TF to initialize.");

	node_handle_.param<std::string>("camera_optical_frame", camera_optical_frame_, "");
	std::cout << "camera_optical_frame: " << camera_optical_frame_ << std::endl;
	node_handle_.param<std::string>("calibration_storage_path", calibration_storage_path_, "/calibration");
	std::cout << "calibration_storage_path: " << calibration_storage_path_ << std::endl;

	node_handle_.param("camera_dof", camera_dof_, 0);
	std::cout << "camera_dof: " << camera_dof_ << std::endl;
	if ( camera_dof_ < 1 )
	{
		std::cout << "Error: Invalid camera_dof: " << camera_dof_ << ". Setting camera_dof to 1." << std::endl;
		camera_dof_ = 1;
	}

	// load gaps including its initial values
	std::vector<std::string> uncertainties_list;
	node_handle_.getParam("uncertainties_list", uncertainties_list);

	if ( uncertainties_list.size() % 2 != 0 )
		ROS_WARN("Size of uncertainsties_list is not a factor of two.");

	for ( int i=0; i<uncertainties_list.size(); i+=2 )
	{
		CalibrationInfo tmp;
		tmp.parent_ = uncertainties_list[i];
		tmp.child_ = uncertainties_list[i+1];
		tmp.trafo_until_next_gap_idx_ = -1;
		bool success = transform_utilities::getTransform(transform_listener_, tmp.child_, tmp.parent_, tmp.current_trafo_);

		if ( success == false )
		{
			ROS_FATAL("Could not retrieve transform from %s to %s from TF!", tmp.parent_.c_str(), tmp.child_.c_str());
			throw std::exception();
		}

		transforms_to_calibrate_.push_back(tmp);
	}

	node_handle_.param("optimization_iterations", optimization_iterations_, 1000);

	if ( transforms_to_calibrate_.size() == 1 )
	{
		std::cout << "Only one transform to calibrate: Setting optimization_iterations to 1." << std::endl;
		optimization_iterations_ = 1;
	}
	std::cout << "optimization_iterations: " << optimization_iterations_ << std::endl;

	node_handle_.getParam("calibration_order", calibration_order_);
	if ( calibration_order_.size() != transforms_to_calibrate_.size() )
	{
		ROS_FATAL("Size of calibration_order and gaps inside uncertainties_list do not match!");
		throw std::exception();
	}

	std::cout << "calibration order:" << std::endl;
	for ( int i=0; i<calibration_order_.size(); ++i )
	{
		if ( calibration_order_[i] < 1 || calibration_order_[i] > transforms_to_calibrate_.size() )
		{
			ROS_FATAL("Invalid index in calibration order %d", calibration_order_[i]);
			throw std::exception();
		}
		else
		{
			calibration_order_[i] = calibration_order_[i]-1; // zero-indexed values from now on
			std::cout << (i+1) << ". From " << transforms_to_calibrate_[calibration_order_[i]].parent_ << " to " << transforms_to_calibrate_[calibration_order_[i]].child_ << std::endl;
			std::cout << "Initial transform: " << transforms_to_calibrate_[calibration_order_[i]].current_trafo_ << std::endl;
		}
	}

	createStorageFolder();

	if (calibration_interface_ == 0) // Throw exception, as we need an calibration interface in order to function properly!
	{
		ROS_FATAL("Could not create a calibration interface.");
		throw std::exception();
	}
}

RobotCalibration::~RobotCalibration()
{
	if ( calibration_interface_ != 0 )
		delete calibration_interface_;
}

// create data storage path if it does not yet exist
void RobotCalibration::createStorageFolder()
{
	boost::filesystem::path storage_path(calibration_storage_path_);

	if (boost::filesystem::exists(storage_path) == false)
	{
		if (boost::filesystem::create_directories(storage_path) == false && boost::filesystem::exists(storage_path) == false)
		{
			std::cout << "Error: RobotCalibration: Could not create directory " << storage_path << std::endl;
			return;
		}
	}
}

void RobotCalibration::displayAndSaveCalibrationResult(std::string output_file_name)
{
	std::stringstream output;

	output << "\n\n\n----- Replace the follwing parameters within the urdf file of your robot ----- \n\n";
	for ( int i=0; i<transforms_to_calibrate_.size(); ++i )
	{
		cv::Vec3d ypr = transform_utilities::YPRFromRotationMatrix(transforms_to_calibrate_[i].current_trafo_);

		output << "<!-- " << transforms_to_calibrate_[i].child_ << " mount positions | camera_base_calibration | relative to " << transforms_to_calibrate_[i].parent_ << "-->\n"
			   << "  <property name=\"" << transforms_to_calibrate_[i].child_ << "_x\" value=\"" << transforms_to_calibrate_[i].current_trafo_.at<double>(0,3) << "\"/>\n"
			   << "  <property name=\"" << transforms_to_calibrate_[i].child_ << "_y\" value=\"" << transforms_to_calibrate_[i].current_trafo_.at<double>(1,3) << "\"/>\n"
			   << "  <property name=\"" << transforms_to_calibrate_[i].child_ << "_z\" value=\"" << transforms_to_calibrate_[i].current_trafo_.at<double>(2,3) << "\"/>\n"
			   << "  <property name=\"" << transforms_to_calibrate_[i].child_ << "_roll\" value=\"" << ypr.val[2] << "\"/>\n"
			   << "  <property name=\"" << transforms_to_calibrate_[i].child_ << "_pitch\" value=\"" << ypr.val[1] << "\"/>\n"
			   << "  <property name=\"" << transforms_to_calibrate_[i].child_ << "_yaw\" value=\"" << ypr.val[0] << "\"/>\n\n";
	}

	std::cout << output.str();

	if ( ros::ok() )
	{
		std::string path_file = calibration_storage_path_ + output_file_name;
		std::fstream file_output;
		file_output.open(path_file.c_str(), std::ios::out);
		if (file_output.is_open())
			file_output << output.str();
		file_output.close();
	}
	else
	{
		ROS_INFO("Skipping to save calibration result.");
	}
}

void RobotCalibration::extrinsicCalibration(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
											std::vector<cv::Mat>& T_gapfirst_to_marker_vector, std::vector< std::vector<cv::Mat> > T_between_gaps_vector,
											std::vector<cv::Mat>& T_gaplast_to_marker_vector, int trafo_to_calibrate)
{
	// transform 3d marker points to respective coordinates systems (camera and torso_upper)
	std::vector<cv::Point3d> points_3d_child, points_3d_parent;
	for (size_t i=0; i<pattern_points_3d.size(); ++i)
	{
		cv::Mat T_child_to_marker;

		// Iterate over uncertain trafos, add in between trafos as well
		// Forwards in chain from child frame on
		for ( int j=trafo_to_calibrate; j<transforms_to_calibrate_.size()-1; ++j )
		{
			if ( transforms_to_calibrate_[j].trafo_until_next_gap_idx_ > -1 )
				T_child_to_marker *= T_between_gaps_vector[i][transforms_to_calibrate_[j].trafo_until_next_gap_idx_];

			T_child_to_marker *= transforms_to_calibrate_[j+1].current_trafo_;
		}
		T_child_to_marker *= T_gaplast_to_marker_vector[i];

		cv::Mat T_parent_to_marker;
		// Backwards in chain from parent frame on
		for ( int j=trafo_to_calibrate-1; j>=0; --j )
		{
			if ( transforms_to_calibrate_[j].trafo_until_next_gap_idx_ > -1 )
				T_parent_to_marker *= T_between_gaps_vector[i][transforms_to_calibrate_[j].trafo_until_next_gap_idx_].inv();

			T_parent_to_marker *= transforms_to_calibrate_[j].current_trafo_.inv();
		}
		T_parent_to_marker *= T_gapfirst_to_marker_vector[i];


		for (size_t j=0; j<pattern_points_3d[i].size(); ++j)
		{
			cv::Mat point = cv::Mat(cv::Vec4d(pattern_points_3d[i][j].x, pattern_points_3d[i][j].y, pattern_points_3d[i][j].z, 1.0));

			// to child coordinate system
			cv::Mat point_child = T_child_to_marker * point;
			points_3d_child.push_back(cv::Point3d(point_child.at<double>(0), point_child.at<double>(1), point_child.at<double>(2)));

			// to parent coordinate system
			cv::Mat point_parent = T_parent_to_marker * point;
			points_3d_parent.push_back(cv::Point3d(point_parent.at<double>(0), point_parent.at<double>(1), point_parent.at<double>(2)));
		}
	}
}

bool RobotCalibration::calculateTransformationChains(cv::Mat& T_gapfirst_to_marker, std::vector<cv::Mat> T_between_gaps,
													 cv::Mat& T_gaplast_to_camera_optical, std::string marker_frame)
{
	bool result = true;
	result &= transform_utilities::getTransform(transform_listener_, marker_frame, transforms_to_calibrate_[0].parent_, T_gapfirst_to_marker);
	result &= transform_utilities::getTransform(transform_listener_, camera_optical_frame_, transforms_to_calibrate_[ transforms_to_calibrate_.size()-1 ].child_, T_gaplast_to_camera_optical);

	for ( int i=0; i<transforms_to_calibrate_.size()-1; ++i )
	{
		if ( transforms_to_calibrate_[i].parent_ == transforms_to_calibrate_[i].child_ ) // several gaps in a row, no certain trafos in between
			continue;

		cv::Mat temp;
		result &= transform_utilities::getTransform(transform_listener_, transforms_to_calibrate_[i+1].parent_, transforms_to_calibrate_[i].child_, temp);
		T_between_gaps.push_back(temp);
		transforms_to_calibrate_[i].trafo_until_next_gap_idx_ = T_between_gaps.size()-1;
	}

	return result;
}

void RobotCalibration::moveRobot(int config_index)
{
	moveCamera(camera_configurations_[config_index]);
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

bool RobotCalibration::moveCamera(const std::vector<double> &cam_configuration)
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

		if ( timeout.getElapsedTimeInSec()>=10.0 )
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
