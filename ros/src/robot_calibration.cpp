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


#include <ros/ros.h>
#include <robotino_calibration/robot_calibration.h>
#include <boost/filesystem.hpp>
#include <exception>
#include <robotino_calibration/timer.h>
#include <robotino_calibration/transformation_utilities.h>

//Exception
#include <tf/exceptions.h>


RobotCalibration::RobotCalibration(ros::NodeHandle nh, CalibrationInterface* interface) :
		node_handle_(nh), transform_listener_(nh), calibrated_(false), calibration_interface_(interface)
{
	// load parameters
	std::cout << "\n========== Calibration Parameters ==========\n";
	node_handle_.param<std::string>("base_frame", base_frame_, "");
	std::cout << "base_frame: " << base_frame_ << std::endl;
	node_handle_.param<std::string>("calibration_storage_path", calibration_storage_path_, "/calibration");
	std::cout << "calibration_storage_path: " << calibration_storage_path_ << std::endl;

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
		bool success = transform_utilities::getTransform(transform_listener_, tmp.parent_, tmp.child_, tmp.current_trafo_);

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

	//calibration_interface_ = CalibrationInterface::createInterfaceByID(calibration_ID_, node_handle_, do_arm_calibration);
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
