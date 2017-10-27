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

//Exception
#include <tf/exceptions.h>


RobotCalibration::RobotCalibration(ros::NodeHandle nh, bool do_arm_calibration) :
		node_handle_(nh), transform_listener_(nh), calibrated_(false)
{
	// load parameters
	std::cout << "\n========== Calibration Parameters ==========\n";
	node_handle_.param<std::string>("base_frame", base_frame_, "base_link");
	std::cout << "base_frame: " << base_frame_ << std::endl;
	node_handle_.param<std::string>("calibration_storage_path", calibration_storage_path_, "/robotino_calibration/calibration");
	std::cout << "calibration_storage_path: " << calibration_storage_path_ << std::endl;
	node_handle_.param("calibration_ID", calibration_ID_, 0);
	std::cout << "calibration_ID: " << calibration_ID_ << std::endl;

	calibration_interface_ = CalibrationInterface::createInterfaceByID(calibration_ID_, node_handle_, do_arm_calibration);
	createStorageFolder();

	if (calibration_interface_ == 0) // Throw exception, as we need an calibration interface in order to function properly!
	{
		ROS_FATAL("Could not create a calibration interface for calibration_ID: %d", calibration_ID_);
		throw std::exception();
	}

	// Check whether relative_localization has initialized the reference frame yet.
	// Do not let the robot start driving when the reference frame has not been set up properly! Bad things could happen!
	if (do_arm_calibration==false) //During arm calibration the robot does not drive and there is no child frame available.
	{
		node_handle_.param<std::string>("child_frame_name", child_frame_name_, "/landmark_reference_nav");
		std::cout << "child_frame_name: " << child_frame_name_ << std::endl;

		Timer timeout;
		while ( timeout.getElapsedTimeInSec() < 10.f )
		{
			try
			{
				bool result = transform_listener_.waitForTransform(base_frame_, child_frame_name_, ros::Time(0), ros::Duration(1.f));
				if (result) // Everything is fine, return
					return;
			}
			catch (tf::TransformException& ex)
			{
				ROS_WARN("%s", ex.what());
				// Continue with loop and try again
			}

			ros::Duration(0.1f).sleep(); //Wait for child_frame transform to register properly
		}

		//Failed to set up child frame, exit
		ROS_FATAL("RobotCalibration::RobotCalibration: Reference frame has not been set up for 10 seconds.");
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

void RobotCalibration::setCalibrationStatus(bool calibrated)
{
		calibrated_ = calibrated;
}



