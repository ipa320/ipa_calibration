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


#include <calibration_interface/camera_laserscanner_type.h>
#include <calibration_interface/ipa_interface.h>
#include <robotino_calibration/transformation_utilities.h>
#include <geometry_msgs/Twist.h>


CameraLaserscannerType::CameraLaserscannerType() :
		last_ref_history_update_(0.0), start_error_x_(0.0), start_error_y_(0.0), start_error_phi_(0.0), ref_history_index_(0)
{

}

CameraLaserscannerType::~CameraLaserscannerType()
{

}

void CameraLaserscannerType::initialize(ros::NodeHandle nh, IPAInterface* calib_interface)
{
	CalibrationType::initialize(nh, calib_interface);  // call parent

	std::cout << "\n========== Camera-Laserscanner Parameters ==========\n";

	// coordinate frame name parameters
	node_handle_.param<std::string>("base_frame", base_frame_, "");
	std::cout << "base_frame: " << base_frame_ << std::endl;
	node_handle_.param<std::string>("reference_frame", reference_frame_, "");
	std::cout << "reference_frame: " << reference_frame_ << std::endl;

	node_handle_.param("max_ref_frame_distance", max_ref_frame_distance_, 1.0);
	std::cout << "max_ref_frame_distance: " << max_ref_frame_distance_ << std::endl;

	bool use_range = false;
	node_handle_.param("use_range", use_range, false);
	std::cout << "use_range: " << use_range << std::endl;

	const int base_dof = NUM_BASE_PARAMS; // coming from calibration_utilities.h
	if (use_range == true)
	{
		// create robot configurations from regular grid
		std::vector<double> x_range;
		node_handle_.getParam("x_range", x_range);
		std::vector<double> y_range;
		node_handle_.getParam("y_range", y_range);
		std::vector<double> phi_range;
		node_handle_.getParam("phi_range", phi_range);

		if ( x_range.size()!=3 || y_range.size()!=3 || phi_range.size()!=3 )
		{
			ROS_ERROR("One of the position range vectors has wrong size.");
			return;
		}

		std::vector<double> temp;
		node_handle_.getParam("camera_ranges", temp);
		if ( temp.size() % 3 != 0 || temp.size() != 3*camera_dof_ )
		{
			ROS_ERROR("The camera range vector has the wrong size, each DOF needs three entries (start,step,stop)");
			std::cout << "size: " << temp.size() << ", needed: " << (3*camera_dof_) << std::endl;
			return;
		}

		std::vector<std::vector<double>> cam_ranges;
		for ( int i=0; i<camera_dof_; i++ )
		{
			std::vector<double> range;
			for ( int j=0; j<3; j++ )
			{
				range.push_back(temp[3*i + j]);
			}
			cam_ranges.push_back(range);
		}

		if (x_range[0] == x_range[2] || x_range[1] == 0.)		// this sets the step to something bigger than 0
			x_range[1] = 1.0;
		if (y_range[0] == y_range[2] || y_range[1] == 0.)
			y_range[1] = 1.0;
		if (phi_range[0] == phi_range[2] || phi_range[1] == 0.)
			phi_range[1] = 1.0;

		for ( int i=0; i<camera_dof_; ++i )
		{
			if ( cam_ranges[i][0] == cam_ranges[i][2] || cam_ranges[i][1] == 0. )
				cam_ranges[i][1] = 1.0;
		}

		// Build configurations from ranges
		// Create a vector that contains each value list of each parameter that's varied
		std::vector<std::vector<double>> param_vector; // Contains all possible values of all parameters
		std::vector<double> values; // Temporary container that stores all values of currently processed parameter

		// Gather all possible values from user defined ranges
		for (double x=x_range[0]; x<=x_range[2]; x+=x_range[1])
			values.push_back(x);
		param_vector.push_back(values);

		values.clear();
		for (double y=y_range[0]; y<=y_range[2]; y+=y_range[1])
			values.push_back(y);
		param_vector.push_back(values);

		values.clear();
		for (double phi=phi_range[0]; phi<=phi_range[2]; phi+=phi_range[1])
			values.push_back(phi);
		param_vector.push_back(values);

		for ( int i=0; i<camera_dof_; ++i )
		{
			values.clear();
			for ( double value=cam_ranges[i][0]; value<=cam_ranges[i][2]; value+=cam_ranges[i][1] )
				values.push_back(value);
			param_vector.push_back(values);
		}

		// Preallocate memory for base and camera configurations
		// Compute number of configs
		int num_configs = 1;
		const int num_params = param_vector.size();
		for ( int i=0; i<num_params; ++i )
			num_configs *= param_vector[i].size();

		if ( num_configs == 0 || num_params == 0 )
		{
			ROS_ERROR("No base or camera configuration can be generated! Num. configs: %d, num. params per config: %d", num_configs, num_params);
			return;
		}

		// Do actual preallocation
		temp.clear();
		temp.resize(base_dof);
		base_configurations_.resize(num_configs, calibration_utilities::BaseConfiguration(temp));
		temp.clear();
		temp.resize(camera_dof_);
		camera_configurations_.resize(num_configs, temp);
		temp.clear();

		// Create robot_configurations grid. Do this by pairing every parameter value with every other parameter value
		// E.g. param_1={f}, param_2={d,e}, param_3={a,b,c}
		// Resulting robot configurations (separated into two lists, one for camera and one for base):
		// (param_1  param_2  param_3)
		//  f        d        a
		//  f        d        b
		//  f        d        c
		//  f        e        a
		//  f        e        b
		//  f        e        c
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

				// robot_configurations_[j][i] = param_vector[i][counter]; -> is now split into two lists
				if ( i < base_dof )
					//base_configurations_[j][i] = param_vector[i][counter]; // base_configurations_ is no vector anymore for better readability
					base_configurations_[j].assign(i, param_vector[i][counter]);
				else
					camera_configurations_[j][i-base_dof] = param_vector[i][counter];
			}

			if ( repetition_pattern == 0 )
				repetition_pattern = param_vector[i].size();
			else
				repetition_pattern *= param_vector[i].size();
		}

		std::cout << "Generated " << (int)camera_configurations_.size() << " robot configurations for calibration." << std::endl;
		if ( (int)camera_configurations_.size() == 0 )
			ROS_WARN("No robot configurations generated. Please check your ranges in the yaml file.");
	}
	else
	{
		// read out user-defined robot configurations
		std::vector<double> temp;
		node_handle_.getParam("robot_configurations", temp);

		const int num_params = base_dof + camera_dof_;
		const int num_configs = temp.size()/num_params;
		if (temp.size()%num_params != 0 || temp.size() < 3*num_params)
		{
			ROS_ERROR("The robot_configurations vector should contain at least 3 configurations with %d values each.", num_params);
			return;
		}

		for ( int i=0; i<num_configs; ++i )
		{
			std::vector<double> data;
			for ( int j=0; j<base_dof; ++j )
			{
				data.push_back(temp[num_params*i + j]);
			}
			base_configurations_.push_back(calibration_utilities::BaseConfiguration(data));

			data.clear();
			for ( int j=base_dof; j<num_params; ++j ) // camera_dof_ iterations
			{
				data.push_back(temp[num_params*i + j]);
			}
			camera_configurations_.push_back(data);
		}
	}

	// Display configurations
	std::cout << "base configurations:" << std::endl;
	for ( int i=0; i<base_configurations_.size(); ++i )
		std::cout << base_configurations_[i].get() << std::endl;
	std::cout << std::endl << "camera configurations:" << std::endl;
	for ( int i=0; i<camera_configurations_.size(); ++i )
	{
		for ( int j=0; j<camera_configurations_[i].size(); ++j )
			std::cout << camera_configurations_[i][j] << "\t";
		std::cout << std::endl;
	}

	// Check whether relative_localization has initialized the reference frame yet.
	// Do not let the robot start driving when the reference frame has not been set up properly! Bad things could happen!
	Timer timeout;
	bool result = false;

	while ( timeout.getElapsedTimeInSec() < 10.f )
	{
		try
		{
			result = transform_listener_.waitForTransform(base_frame_, reference_frame_, ros::Time(0), ros::Duration(1.f));
			if (result) // Everything is fine, exit loop
			{
				cv::Mat T;
				transform_utilities::getTransform(transform_listener_, base_frame_, reference_frame_, T); // from base frame to reference frame, used to check whether there is an error in detecting the reference frame
				double start_dist = T.at<double>(0,3)*T.at<double>(0,3) + T.at<double>(1,3)*T.at<double>(1,3) + T.at<double>(2,3)*T.at<double>(2,3); // Squared norm is suffice here, no need to take root.
				for ( int i=0; i<REF_FRAME_HISTORY_SIZE; ++i ) // Initialize history array
					ref_frame_history_[i] = start_dist;

				break;
			}
		}
		catch (tf::TransformException& ex)
		{
			ROS_WARN("%s", ex.what());
			// Continue with loop and try again
		}

		ros::Duration(0.1f).sleep(); //Wait for child_frame transform to register properly
	}

	// Failed to set up child frame, exit
	if ( !result )
	{
		ROS_FATAL("CameraBaseCalibrationMarker::CameraBaseCalibrationMarker: Reference frame has not been set up for 10 seconds.");
		throw std::exception();
	}

	// Create a timer that loops and fills the reference frame history vector?
	//ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);
}

bool CameraLaserscannerType::moveRobot(int config_index)
{
	bool result = CalibrationType::moveRobot(config_index);  // call parent

	if ( result )
	{
		for ( short i=0; i<NUM_MOVE_TRIES; ++i )
		{
			unsigned short error_code = moveBase(base_configurations_[config_index]);

			if ( error_code == MOV_NO_ERR ) // Exit loop, as successfully executed move
			{
				std::cout << "Base configuration reached." << std::endl;
				break;
			}
			else if ( error_code == MOV_ERR_SOFT ) // Retry last failed move
			{
				ROS_WARN("CameraBaseCalibrationMarker::moveRobot: Could not execute moveBase, (%d/%d) tries.", i+1, NUM_MOVE_TRIES);
				if ( i<NUM_MOVE_TRIES-1 )
				{
					ROS_INFO("CameraBaseCalibrationMarker::moveRobot: Trying again in 2 sec.");
					ros::Duration(2.f).sleep();
				}
				else
					ROS_WARN("CameraBaseCalibrationMarker::moveRobot: Skipping base configuration %d.", config_index);
			}
			else
			{
				ROS_FATAL("CameraBaseCalibrationMarker::moveRobot: Exiting calibration, to avoid potential damage.");
				throw std::exception();
			}
		}
	}

	return result;
}

unsigned short CameraLaserscannerType::moveBase(const calibration_utilities::BaseConfiguration &base_configuration)
{
	const double k_base = 0.25;
	const double k_phi = 0.25;

	double error_phi = 0;
	double error_x = 0;
	double error_y = 0;

	cv::Mat T;
	unsigned short error_code = MOV_NO_ERR;

	if (!isReferenceFrameValid(T, error_code))
	{
		turnOffBaseMotion();
		return error_code;
	}

	cv::Vec3d ypr = transform_utilities::YPRFromRotationMatrix(T);
	double robot_yaw = ypr.val[0];
	geometry_msgs::Twist tw;
	error_phi = base_configuration.pose_phi_ - robot_yaw;
	while (error_phi < -CV_PI*0.5)
		error_phi += CV_PI;
	while (error_phi > CV_PI*0.5)
		error_phi -= CV_PI;
	error_x = base_configuration.pose_x_ - T.at<double>(0,3);
	error_y = base_configuration.pose_y_ - T.at<double>(1,3);

	// do not move if close to goal
	bool start_value = true; // for divergence detection
	if (fabs(error_phi) > 0.03 || fabs(error_x) > 0.02 || fabs(error_y) > 0.02)
	{
		// control robot angle
		while(true)
		{
			if (!isReferenceFrameValid(T, error_code))
			{
				turnOffBaseMotion();
				return error_code;
			}

			cv::Vec3d ypr = transform_utilities::YPRFromRotationMatrix(T);
			double robot_yaw = ypr.val[0];
			geometry_msgs::Twist tw;
			error_phi = base_configuration.pose_phi_ - robot_yaw;

			while (error_phi < -CV_PI*0.5)
				error_phi += CV_PI;
			while (error_phi > CV_PI*0.5)
				error_phi -= CV_PI;

			if (fabs(error_phi) < 0.02 || !ros::ok())
				break;

			if ( divergenceDetectedRotation(error_phi, start_value) )
			{
				turnOffBaseMotion();
				return MOV_ERR_FATAL;
			}
			start_value = false;

			tw.angular.z = std::max(-0.05, std::min(0.05, k_phi*error_phi));
			calibration_interface_->assignNewRobotVelocity(tw);
			ros::Rate(20).sleep();
		}

		turnOffBaseMotion();

		// control position
		start_value = true;
		while(true)
		{
			if (!isReferenceFrameValid(T, error_code))
			{
				turnOffBaseMotion();
				return error_code;
			}

			geometry_msgs::Twist tw;
			error_x = base_configuration.pose_x_ - T.at<double>(0,3);
			error_y = base_configuration.pose_y_ - T.at<double>(1,3);
			if ((fabs(error_x) < 0.01 && fabs(error_y) < 0.01) || !ros::ok())
				break;

			if ( divergenceDetectedLocation(error_x, error_y, start_value) )
			{
				turnOffBaseMotion();
				return MOV_ERR_FATAL;
			}
			start_value = false;

			tw.linear.x = std::max(-0.05, std::min(0.05, k_base*error_x));
			tw.linear.y = std::max(-0.05, std::min(0.05, k_base*error_y));
			calibration_interface_->assignNewRobotVelocity(tw);
			ros::Rate(20).sleep();
		}

		turnOffBaseMotion();

		// control robot angle
		start_value = true;
		while (true)
		{
			if (!isReferenceFrameValid(T, error_code))
			{
				turnOffBaseMotion();
				return error_code;
			}

			cv::Vec3d ypr = transform_utilities::YPRFromRotationMatrix(T);
			double robot_yaw = ypr.val[0];
			geometry_msgs::Twist tw;
			error_phi = base_configuration.pose_phi_ - robot_yaw;

			while (error_phi < -CV_PI*0.5)
				error_phi += CV_PI;
			while (error_phi > CV_PI*0.5)
				error_phi -= CV_PI;

			if (fabs(error_phi) < 0.02 || !ros::ok())
				break;

			if ( divergenceDetectedRotation(error_phi, start_value) )
			{
				turnOffBaseMotion();
				return MOV_ERR_FATAL;
			}
			start_value = false;

			tw.angular.z = std::max(-0.05, std::min(0.05, k_phi*error_phi));
			calibration_interface_->assignNewRobotVelocity(tw);
			ros::Rate(20).sleep();
		}

		// turn off robot motion
		turnOffBaseMotion();
	}

	ros::spinOnce();
	return error_code;
}

bool CameraLaserscannerType::isReferenceFrameValid(cv::Mat &T, unsigned short& error_code) // Safety measure, to avoid undetermined motion
{
	if (!transform_utilities::getTransform(transform_listener_, reference_frame_, base_frame_, T, true)) // from reference frame to base frame, swapped order is correct here!
	{
		ROS_WARN("CameraBaseCalibrationMarker::isReferenceFrameValid: Can't retrieve transform between base of robot and reference frame.");
		error_code = MOV_ERR_SOFT;
		return false;
	}

	double currentSqNorm = T.at<double>(0,3)*T.at<double>(0,3) + T.at<double>(1,3)*T.at<double>(1,3) + T.at<double>(2,3)*T.at<double>(2,3);

	// Avoid robot movement if reference frame is too far away
	if ( max_ref_frame_distance_ > 0.0 && currentSqNorm > max_ref_frame_distance_*max_ref_frame_distance_ )
	{
		 ROS_ERROR("CameraBaseCalibrationMarker::isReferenceFrameValid: Reference frame is too far away from current position of the robot.");
		 error_code = MOV_ERR_FATAL;
		 return false;
	}

	// Avoid robot movement if reference frame is jumping around
	double average = 0.0;
	for ( int i=0; i<REF_FRAME_HISTORY_SIZE; ++i )
		average += ref_frame_history_[i];
	average /= REF_FRAME_HISTORY_SIZE;

	if ( average <= 0.0001 || fabs(1.0 - (currentSqNorm/average)) > 0.15  ) // Up to 15% deviation from average is allowed.
	{
		ROS_WARN("CameraBaseCalibrationMarker::isReferenceFrameValid: Reference frame can't be detected reliably. It's current deviation from the average is to too great.");
		error_code = MOV_ERR_SOFT;
		return false;
	}

	// update moving average
	const double current_time = ref_history_timer_.getElapsedTime();
	if ( current_time - last_ref_history_update_ >= 0.05f )  // every 0.05 sec instead of every call -> safer, as history does not fill up so quickly (potentially with bad values)
	{
		last_ref_history_update_ = current_time;
		ref_frame_history_[ref_history_index_] = currentSqNorm; // Update with new measurement
		ref_history_index_ = (++ref_history_index_ < REF_FRAME_HISTORY_SIZE) ? ref_history_index_ : 0;
	}

	return true;
}

void CameraLaserscannerType::turnOffBaseMotion()
{
	geometry_msgs::Twist tw;
	tw.linear.x = 0;
	tw.linear.y = 0;
	tw.angular.z = 0;
	calibration_interface_->assignNewRobotVelocity(tw);
}

bool CameraLaserscannerType::divergenceDetectedRotation(double error_phi, bool start_value)
{
	// unsigned error
	error_phi = fabs(error_phi);

	if ( start_value )
		start_error_phi_ = error_phi;
	else if ( error_phi > (start_error_phi_ + 0.1) ) // ~5° deviation allowed
	{
		ROS_ERROR("Divergence in robot angle detected, robot diverges from rotation setpoint.");
		return true;
	}

	return false;
}

bool CameraLaserscannerType::divergenceDetectedLocation(double error_x, double error_y, bool start_value)
{
	// unsigned error
	error_x = fabs(error_x);
	error_y = fabs(error_y);

	if ( start_value )
	{
		start_error_x_ = error_x;
		start_error_y_ = error_y;
	}
	else if ( error_x > (start_error_x_+ 0.1) || error_y > (start_error_y_+ 0.1) ) // 0.1 m deviation allowed
	{
		ROS_ERROR("Divergence in x- or y-component detected, robot diverges from position setpoint.");
		return true;
	}

	return false;
}

std::string CameraLaserscannerType::getString()
{
	return "camera_laserscanner";
}
