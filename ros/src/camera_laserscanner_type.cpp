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
#include <robotino_calibration/time_utilities.h>


CameraLaserscannerType::CameraLaserscannerType() :
		last_ref_history_update_(0.0), start_error_x_(0.0), start_error_y_(0.0), start_error_phi_(0.0), ref_history_index_(0), max_ref_frame_distance_(1.0),
		mapped_base_index_(0), current_camera_counter_(0), mapped_camera_index_(0), cameras_done_(false)
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

	const int base_dof = NUM_POSE_PARAMS; // coming from pose_definition.h

	// read out user-defined robot configurations
	std::vector<double> base_data;
	node_handle_.getParam("base_configs", base_data);

	if ( !base_data.empty() )  // configs has been found
	{
		const int num_params = base_dof;
		const int num_configs = base_data.size()/num_params;
		if (base_data.size() % num_params != 0 || base_data.size() < 3*num_params)
		{
			ROS_ERROR("The base_configs vector should contain at least 3 configurations with %d values each.", num_params);
			return;
		}

		for ( int i=0; i<num_configs; ++i )
		{
			std::vector<double> data;
			for ( int j=0; j<base_dof; ++j )
			{
				data.push_back(base_data[num_params*i + j]);
			}
			base_configurations_.push_back(pose_definition::RobotConfiguration(data));
		}
	}
	else
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

		if (x_range[0] == x_range[2] || x_range[1] == 0.)		// this sets the step to something bigger than 0
			x_range[1] = 1.0;
		if (y_range[0] == y_range[2] || y_range[1] == 0.)
			y_range[1] = 1.0;
		if (phi_range[0] == phi_range[2] || phi_range[1] == 0.)
			phi_range[1] = 1.0;

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

		std::vector< std::vector<double> > base_configs;
		generateConfigs(param_vector, base_configs);

		base_configurations_.reserve(base_configs.size());
		for ( int i=0; i<base_configs.size(); ++i )
			base_configurations_.push_back(pose_definition::RobotConfiguration(base_configs[i]));

		std::cout << "Generated " << (int)base_configurations_.size() << " base configurations for calibration." << std::endl;
	}

	// Display base configurations
	std::cout << "base configurations:" << std::endl;
	for ( int i=0; i<base_configurations_.size(); ++i )
		std::cout << base_configurations_[i].getString() << std::endl;
	std::cout << std::endl;

	if ( base_configurations_.empty() )
	{
		ROS_ERROR("No base configurations available. Please check your yaml file.");
		return;
	}

	max_configuration_count = 0;
	for ( int i=0; i<cameras_.size(); ++i )  // compute total configuration count for cameras
		max_configuration_count += cameras_[i].configurations_.size();

	max_configuration_count *= base_configurations_.size();  // each base config contains all camera configs
	std::cout << max_configuration_count << " configurations in total used for calibration." << std::endl;

	// Check whether relative_localization has initialized the reference frame yet.
	// Do not let the robot start driving when the reference frame has not been set up properly! Bad things could happen!
	const double start_time = time_utilities::getSystemTimeSec();
	bool result = false;

	while ( time_utilities::getTimeElapsedSec(start_time) < 10.f )
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

	initialized_ = true;
}

bool CameraLaserscannerType::moveRobot(int config_index)
{
	bool result = CalibrationType::moveRobot(config_index);  // call parent to move current camera

	// Move base
	if ( result )
	{
		if ( cameras_done_ )  // go to new location only when all cameras are done with previous location
			++mapped_base_index_;

		for ( short i=0; i<NUM_MOVE_TRIES; ++i )
		{
			unsigned short error_code = moveBase(base_configurations_[mapped_base_index_]);

			if ( error_code == MOV_NO_ERR ) // Exit loop, as successfully executed move
			{
				std::cout << "Base configuration reached." << std::endl;
				break;
			}
			else if ( error_code == MOV_ERR_SOFT ) // Retry last failed move
			{
				ROS_WARN("CameraLaserscannerType::moveRobot: Could not execute moveBase, (%d/%d) tries.", i+1, NUM_MOVE_TRIES);
				if ( i<NUM_MOVE_TRIES-1 )
				{
					ROS_INFO("CameraLaserscannerType::moveRobot: Trying again in 2 sec.");
					ros::Duration(2.f).sleep();
				}
				else
					ROS_WARN("CameraLaserscannerType::moveRobot: Skipping base configuration %d.", mapped_base_index_);
			}
			else
			{
				ROS_FATAL("CameraLaserscannerType::moveRobot: Exiting calibration, to avoid potential damage.");
				throw std::exception();
			}
		}
	}

	return result;
}

bool CameraLaserscannerType::moveCameras(int config_index)
{
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

unsigned short CameraLaserscannerType::moveBase(const pose_definition::RobotConfiguration &base_configuration)
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
	if ( fabs(error_phi) > 0.02 || fabs(error_x) > 0.01 || fabs(error_y) > 0.01 )
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

	return error_code;
}

bool CameraLaserscannerType::isReferenceFrameValid(cv::Mat &T, unsigned short& error_code) // Safety measure, to avoid undetermined motion
{
	ros::spinOnce();  // get newest messages
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
	const double current_time = time_utilities::getSystemTimeSec();
	if ( time_utilities::getTimeElapsedSec(last_ref_history_update_) >= 0.05f )  // every 0.05 sec instead of every call -> safer, as history does not fill up so quickly (potentially with bad values)
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
