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
 * Date of creation: September 2016
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


#include <robotino_calibration/arm_base_calibration.h>
#include <robotino_calibration/transformation_utilities.h>
#include <std_msgs/Float64MultiArray.h>
#include <sstream>
#include <numeric>  // std::inner_product
#include <robotino_calibration/timer.h>
#include <robotino_calibration/calibration_utilities.h>
#include <cv_bridge/cv_bridge.h>


ArmBaseCalibration::ArmBaseCalibration(ros::NodeHandle nh, CalibrationInterface* interface) :
	RobotCalibration(nh, interface)
{
	// load parameters
	std::cout << "\n========== ArmBaseCalibration Parameters ==========\n";

	// load parameters
	node_handle_.param("chessboard_cell_size", chessboard_cell_size_, 0.05);
	std::cout << "chessboard_cell_size: " << chessboard_cell_size_ << std::endl;
	chessboard_pattern_size_ = cv::Size(6,4);
	std::vector<double> temp;
	node_handle_.getParam("chessboard_pattern_size", temp);
	if (temp.size() == 2)
		chessboard_pattern_size_ = cv::Size(temp[0], temp[1]);
	std::cout << "pattern: " << chessboard_pattern_size_ << std::endl;
	node_handle_.param("arm_dof", arm_dof_, 5);
	std::cout << "arm_dof: " << arm_dof_ << std::endl;

	if ( arm_dof_ < 1 )
	{
		std::cout << "Error: Invalid arm_dof: " << arm_dof_ << ". Setting arm_dof to 1." << std::endl;
		arm_dof_ = 1;
	}

	// coordinate frame name parameters
	node_handle_.param<std::string>("checkerboard_frame", checkerboard_frame_, "");
	std::cout << "checkerboard_frame: " << checkerboard_frame_ << std::endl;
	node_handle_.param<std::string>("camera_image_topic", camera_image_topic_, "");
	std::cout << "camera_image_topic: " << camera_image_topic_ << std::endl;
	node_handle_.param("max_angle_deviation", max_angle_deviation_, 0.5);
	std::cout << "max_angle_deviation: " << max_angle_deviation_ << std::endl;

	temp.clear();
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

	// set up messages
	it_ = new image_transport::ImageTransport(node_handle_);
	color_image_sub_.subscribe(*it_, camera_image_topic_, 1);
	color_image_sub_.registerCallback(boost::bind(&ArmBaseCalibration::imageCallback, this, _1));

	ROS_INFO("ArmBaseCalibration initialized.");
}

ArmBaseCalibration::~ArmBaseCalibration()
{
	if (it_ != 0)
		delete it_;
}

void ArmBaseCalibration::imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg)
{
	// secure this access with a mutex
	boost::mutex::scoped_lock lock(camera_data_mutex_);

	if (capture_image_ == true)
	{
		// read image
		cv_bridge::CvImageConstPtr color_image_ptr;
		if (calibration_utilities::convertImageMessageToMat(color_image_msg, color_image_ptr, camera_image_) == false)
			return;

		latest_image_time_ = color_image_msg->header.stamp;

		if ( latest_image_time_.toSec() <= 0.01f ) //No time stamps -> use current time as stamp
			latest_image_time_ = ros::Time::now();

		capture_image_ = false;
	}
}

bool ArmBaseCalibration::calibrateArmToBase(const bool load_images)
{
	// pre-cache images
	if (load_images == false)
	{
		ros::spinOnce();
		ros::Duration(2).sleep();
		capture_image_ = true;
		ros::spinOnce();
		ros::Duration(2).sleep();
		capture_image_ = true;
	}

	// acquire images
	int image_width=0, image_height=0;
	std::vector< std::vector<cv::Point2f> > points_2d_per_image;
	std::vector<cv::Mat> T_gapfirst_to_marker_vector;
	std::vector< std::vector<cv::Mat> > T_between_gaps_vector;
	std::vector<cv::Mat> T_gaplast_to_camera_optical_vector;
	acquireCalibrationImages(chessboard_pattern_size_, load_images, image_width, image_height, points_2d_per_image, T_gapfirst_to_marker_vector,
							 T_between_gaps_vector, T_gaplast_to_camera_optical_vector);

	// prepare chessboard 3d points
	std::vector< std::vector<cv::Point3f> > pattern_points_3d;
	calibration_utilities::computeCheckerboard3dPoints(pattern_points_3d, chessboard_pattern_size_, chessboard_cell_size_, points_2d_per_image.size());

	// intrinsic calibration for camera
	std::vector<cv::Mat> rvecs, tvecs, T_gaplast_to_marker_vector;
	intrinsicCalibration(pattern_points_3d, points_2d_per_image, cv::Size(image_width, image_height), rvecs, tvecs);
	for (size_t i=0; i<rvecs.size(); ++i)
	{
		cv::Mat R, t;
		cv::Rodrigues(rvecs[i], R);
		cv::Mat T_gaplast_to_marker = T_gaplast_to_camera_optical_vector[i] * transform_utilities::makeTransform(R, tvecs[i]);
		T_gaplast_to_marker_vector.push_back(T_gaplast_to_marker);
	}

	// extrinsic calibration optimization
	for (int i=0; i<optimization_iterations_; ++i)
	{
		for ( int j=0; j<transforms_to_calibrate_.size(); ++j )
		{
			extrinsicCalibration(pattern_points_3d, T_gapfirst_to_marker_vector, T_between_gaps_vector, T_gaplast_to_marker_vector, calibration_order_[j]);
		}
	}

	// display and save calibration parameters
	RobotCalibration::displayAndSaveCalibrationResult("arm_calibration_urdf.txt");

	calibrated_ = true;
	return true;
}

void ArmBaseCalibration::intrinsicCalibration(const std::vector< std::vector<cv::Point3f> >& pattern_points, const std::vector< std::vector<cv::Point2f> >& camera_points_2d_per_image, const cv::Size& image_size, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs)
{
	std::cout << "Intrinsic calibration started ..." << std::endl;
	K_ = cv::Mat::eye(3, 3, CV_64F);
	distortion_ = cv::Mat::zeros(8, 1, CV_64F);
	cv::calibrateCamera(pattern_points, camera_points_2d_per_image, image_size, K_, distortion_, rvecs, tvecs);
	std::cout << "Intrinsic calibration:\nK:\n" << K_ << "\ndistortion:\n" << distortion_ << std::endl;

	double error = calibration_utilities::computeReprojectionError(pattern_points, camera_points_2d_per_image, rvecs, tvecs, K_, distortion_);
	std::cout << "Total reprojection error: " << error << std::endl;
}

void ArmBaseCalibration::moveRobot(int config_index)
{
	RobotCalibration::moveRobot(config_index); // Call parent
	moveArm(arm_configurations_[config_index]);
	// Does not make too much sense here for now, as it only returns false in case of an dimension error
	/*for ( short i=0; i<NUM_MOVE_TRIES; ++i )
	{
		if ( !moveArm(arm_configurations_[config_index]) )
		{
			ROS_ERROR("ArmBaseCalibration::moveRobot: Could not execute moveArm, (%d/%d) tries.", i+1, NUM_MOVE_TRIES);
			if ( i<NUM_MOVE_TRIES-1 )
			{
				ROS_INFO("ArmBaseCalibration::moveRobot: Trying again in 1 sec.");
				ros::Duration(1.f).sleep();
			}
			else
				ROS_WARN("ArmBaseCalibration::moveRobot: Skipping arm configuration %d.", config_index);
		}
		else
			break;
	}*/
}

bool ArmBaseCalibration::moveArm(const std::vector<double>& arm_configuration)
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

bool ArmBaseCalibration::acquireCalibrationImages(const cv::Size pattern_size, const bool load_images, int& image_width, int& image_height,
												  std::vector< std::vector<cv::Point2f> >& points_2d_per_image, std::vector<cv::Mat>& T_gapfirst_to_marker_vector,
												  std::vector< std::vector<cv::Mat> >& T_between_gaps_vector, std::vector<cv::Mat>& T_gaplast_to_camera_optical_vector)
{
	// capture images from different perspectives
	const int number_images_to_capture = (int)camera_configurations_.size();
	for (int image_counter = 0; image_counter < number_images_to_capture; ++image_counter)
	{
		if ( !ros::ok() )
			return false;

		std::cout << "Configuration " << (image_counter+1) << "/" << number_images_to_capture << std::endl;

		if ( !load_images )
			moveRobot(image_counter);

		// acquire image and extract checkerboard points
		std::vector<cv::Point2f> checkerboard_points_2d;
		int return_value = acquireCalibrationImage(image_width, image_height, checkerboard_points_2d, pattern_size, load_images, image_counter);
		if (return_value != 0)
			continue;

		// retrieve transformations
		std::stringstream path;
		path << calibration_storage_path_ << image_counter << ".yml";
		cv::Mat T_gapfirst_to_marker, T_gaplast_to_camera_optical;
		std::vector<cv::Mat> T_between_gaps;
		if ( load_images == false )
		{
			bool result = calculateTransformationChains(T_gapfirst_to_marker, T_between_gaps, T_gaplast_to_camera_optical, checkerboard_frame_);
			if (result == false)
				continue;

			// save transforms to file
			cv::FileStorage fs(path.str().c_str(), cv::FileStorage::WRITE);
			if (fs.isOpened())
			{
				fs << "T_gapfirst_to_marker" << T_gapfirst_to_marker;
				fs << "T_gaplast_to_camera_optical" << T_gaplast_to_camera_optical;
				fs << "T_between_gaps" << T_between_gaps;
			}
			else
			{
				ROS_WARN("Could not write transformations to file '%s'.", path.str().c_str());
			}
			fs.release();
		}
		else
		{
			// load data from file
			cv::FileStorage fs(path.str().c_str(), cv::FileStorage::READ);
			if (fs.isOpened())
			{
				fs["T_gapfirst_to_marker"] >> T_gapfirst_to_marker;
				fs["T_gaplast_to_camera_optical"] >> T_gaplast_to_camera_optical;
				fs["T_between_gaps"] >> T_between_gaps;
			}
			else
			{
				ROS_WARN("Could not read transformations from file '%s'.", path.str().c_str());
			}
			fs.release();
		}

		points_2d_per_image.push_back(checkerboard_points_2d);
		T_gapfirst_to_marker_vector.push_back(T_gapfirst_to_marker);
		T_between_gaps_vector.push_back(T_between_gaps);
		T_gaplast_to_camera_optical_vector.push_back(T_gaplast_to_camera_optical);
		std::cout << "Captured perspectives: " << points_2d_per_image.size() << std::endl;
	}

	return true;
}

int ArmBaseCalibration::acquireCalibrationImage(int& image_width, int& image_height,
												std::vector<cv::Point2f>& checkerboard_points_2d, const cv::Size pattern_size, const bool load_images, int& image_counter)
{
	int return_value = 0;

	// acquire image
	cv::Mat image, gray;
	if (load_images == false)
	{
		ros::Duration(3).sleep();
		capture_image_ = true;
		ros::spinOnce();
		ros::Duration(2).sleep();

		// retrieve image from camera
		{
			boost::mutex::scoped_lock lock(camera_data_mutex_);

			std::cout << "Time diff: " << (ros::Time::now() - latest_image_time_).toSec() << std::endl;

			if ((ros::Time::now() - latest_image_time_).toSec() < 20.0)
			{
				image = camera_image_.clone();

				// convert to grayscale
				gray = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
				cv::cvtColor(image, gray, CV_BGR2GRAY);
			}
			else
			{
				ROS_WARN("Did not receive camera images recently.");
				return -1;		// -1 = no fresh image available
			}
		}
	}
	else
	{
		// load image from file
		std::stringstream ss;
		ss << calibration_storage_path_ << image_counter;
		std::string image_name = ss.str() + ".png";
		gray = cv::imread(image_name.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
		if (gray.empty())
			return -2;
	}
	image_width = gray.cols;
	image_height = gray.rows;

	// find pattern in image
	bool pattern_found = cv::findChessboardCorners(gray, pattern_size, checkerboard_points_2d,
												   cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

	if ( pattern_found )
	{
		cv::cornerSubPix( gray, checkerboard_points_2d, cv::Size(11,11),
						  cv::Size(-1,-1), cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1 ));
	}

	// collect 2d points
	if (checkerboard_points_2d.size() == pattern_size.height*pattern_size.width)
	{
		// save images
		if (load_images == false)
		{
			std::stringstream ss;
			ss << calibration_storage_path_ << image_counter;
			std::string image_name = ss.str() + ".png";
			cv::imwrite(image_name.c_str(), image);
		}
	}
	else
	{
		ROS_WARN("Not all checkerboard points have been observed.");
		return_value = -2;
	}

	return return_value;
}
