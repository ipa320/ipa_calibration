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
 * Author: Richard Bormann, email:richard.bormann@ipa.fhg.de
 *
 * Date of creation: December 2015
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

#include <robotino_calibration/camera_base_calibration_checkerboard.h>
#include <robotino_calibration/transformation_utilities.h>

#include <std_msgs/Float64.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <sstream>


CameraBaseCalibrationCheckerboard::CameraBaseCalibrationCheckerboard(ros::NodeHandle nh) :
			CameraBaseCalibrationMarker(nh)
{
	// load parameters
	std::cout << "========== CameraBaseCalibrationCheckerboard Parameters ==========\n";
	node_handle_.param("chessboard_cell_size", chessboard_cell_size_, 0.05);
	std::cout << "chessboard_cell_size: " << chessboard_cell_size_ << std::endl;
	chessboard_pattern_size_ = cv::Size(6,4);
	std::vector<double> temp;
	node_handle_.getParam("chessboard_pattern_size", temp);
	if (temp.size() == 2)
		chessboard_pattern_size_ = cv::Size(temp[0], temp[1]);
	std::cout << "pattern: " << chessboard_pattern_size_ << std::endl;
	node_handle_.param<std::string>("checkerboard_frame", checkerboard_frame_, "");
	std::cout << "checkerboard_frame: " << checkerboard_frame_ << std::endl;
	node_handle_.param<std::string>("camera_image_topic", camera_image_topic_, "");
	std::cout << "camera_image_topic: " << camera_image_topic_ << std::endl;

	// set up messages
	it_ = new image_transport::ImageTransport(node_handle_);
	color_image_sub_.subscribe(*it_, camera_image_topic_, 1);
	color_image_sub_.registerCallback(boost::bind(&CameraBaseCalibrationCheckerboard::imageCallback, this, _1));

	ROS_INFO("CameraBaseCalibrationCheckerboard initialized.");
}

CameraBaseCalibrationCheckerboard::~CameraBaseCalibrationCheckerboard()
{
	if (it_ != 0)
		delete it_;
}

void CameraBaseCalibrationCheckerboard::imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg)
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

		capture_image_ = false;
	}
}

bool CameraBaseCalibrationCheckerboard::calibrateCameraToBase(const bool load_images)
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
	acquireCalibrationImages(robot_configurations_, chessboard_pattern_size_, load_images, image_width, image_height, points_2d_per_image, T_gapfirst_to_marker_vector,
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

	// display calibration parameters
	displayAndSaveCalibrationResult();

	calibrated_ = true;
	return true;
}

bool CameraBaseCalibrationCheckerboard::acquireCalibrationImages(const std::vector<calibration_utilities::RobotConfiguration>& robot_configurations,
		const cv::Size pattern_size, const bool load_images, int& image_width, int& image_height,
		std::vector< std::vector<cv::Point2f> >& points_2d_per_image, std::vector<cv::Mat>& T_gapfirst_to_marker_vector,
		std::vector< std::vector<cv::Mat> >& T_between_gaps_vector, std::vector<cv::Mat>& T_gaplast_to_camera_optical_vector)
{
	// capture images from different perspectives
	const int number_images_to_capture = (int)robot_configurations.size();
	for (int image_counter = 0; image_counter < number_images_to_capture; ++image_counter)
	{
		if ( !ros::ok() )
			return false;

		std::cout << "Configuration " << (image_counter+1) << "/" << number_images_to_capture << std::endl;

		if (!load_images)
			moveRobot(robot_configurations[image_counter]);

		// acquire image and extract checkerboard points
		std::vector<cv::Point2f> checkerboard_points_2d;
		int return_value = acquireCalibrationImage(image_width, image_height, checkerboard_points_2d, pattern_size, load_images, image_counter);
		if (return_value != 0)
			continue;

		// retrieve transformations
		std::stringstream path;
		path << calibration_storage_path_ << image_counter << ".yml";
		cv::Mat T_gapfirst_to_marker, T_gaplast_to_camera_optical, T_camera_optical_to_marker, T_gaplast_to_marker;
		std::vector<cv::Mat> T_between_gaps;
		if ( load_images == false )
		{
			bool result = true;
			result &= transform_utilities::getTransform(transform_listener_, transforms_to_calibrate_[0].parent_, checkerboard_frame_, T_gapfirst_to_marker);
			result &= transform_utilities::getTransform(transform_listener_, transforms_to_calibrate_[ transforms_to_calibrate_.size()-1 ].child_, camera_optical_frame_, T_gaplast_to_camera_optical);

			for ( int i=0; i<transforms_to_calibrate_.size()-1; ++i )
			{
				if ( transforms_to_calibrate_[i].parent_ == transforms_to_calibrate_[i].child_ ) // several gaps in a row, no certain trafos in between
					continue;

				cv::Mat temp;
				result &= transform_utilities::getTransform(transform_listener_, transforms_to_calibrate_[i].child_, transforms_to_calibrate_[i+1].parent_, temp);
				T_between_gaps.push_back(temp);
				transforms_to_calibrate_[i].trafo_until_next_gap_idx_ = T_between_gaps.size()-1;
			}

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

int CameraBaseCalibrationCheckerboard::acquireCalibrationImage(int& image_width, int& image_height,
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

void CameraBaseCalibrationCheckerboard::intrinsicCalibration(const std::vector< std::vector<cv::Point3f> >& pattern_points, const std::vector< std::vector<cv::Point2f> >& camera_points_2d_per_image, const cv::Size& image_size, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs)
{
	std::cout << "Intrinsic calibration started ..." << std::endl;
	K_ = cv::Mat::eye(3, 3, CV_64F);
	distortion_ = cv::Mat::zeros(8, 1, CV_64F);
	cv::calibrateCamera(pattern_points, camera_points_2d_per_image, image_size, K_, distortion_, rvecs, tvecs);
	std::cout << "Intrinsic calibration:\nK:\n" << K_ << "\ndistortion:\n" << distortion_ << std::endl;
}

void CameraBaseCalibrationCheckerboard::undistort(const cv::Mat& image, cv::Mat& image_undistorted)
{
	if ( calibrated_ == false )
	{
		std::cout << "Error: CameraBaseCalibrationCheckerboard not calibrated." << std::endl;
		return;
	}

	cv::undistort(image, image_undistorted, K_, distortion_);
}
