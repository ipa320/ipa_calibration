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

	// set up messages
	it_ = new image_transport::ImageTransport(node_handle_);
	color_image_sub_.subscribe(*it_, "colorimage_in", 1);
	color_image_sub_.registerCallback(boost::bind(&CameraBaseCalibrationCheckerboard::imageCallback, this, _1));

	ROS_INFO("CameraBaseCalibration initialized.");
}

CameraBaseCalibrationCheckerboard::~CameraBaseCalibrationCheckerboard()
{
	if (it_ != 0)
		delete it_;
}

bool CameraBaseCalibrationCheckerboard::convertImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
{
	try
	{
		image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);//image_msg->encoding);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ImageFlip::convertColorImageMessageToMat: cv_bridge exception: %s", e.what());
		return false;
	}
	image = image_ptr->image;

	return true;
}

void CameraBaseCalibrationCheckerboard::imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg)
{
	// secure this access with a mutex
	boost::mutex::scoped_lock lock(camera_data_mutex_);

	if (capture_image_ == true)
	{
		// read image
		cv_bridge::CvImageConstPtr color_image_ptr;
		if (convertImageMessageToMat(color_image_msg, color_image_ptr, camera_image_) == false)
			return;

		latest_image_time_ = color_image_msg->header.stamp;

		capture_image_ = false;
	}
}

bool CameraBaseCalibrationCheckerboard::calibrateCameraToBase(const bool load_images)
{
	// setup storage folder
	int return_value = system("mkdir -p robotino_calibration/camera_calibration");

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
	std::vector<cv::Mat> T_base_to_checkerboard_vector;
	std::vector<cv::Mat> T_torso_lower_to_torso_upper_vector;
	std::vector<cv::Mat> T_camera_to_camera_optical_vector;
	acquireCalibrationImages(robot_configurations_, chessboard_pattern_size_, load_images, image_width, image_height, points_2d_per_image, T_base_to_checkerboard_vector,
			T_torso_lower_to_torso_upper_vector, T_camera_to_camera_optical_vector);

	// prepare chessboard 3d points
	std::vector< std::vector<cv::Point3f> > pattern_points_3d;
	computeCheckerboard3dPoints(pattern_points_3d, chessboard_pattern_size_, chessboard_cell_size_, points_2d_per_image.size());

	// intrinsic calibration for camera
	std::vector<cv::Mat> rvecs, tvecs, T_camera_to_checkerboard_vector;
	intrinsicCalibration(pattern_points_3d, points_2d_per_image, cv::Size(image_width, image_height), rvecs, tvecs);
	for (size_t i=0; i<rvecs.size(); ++i)
	{
		cv::Mat R, t;
		cv::Rodrigues(rvecs[i], R);
		cv::Mat T_camera_to_checkerboard = T_camera_to_camera_optical_vector[i] * robotino_calibration::makeTransform(R, tvecs[i]);
		T_camera_to_checkerboard_vector.push_back(T_camera_to_checkerboard);
	}

	// extrinsic calibration between base and torso_lower as well ass torso_upper and camera
	for (int i=0; i<optimization_iterations_; ++i)
	{
//		std::cout << "\nExtrinsic optimization run " << i << ":" << std::endl;
		extrinsicCalibrationBaseToTorsoLower(pattern_points_3d, T_base_to_checkerboard_vector, T_torso_lower_to_torso_upper_vector, T_camera_to_checkerboard_vector);
//		cv::Vec3d ypr = YPRFromRotationMatrix(T_base_to_torso_lower_);
//		std::cout << "T_base_to_torso_lower:\n" << T_base_to_torso_lower_ << std::endl;
//		std::cout << "yaw=" << ypr.val[0] << "   pitch=" << ypr.val[1] << "   roll=" << ypr.val[2] << std::endl;
		extrinsicCalibrationTorsoUpperToCamera(pattern_points_3d, T_base_to_checkerboard_vector, T_torso_lower_to_torso_upper_vector, T_camera_to_checkerboard_vector);
//		ypr = YPRFromRotationMatrix(T_torso_upper_to_camera_);
//		std::cout << "T_torso_upper_to_camera:\n" << T_torso_upper_to_camera_ << std::endl;
//		std::cout << "yaw=" << ypr.val[0] << "   pitch=" << ypr.val[1] << "   roll=" << ypr.val[2] << std::endl;
	}

	// display calibration parameters
	std::cout << "\n\n\n----- Replace these parameters in your 'squirrel_robotino/robotino_bringup/robots/xyz_robotino/urdf/properties.urdf.xacro' file -----\n\n";
	cv::Vec3d ypr = robotino_calibration::YPRFromRotationMatrix(T_base_to_torso_lower_);
	std::cout << "  <!-- pan_tilt mount positions | handeye calibration | relative to base_link -->\n"
			  << "  <property name=\"pan_tilt_x\" value=\"" << T_base_to_torso_lower_.at<double>(0,3) << "\"/>\n"
			  << "  <property name=\"pan_tilt_y\" value=\"" << T_base_to_torso_lower_.at<double>(1,3) << "\"/>\n"
			  << "  <property name=\"pan_tilt_z\" value=\"" << T_base_to_torso_lower_.at<double>(2,3) << "\"/>\n"
			  << "  <property name=\"pan_tilt_roll\" value=\"" << ypr.val[2] << "\"/>\n"
			  << "  <property name=\"pan_tilt_pitch\" value=\"" << ypr.val[1] << "\"/>\n"
			  << "  <property name=\"pan_tilt_yaw\" value=\"" << ypr.val[0] << "\"/>\n\n";
	ypr = robotino_calibration::YPRFromRotationMatrix(T_torso_upper_to_camera_);
	std::cout << "  <!-- kinect mount positions | handeye calibration | relative to pan_tilt_link -->\n"
			  << "  <property name=\"kinect_x\" value=\"" << T_torso_upper_to_camera_.at<double>(0,3) << "\"/>\n"
			  << "  <property name=\"kinect_y\" value=\"" << T_torso_upper_to_camera_.at<double>(1,3) << "\"/>\n"
			  << "  <property name=\"kinect_z\" value=\"" << T_torso_upper_to_camera_.at<double>(2,3) << "\"/>\n"
			  << "  <property name=\"kinect_roll\" value=\"" << ypr.val[2] << "\"/>\n"
			  << "  <property name=\"kinect_pitch\" value=\"" << ypr.val[1] << "\"/>\n"
			  << "  <property name=\"kinect_yaw\" value=\"" << ypr.val[0] << "\"/>\n" << std::endl;

	// save calibration
	saveCalibration();
	calibrated_ = true;

	return true;
}

bool CameraBaseCalibrationCheckerboard::acquireCalibrationImages(const std::vector<RobotConfiguration>& robot_configurations,
		const cv::Size pattern_size, const bool load_images, int& image_width, int& image_height,
		std::vector< std::vector<cv::Point2f> >& points_2d_per_image, std::vector<cv::Mat>& T_base_to_checkerboard_vector,
		std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector, std::vector<cv::Mat>& T_camera_to_camera_optical_vector)
{
	// capture images from different perspectives
	const int number_images_to_capture = (int)robot_configurations.size();
	for (int image_counter = 0; image_counter < number_images_to_capture; ++image_counter)
	{
		if (!load_images)
			moveRobot(robot_configurations[image_counter]);

		// acquire image and extract checkerboard points
		std::vector<cv::Point2f> checkerboard_points_2d;
		int return_value = acquireCalibrationImage(image_width, image_height, checkerboard_points_2d, pattern_size, load_images, image_counter);
		if (return_value != 0)
			continue;

		// retrieve transformations
		cv::Mat T_base_to_checkerboard, T_torso_lower_to_torso_upper, T_camera_to_camera_optical;
		std::stringstream path;
		path << camera_calibration_path_ << image_counter << ".yml";
		if (load_images)
		{
			cv::FileStorage fs(path.str().c_str(), cv::FileStorage::READ);
			if (fs.isOpened())
			{
				fs["T_base_to_checkerboard"] >> T_base_to_checkerboard;
				fs["T_torso_lower_to_torso_upper"] >> T_torso_lower_to_torso_upper;
				fs["T_camera_to_camera_optical"] >> T_camera_to_camera_optical;
			}
			else
			{
				ROS_WARN("Could not read transformations from file '%s'.", path.str().c_str());
				continue;
			}
			fs.release();
		}
		else
		{
			bool result = true;
			result &= getTransform(base_frame_, checkerboard_frame_, T_base_to_checkerboard);
			result &= getTransform(torso_lower_frame_, torso_upper_frame_, T_torso_lower_to_torso_upper);
			result &= getTransform(camera_frame_, camera_optical_frame_, T_camera_to_camera_optical);

			if (result == false)
				continue;

			// save transforms to file
			cv::FileStorage fs(path.str().c_str(), cv::FileStorage::WRITE);
			if (fs.isOpened())
			{
				fs << "T_base_to_checkerboard" << T_base_to_checkerboard;
				fs << "T_torso_lower_to_torso_upper" << T_torso_lower_to_torso_upper;
				fs << "T_camera_to_camera_optical" << T_camera_to_camera_optical;
			}
			else
			{
				ROS_WARN("Could not write transformations to file '%s'.", path.str().c_str());
				continue;
			}
			fs.release();
		}

		points_2d_per_image.push_back(checkerboard_points_2d);
		T_base_to_checkerboard_vector.push_back(T_base_to_checkerboard);
		T_torso_lower_to_torso_upper_vector.push_back(T_torso_lower_to_torso_upper);
		T_camera_to_camera_optical_vector.push_back(T_camera_to_camera_optical);
		std::cout << "Captured perspectives: " << points_2d_per_image.size() << std::endl;
	}

	return true;
}

int CameraBaseCalibrationCheckerboard::acquireCalibrationImage(int& image_width, int& image_height,
		std::vector<cv::Point2f>& checkerboard_points_2d, const cv::Size pattern_size, const bool load_images, int& image_counter)
{
	int return_value = 0;

	// acquire image
	cv::Mat image;
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
		ss << camera_calibration_path_ << image_counter;
		std::string image_name = ss.str() + ".png";
		image = cv::imread(image_name.c_str(), 0);
		if (image.empty())
			return -2;
	}
	image_width = image.cols;
	image_height = image.rows;

	// find pattern in image
	bool pattern_found = cv::findChessboardCorners(image, pattern_size, checkerboard_points_2d, cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_FILTER_QUADS);

	// display
	cv::Mat display = image.clone();
	cv::drawChessboardCorners(display, pattern_size, cv::Mat(checkerboard_points_2d), pattern_found);
	cv::imshow("image", display);
	cv::waitKey(50);

	// collect 2d points
	if (checkerboard_points_2d.size() == pattern_size.height*pattern_size.width)
	{
		// save images
		if (load_images == false)
		{
			std::stringstream ss;
			ss << camera_calibration_path_ << image_counter;
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

void CameraBaseCalibrationCheckerboard::computeCheckerboard3dPoints(std::vector< std::vector<cv::Point3f> >& pattern_points, const cv::Size pattern_size, const double chessboard_cell_size, const int number_images)
{
	// prepare chessboard 3d points
	pattern_points.clear();
	pattern_points.resize(1);
	pattern_points[0].resize(pattern_size.height*pattern_size.width);
	for (int v=0; v<pattern_size.height; ++v)
		for (int u=0; u<pattern_size.width; ++u)
			pattern_points[0][v*pattern_size.width+u] = cv::Point3f(u*chessboard_cell_size, v*chessboard_cell_size, 0.f);
	pattern_points.resize(number_images, pattern_points[0]);
}

void CameraBaseCalibrationCheckerboard::intrinsicCalibration(const std::vector< std::vector<cv::Point3f> >& pattern_points, const std::vector< std::vector<cv::Point2f> >& camera_points_2d_per_image, const cv::Size& image_size, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs)
{
	std::cout << "Intrinsic calibration started ..." << std::endl;
	K_ = cv::Mat::eye(3, 3, CV_64F);
	distortion_ = cv::Mat::zeros(8, 1, CV_64F);
	cv::calibrateCamera(pattern_points, camera_points_2d_per_image, image_size, K_, distortion_, rvecs, tvecs);
	std::cout << "Intrinsic calibration:\nK:\n" << K_ << "\ndistortion:\n" << distortion_ << std::endl;
}

bool CameraBaseCalibrationCheckerboard::saveCalibration()
{
	bool success = true;

	// save calibration
	std::string filename = camera_calibration_path_ + "camera_calibration.yml";
	cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
	if (fs.isOpened() == true)
	{
		fs << "K" << K_;
		fs << "distortion" << distortion_;
		fs << "T_base_to_torso_lower" << T_base_to_torso_lower_;
		fs << "T_torso_upper_to_camera" << T_torso_upper_to_camera_;
	}
	else
	{
		std::cout << "Error: CameraBaseCalibration::saveCalibration: Could not write calibration to file.";
		success = false;
	}
	fs.release();

	return success;
}

bool CameraBaseCalibrationCheckerboard::loadCalibration()
{
	bool success = true;

	// load calibration
	std::string filename = camera_calibration_path_ + "camera_calibration.yml";
	cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
	if (fs.isOpened() == true)
	{
		fs["K"] >> K_;
		fs["distortion"] >> distortion_;
		fs["T_base_to_torso_lower"] >> T_base_to_torso_lower_;
		fs["T_torso_upper_to_camera"] >> T_torso_upper_to_camera_;
	}
	else
	{
		std::cout << "Error: CameraBaseCalibration::loadCalibration: Could not read calibration from file.";
		success = false;
	}
	fs.release();

	calibrated_ = true;

	return success;
}

void CameraBaseCalibrationCheckerboard::getCalibration(cv::Mat& K, cv::Mat& distortion, cv::Mat& T_base_to_torso_lower, cv::Mat& T_torso_upper_to_camera)
{
	if (calibrated_ == false && loadCalibration() == false)
	{
		std::cout << "Error: CameraBaseCalibration not calibrated and no calibration data available on disk." << std::endl;
		return;
	}

	K = K_.clone();
	distortion = distortion_.clone();
	T_base_to_torso_lower = T_base_to_torso_lower_.clone();
	T_torso_upper_to_camera = T_torso_upper_to_camera_.clone();
}

void CameraBaseCalibrationCheckerboard::undistort(const cv::Mat& image, cv::Mat& image_undistorted)
{
	if (calibrated_ == false && loadCalibration() == false)
	{
		std::cout << "Error: HandCameraCalibration not calibrated and no calibration data available on disk." << std::endl;
		return;
	}

	cv::undistort(image, image_undistorted, K_, distortion_);
}
