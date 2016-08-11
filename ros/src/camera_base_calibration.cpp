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
 * ROS stack name: squirrel_robotino
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

#include <robotino_calibration/camera_base_calibration.h>

#include <std_msgs/Float64.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <sstream>


/*cv::Mat rotationMatrixFromYPR(double roll, double pitch, double yaw) // deprecated
{
	double sr = sin(roll);
	double cr = cos(roll);
	double sp = sin(pitch);
	double cp = cos(pitch);
	double sy = sin(yaw);
	double cy = cos(yaw);
	cv::Mat rotation = (cv::Mat_<double>(3,3) <<
			cr*cp,		cr*sp*sy - sr*cy,		cr*sp*cy + sr*sy,
			sr*cp,		sr*sp*sy + cr*cy,		sr*sp*cy - cr*sy,
			-sp,		cp*sy,					cp*cy);

	return rotation;
}*/

cv::Mat rotationMatrixFromYPR(double yaw, double pitch, double roll)
{
	double sr = sin(roll);
	double cr = cos(roll);
	double sp = sin(pitch);
	double cp = cos(pitch);
	double sy = sin(yaw);
	double cy = cos(yaw);

	cv::Mat rotation = (cv::Mat_<double>(3,3) <<
	  		cp*cy,		cy*sp*sr - cr*sy,		sr*sy + cr*cy*sp,
	  		cp*sy,		cr*cy + sp*sr*sy,		cr*sp*sy - cy*sr,
	  		-sp,		cp*sr,					cp*cr);

	return rotation;
}

cv::Vec3d RPYFromRotationMatrix(const cv::Mat& rot)
{
	Eigen::Matrix3f rot_eigen;
	for (int i=0; i<3; ++i)
		for (int j=0; j<3; ++j)
			rot_eigen(i,j) = rot.at<double>(i,j);
	Eigen::Vector3f euler_angles = rot_eigen.eulerAngles(2,1,0);
	return cv::Vec3d(euler_angles(2), euler_angles(1), euler_angles(0));
}

cv::Mat makeTransform(const cv::Mat& R, const cv::Mat& t)
{
	cv::Mat T = (cv::Mat_<double>(4,4) <<
			R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
			R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
			R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2),
			0., 0., 0., 1);
	return T;
}

CameraBaseCalibration::CameraBaseCalibration(ros::NodeHandle nh) :
			node_handle_(nh), transform_listener_(nh), camera_calibration_path_("robotino_calibration/camera_calibration/"),
			tilt_controller_command_("/tilt_controller/command"), pan_controller_command_("/pan_controller/command"), capture_image_(true)
{
	// load parameters
	std::cout << "\n========== CameraBaseCalibration Parameters ==========\n";
	node_handle_.param("chessboard_cell_size", chessboard_cell_size_, 0.05);
	std::cout << "chessboard_cell_size: " << chessboard_cell_size_ << std::endl;
	chessboard_pattern_size_ = cv::Size(6,4);
	std::vector<double> temp;
	node_handle_.getParam("chessboard_pattern_size", temp);
	if (temp.size() == 2)
		chessboard_pattern_size_ = cv::Size(temp[0], temp[1]);
	std::cout << "pattern: " << chessboard_pattern_size_ << std::endl;
	// coordinate frame name parameters
	node_handle_.param<std::string>("torso_lower_frame", torso_lower_frame_, "base_pan_link");
	std::cout << "torso_lower_frame: " << torso_lower_frame_ << std::endl;
	node_handle_.param<std::string>("torso_upper_frame", torso_upper_frame_, "tilt_link");
	std::cout << "torso_upper_frame: " << torso_upper_frame_ << std::endl;
	node_handle_.param<std::string>("camera_frame", camera_frame_, "kinect_link");
	std::cout << "camera_frame: " << camera_frame_ << std::endl;
	node_handle_.param<std::string>("camera_optical_frame", camera_optical_frame_, "kinect_rgb_optical_frame");
	std::cout << "camera_optical_frame: " << camera_optical_frame_ << std::endl;
	node_handle_.param<std::string>("base_frame", base_frame_, "base_link");
	std::cout << "base_frame: " << base_frame_ << std::endl;
	node_handle_.param<std::string>("checkerboard_frame", checkerboard_frame_, "checkerboard");
	std::cout << "checkerboard_frame: " << checkerboard_frame_ << std::endl;
	// initial parameters
	T_base_to_torso_lower_ = makeTransform(rotationMatrixFromYPR(0.0, 0.0, 0.0), cv::Mat(cv::Vec3d(0.25, 0, 0.5)));
	T_torso_upper_to_camera_ = makeTransform(rotationMatrixFromYPR(0.0, 0.0, -1.57), cv::Mat(cv::Vec3d(0.0, 0.065, 0.0)));
	temp.clear();
	node_handle_.getParam("T_base_to_torso_lower_initial", temp);
	if (temp.size()==6)
		T_base_to_torso_lower_ = makeTransform(rotationMatrixFromYPR(temp[3], temp[4], temp[5]), cv::Mat(cv::Vec3d(temp[0], temp[1], temp[2])));
	std::cout << "T_base_to_torso_lower_initial:\n" << T_base_to_torso_lower_ << std::endl;
	temp.clear();
	node_handle_.getParam("T_torso_upper_to_camera_initial", temp);
	if (temp.size()==6)
		T_torso_upper_to_camera_ = makeTransform(rotationMatrixFromYPR(temp[3], temp[4], temp[5]), cv::Mat(cv::Vec3d(temp[0], temp[1], temp[2])));
	std::cout << "T_torso_upper_to_camera_initial:\n" << T_torso_upper_to_camera_ << std::endl;
	// optimization parameters
	node_handle_.param("optimization_iterations", optimization_iterations_, 100);
	std::cout << "optimization_iterations: " << optimization_iterations_ << std::endl;
	// pan/tilt unit positions and robot base locations relative to checkerboard
	temp.clear();
	node_handle_.getParam("robot_configurations", temp);
	const int number_configurations = temp.size()/5;
	if (temp.size()%5 != 0 || temp.size() < 3*5)
	{
		ROS_ERROR("The robot_configurations vector should contain at least 3 configurations with 5 values each.");
		return;
	}
	std::cout << "Robot configurations:\n";
	for (int i=0; i<number_configurations; ++i)
	{
		robot_configurations_.push_back(RobotConfiguration(temp[5*i], temp[5*i+1], temp[5*i+2], temp[5*i+3], temp[5*i+4]));
		std::cout << temp[5*i] << "\t" << temp[5*i+1] << "\t" << temp[5*i+2] << "\t" << temp[5*i+3] << "\t" << temp[5*i+4] << std::endl;
	}

	// set up messages
	it_ = new image_transport::ImageTransport(node_handle_);
	color_image_sub_.subscribe(*it_, "colorimage_in", 1);
	color_image_sub_.registerCallback(boost::bind(&CameraBaseCalibration::imageCallback, this, _1));

	tilt_controller_ = node_handle_.advertise<std_msgs::Float64>(tilt_controller_command_, 1, false);
	pan_controller_ = node_handle_.advertise<std_msgs::Float64>(pan_controller_command_, 1, false);
	base_controller_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, false);

	ROS_INFO("CameraBaseCalibration initialized.");
}

CameraBaseCalibration::~CameraBaseCalibration()
{
	if (it_ != 0)
		delete it_;
}

bool CameraBaseCalibration::convertImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
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

void CameraBaseCalibration::imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg)
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

bool CameraBaseCalibration::calibrateCameraToBase(const bool load_images)
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
		cv::Mat T_camera_to_checkerboard = T_camera_to_camera_optical_vector[i] * makeTransform(R, tvecs[i]);
		T_camera_to_checkerboard_vector.push_back(T_camera_to_checkerboard);
	}

	// extrinsic calibration between base and torso_lower as well ass torso_upper and camera
	for (int i=0; i<optimization_iterations_; ++i)
	{
//		std::cout << "\nExtrinsic optimization run " << i << ":" << std::endl;
		extrinsicCalibrationBaseToTorsoLower(pattern_points_3d, T_base_to_checkerboard_vector, T_torso_lower_to_torso_upper_vector, T_camera_to_checkerboard_vector);
//		cv::Vec3d rpy = RPYFromRotationMatrix(T_base_to_torso_lower_);
//		std::cout << "T_base_to_torso_lower:\n" << T_base_to_torso_lower_ << std::endl;
//		std::cout << "yaw=" << rpy.val[2] << "   pitch=" << rpy.val[1] << "   roll=" << rpy.val[0] << std::endl;
		extrinsicCalibrationTorsoUpperToCamera(pattern_points_3d, T_base_to_checkerboard_vector, T_torso_lower_to_torso_upper_vector, T_camera_to_checkerboard_vector);
//		rpy = RPYFromRotationMatrix(T_torso_upper_to_camera_);
//		std::cout << "T_torso_upper_to_camera:\n" << T_torso_upper_to_camera_ << std::endl;
//		std::cout << "yaw=" << rpy.val[2] << "   pitch=" << rpy.val[1] << "   roll=" << rpy.val[0] << std::endl;
	}

	// display calibration parameters
	std::cout << "\n\n\n----- Replace these parameters in your 'squirrel_robotino/robotino_bringup/robots/xyz_robotino/urdf/properties.urdf.xacro' file -----\n\n";
	cv::Vec3d rpy = RPYFromRotationMatrix(T_base_to_torso_lower_);
	std::cout << "  <!-- pan_tilt mount positions | handeye calibration | relative to base_link -->\n"
			  << "  <property name=\"pan_tilt_x\" value=\"" << T_base_to_torso_lower_.at<double>(0,3) << "\"/>\n"
			  << "  <property name=\"pan_tilt_y\" value=\"" << T_base_to_torso_lower_.at<double>(1,3) << "\"/>\n"
			  << "  <property name=\"pan_tilt_z\" value=\"" << T_base_to_torso_lower_.at<double>(2,3) << "\"/>\n"
			  << "  <property name=\"pan_tilt_roll\" value=\"" << rpy.val[0] << "\"/>\n"
			  << "  <property name=\"pan_tilt_pitch\" value=\"" << rpy.val[1] << "\"/>\n"
			  << "  <property name=\"pan_tilt_yaw\" value=\"" << rpy.val[2] << "\"/>\n\n";
	rpy = RPYFromRotationMatrix(T_torso_upper_to_camera_);
	std::cout << "  <!-- kinect mount positions | handeye calibration | relative to pan_tilt_link -->\n"
			  << "  <property name=\"kinect_x\" value=\"" << T_torso_upper_to_camera_.at<double>(0,3) << "\"/>\n"
			  << "  <property name=\"kinect_y\" value=\"" << T_torso_upper_to_camera_.at<double>(1,3) << "\"/>\n"
			  << "  <property name=\"kinect_z\" value=\"" << T_torso_upper_to_camera_.at<double>(2,3) << "\"/>\n"
			  << "  <property name=\"kinect_roll\" value=\"" << rpy.val[0] << "\"/>\n"
			  << "  <property name=\"kinect_pitch\" value=\"" << rpy.val[1] << "\"/>\n"
			  << "  <property name=\"kinect_yaw\" value=\"" << rpy.val[2] << "\"/>\n" << std::endl;

	// save calibration
	saveCalibration();
	calibrated_ = true;

	return true;
}

bool CameraBaseCalibration::acquireCalibrationImages(const std::vector<RobotConfiguration>& robot_configurations,
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

int CameraBaseCalibration::acquireCalibrationImage(int& image_width, int& image_height,
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

bool CameraBaseCalibration::moveRobot(const RobotConfiguration& robot_configuration)
{
	// do not move if close to goal
	double error_phi = 10;
	double error_x = 10;
	double error_y = 10;
	cv::Mat T;
	if (!getTransform("checkerboard_reference_nav", "base_link", T))
		return false;
	cv::Vec3d rpy = RPYFromRotationMatrix(T);
	double robot_yaw = rpy.val[2];
	geometry_msgs::Twist tw;
	error_phi = robot_configuration.pose_phi_ - robot_yaw;
	while (error_phi < -CV_PI*0.5)
		error_phi += CV_PI;
	while (error_phi > CV_PI*0.5)
		error_phi -= CV_PI;
	error_x = robot_configuration.pose_x_ - T.at<double>(0,3);
	error_y = robot_configuration.pose_y_ - T.at<double>(1,3);

	std::cout << "error_x=" << error_x << "   error_y=" << error_y << "   error_phi=" << error_phi << std::endl;
	if (fabs(error_phi) > 0.03 || fabs(error_x) > 0.02 || fabs(error_y) > 0.02)
	{
		// control robot angle
		while(true)
		{
			if (!getTransform("checkerboard_reference_nav", "base_link", T))
				return false;
			cv::Vec3d rpy = RPYFromRotationMatrix(T);
				double robot_yaw = rpy.val[2];
			geometry_msgs::Twist tw;
			error_phi = robot_configuration.pose_phi_ - robot_yaw;
			while (error_phi < -CV_PI*0.5)
				error_phi += CV_PI;
			while (error_phi > CV_PI*0.5)
				error_phi -= CV_PI;
			if (fabs(error_phi) < 0.02 || !ros::ok())
				break;
			tw.angular.z = std::min(0.05, error_phi);
			base_controller_.publish(tw);
			ros::Rate(20).sleep();
		}

		// control position
		while(true)
		{
			if (!getTransform("checkerboard_reference_nav", "base_link", T))
				return false;
			geometry_msgs::Twist tw;
			error_x = robot_configuration.pose_x_ - T.at<double>(0,3);
			error_y = robot_configuration.pose_y_ - T.at<double>(1,3);
			if ((fabs(error_x) < 0.01 && fabs(error_y) < 0.01) || !ros::ok())
				break;
//			std::cout << "error_x: " << error_x << std::endl;
//			std::cout << "error_y: " << error_y << std::endl;
			tw.linear.x = std::min(0.05, error_x);
			tw.linear.y = std::min(0.05, error_y);
			base_controller_.publish(tw);
			ros::Rate(20).sleep();
		}

		// control robot angle
		while (true)
		{
			if (!getTransform("checkerboard_reference_nav", "base_link", T))
				return false;
			cv::Vec3d rpy = RPYFromRotationMatrix(T);
				double robot_yaw = rpy.val[2];
			geometry_msgs::Twist tw;
			error_phi = robot_configuration.pose_phi_ - robot_yaw;
			while (error_phi < -CV_PI*0.5)
				error_phi += CV_PI;
			while (error_phi > CV_PI*0.5)
				error_phi -= CV_PI;
			if (fabs(error_phi) < 0.02 || !ros::ok())
				break;
			tw.angular.z = std::min(0.05, error_phi);
			base_controller_.publish(tw);
			ros::Rate(20).sleep();
		}
	}

	std_msgs::Float64 msg;
	msg.data = robot_configuration.pan_angle_;
	pan_controller_.publish(msg);
	msg.data = robot_configuration.tilt_angle_;
	tilt_controller_.publish(msg);

	ros::Duration(3).sleep();

	std::cout << "Positioning successful: x=" << robot_configuration.pose_x_ << ", y=" << robot_configuration.pose_y_
			<< ", phi=" << robot_configuration.pose_phi_ << ", pan=" << robot_configuration.pan_angle_
			<< ", tilt=" << robot_configuration.tilt_angle_ << std::endl;

	return true;
}

void CameraBaseCalibration::computeCheckerboard3dPoints(std::vector< std::vector<cv::Point3f> >& pattern_points, const cv::Size pattern_size, const double chessboard_cell_size, const int number_images)
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

void CameraBaseCalibration::intrinsicCalibration(const std::vector< std::vector<cv::Point3f> >& pattern_points, const std::vector< std::vector<cv::Point2f> >& camera_points_2d_per_image, const cv::Size& image_size, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs)
{
	std::cout << "Intrinsic calibration started ..." << std::endl;
	K_ = cv::Mat::eye(3, 3, CV_64F);
	distortion_ = cv::Mat::zeros(8, 1, CV_64F);
	cv::calibrateCamera(pattern_points, camera_points_2d_per_image, image_size, K_, distortion_, rvecs, tvecs);
	std::cout << "Intrinsic calibration:\nK:\n" << K_ << "\ndistortion:\n" << distortion_ << std::endl;
}

void CameraBaseCalibration::extrinsicCalibrationTorsoUpperToCamera(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
		std::vector<cv::Mat>& T_base_to_checkerboard_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
		std::vector<cv::Mat>& T_camera_to_checkerboard_vector)
{
	// transform 3d chessboard points to respective coordinates systems (camera and torso_upper)
	std::vector<cv::Point3d> points_3d_torso_upper, points_3d_camera;
	for (size_t i=0; i<pattern_points_3d.size(); ++i)
	{
		cv::Mat T_torso_upper_to_checkerboard = T_torso_lower_to_torso_upper_vector[i].inv() * T_base_to_torso_lower_.inv() * T_base_to_checkerboard_vector[i];
//		std::cout << "T_camera_to_checkerboard_vector[i]:\n" << T_camera_to_checkerboard_vector[i] << std::endl;
//		std::cout << "T_torso_upper_to_checkerboard:\n" << T_torso_upper_to_checkerboard << std::endl;
		for (size_t j=0; j<pattern_points_3d[i].size(); ++j)
		{
			cv::Mat point = cv::Mat(cv::Vec4d(pattern_points_3d[i][j].x, pattern_points_3d[i][j].y, pattern_points_3d[i][j].z, 1.0));

			// to camera coordinate system
			cv::Mat point_camera = T_camera_to_checkerboard_vector[i] * point;
			//std::cout << "point_camera=" << point_camera << std::endl;
			points_3d_camera.push_back(cv::Point3d(point_camera.at<double>(0), point_camera.at<double>(1), point_camera.at<double>(2)));

			// to torso_upper coordinate
			cv::Mat point_torso_upper = T_torso_upper_to_checkerboard * point;
			//std::cout << "point_torso_upper=" << point_torso_upper << std::endl;
			points_3d_torso_upper.push_back(cv::Point3d(point_torso_upper.at<double>(0), point_torso_upper.at<double>(1), point_torso_upper.at<double>(2)));
		}
	}

	T_torso_upper_to_camera_ = computeExtrinsicTransform(points_3d_torso_upper, points_3d_camera);
}

void CameraBaseCalibration::extrinsicCalibrationBaseToTorsoLower(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
		std::vector<cv::Mat>& T_base_to_checkerboard_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
		std::vector<cv::Mat>& T_camera_to_checkerboard_vector)
{
	// transform 3d chessboard points to respective coordinates systems (base and torso_lower)
	std::vector<cv::Point3d> points_3d_base, points_3d_torso_lower;
	for (size_t i=0; i<pattern_points_3d.size(); ++i)
	{
		cv::Mat T_torso_lower_to_checkerboard = T_torso_lower_to_torso_upper_vector[i] * T_torso_upper_to_camera_ * T_camera_to_checkerboard_vector[i];
//		std::cout << "T_base_to_checkerboard_vector[" << i << "]:\n" << T_base_to_checkerboard_vector[i] << std::endl;
//		std::cout << "T_torso_lower_to_checkerboard:\n" << T_torso_lower_to_checkerboard << std::endl;
//		std::cout << "T_torso_lower_to_torso_upper_vector[i]:\n" << T_torso_lower_to_torso_upper_vector[i] << std::endl;
//		std::cout << "T_torso_upper_to_camera_:\n" << T_torso_upper_to_camera_ << std::endl;
//		std::cout << "T_camera_to_checkerboard_vector[i]:\n" << T_camera_to_checkerboard_vector[i] << std::endl;
		for (size_t j=0; j<pattern_points_3d[i].size(); ++j)
		{
			cv::Mat point = cv::Mat(cv::Vec4d(pattern_points_3d[i][j].x, pattern_points_3d[i][j].y, pattern_points_3d[i][j].z, 1.0));

			// to camera coordinate system
			cv::Mat point_base = T_base_to_checkerboard_vector[i] * point;
			//std::cout << "point_base: " << pattern_points_3d[i][j].x <<", "<< pattern_points_3d[i][j].y <<", "<< pattern_points_3d[i][j].z << " --> " << point_base.at<double>(0,0) <<", "<< point_base.at<double>(1,0) << ", " << point_base.at<double>(2,0) << std::endl;
			points_3d_base.push_back(cv::Point3d(point_base.at<double>(0), point_base.at<double>(1), point_base.at<double>(2)));

			// to torso_upper coordinate
			cv::Mat point_torso_lower = T_torso_lower_to_checkerboard * point;
			//std::cout << "point_torso_lower: " << pattern_points_3d[i][j].x <<", "<< pattern_points_3d[i][j].y <<", "<< pattern_points_3d[i][j].z << " --> " << point_torso_lower.at<double>(0) <<", "<< point_torso_lower.at<double>(1) << ", " << point_torso_lower.at<double>(2) << std::endl;
			points_3d_torso_lower.push_back(cv::Point3d(point_torso_lower.at<double>(0), point_torso_lower.at<double>(1), point_torso_lower.at<double>(2)));
		}
	}

	T_base_to_torso_lower_ = computeExtrinsicTransform(points_3d_base, points_3d_torso_lower);
}

cv::Mat CameraBaseCalibration::computeExtrinsicTransform(const std::vector<cv::Point3d>& points_3d_source, const std::vector<cv::Point3d>& points_3d_target)
{
	// from: http://nghiaho.com/?page_id=671 : ‘A Method for Registration of 3-D Shapes’, by Besl and McKay, 1992.
	cv::Point3d centroid_source, centroid_target;
	for (size_t i=0; i<points_3d_source.size(); ++i)
	{
		centroid_source += points_3d_source[i];
		centroid_target += points_3d_target[i];
	}
	centroid_source *= 1.0/(double)points_3d_source.size();
	centroid_target *= 1.0/(double)points_3d_target.size();

	// covariance matrix
	cv::Mat M = cv::Mat::zeros(3,3,CV_64FC1);
	for (size_t i=0; i<points_3d_source.size(); ++i)
		M += cv::Mat(points_3d_target[i] - centroid_target)*cv::Mat(points_3d_source[i] - centroid_source).t();

	// SVD on covariance matrix yields rotation
	cv::Mat w, u, vt;
	cv::SVD::compute(M, w, u, vt, cv::SVD::FULL_UV);
	cv::Mat R = vt.t()*u.t();

	// correct reflection matrix cases
	if (cv::determinant(R) < 0)
		for (int r=0; r<3; ++r)
			R.at<double>(r,2) *= -1;

	// translation
	cv::Mat t = -R*cv::Mat(centroid_target) + cv::Mat(centroid_source);

	return makeTransform(R, t);
}

bool CameraBaseCalibration::getTransform(const std::string& target_frame, const std::string& source_frame, cv::Mat& T)
{
	try
	{
		tf::StampedTransform Ts;
		transform_listener_.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
		transform_listener_.lookupTransform(target_frame, source_frame, ros::Time(0), Ts);
		const tf::Matrix3x3& rot = Ts.getBasis();
		const tf::Vector3& trans = Ts.getOrigin();
		cv::Mat rotcv(3,3,CV_64FC1);
		cv::Mat transcv(3,1,CV_64FC1);
		for (int v=0; v<3; ++v)
			for (int u=0; u<3; ++u)
				rotcv.at<double>(v,u) = rot[v].m_floats[u];
		for (int v=0; v<3; ++v)
			transcv.at<double>(v) = trans.m_floats[v];
		T = makeTransform(rotcv, transcv);
		//std::cout << "Transform from " << source_frame << " to " << target_frame << ":\n" << T << std::endl;
	}
	catch (tf::TransformException& ex)
	{
		ROS_WARN("%s",ex.what());
		return false;
	}

	return true;
}

bool CameraBaseCalibration::saveCalibration()
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

bool CameraBaseCalibration::loadCalibration()
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

void CameraBaseCalibration::getCalibration(cv::Mat& K, cv::Mat& distortion, cv::Mat& T_base_to_torso_lower, cv::Mat& T_torso_upper_to_camera)
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

void CameraBaseCalibration::undistort(const cv::Mat& image, cv::Mat& image_undistorted)
{
	if (calibrated_ == false && loadCalibration() == false)
	{
		std::cout << "Error: HandCameraCalibration not calibrated and no calibration data available on disk." << std::endl;
		return;
	}

	cv::undistort(image, image_undistorted, K_, distortion_);
}
