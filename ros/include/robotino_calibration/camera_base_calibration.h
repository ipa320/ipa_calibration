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

#ifndef __CAMERA_BASE_CALIBRATION_H__
#define __CAMERA_BASE_CALIBRATION_H__

// ROS
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

// image transport
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// OpenCV
#include <opencv/cv.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Boost
#include <boost/thread/mutex.hpp>


// compute rotation matrix from roll, pitch, yaw
// (w, p, r) = (yaW, Pitch, Roll) with
// 1. rotation = roll around z
// 2. rotation = pitch around y'
// 3. rotation = yaw around x''
// ATTENTION: this definition does not match with the ROS definition (1. yaw around z, 2. pitch around y', 3. roll around x'') -> swap roll and yaw in function call
cv::Mat rotationMatrixFromRPY(double roll, double pitch, double yaw);

// computes roll, pitch, yaw angles from rotation matrix rot (can also be a 4x4 transformation matrix with rotation matrix at upper left corner)
cv::Vec3d RPYFromRotationMatrix(const cv::Mat& rot);

cv::Mat makeTransform(const cv::Mat& R, const cv::Mat& t);


// struct for the configuration (x,y location + base rotation, torso links) of the robot
struct RobotConfiguration
{
	double pose_x_;
	double pose_y_;
	double pose_phi_;
	double pan_angle_;
	double tilt_angle_;

	RobotConfiguration(const double pose_x, const double pose_y, const double pose_phi, const double pan_angle, const double tilt_angle)
	{
		pose_x_ = pose_x;
		pose_y_ = pose_y;
		pose_phi_ = pose_phi;
		pan_angle_ = pan_angle;
		tilt_angle_ = tilt_angle;
	}
};

class CameraBaseCalibration
{
public:

	CameraBaseCalibration(ros::NodeHandle nh);

	~CameraBaseCalibration();

	// starts the calibration between camera and base including data acquisition
	bool calibrateCameraToBase(const bool load_images);

	void setCalibrationStatus(bool calibrated)
	{
		calibrated_ = calibrated;
	}

	// load/save calibration data from/to file
	bool saveCalibration();
	bool loadCalibration();

	void getCalibration(cv::Mat& K, cv::Mat& distortion, cv::Mat& T_base_to_torso_lower, cv::Mat& T_torso_upper_to_camera);

	void undistort(const cv::Mat& image, cv::Mat& image_undistorted);

protected:

	bool convertImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);

	void imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg);

	// acquires images automatically from all set up robot configurations and detects the checkerboard points
	// @param load_images loads calibration images and transformations from hard disk if set to true (images and transformations are stored automatically during recording from a real camera)
	// retrieves the image size, checkerboard points per image as well as all relevant transformations
	bool acquireCalibrationImages(const std::vector<RobotConfiguration>& robot_configurations, const cv::Size pattern_size, const bool load_images,
			int& image_width, int& image_height, std::vector< std::vector<cv::Point2f> >& points_2d_per_image,
			std::vector<cv::Mat>& T_base_to_checkerboard_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
			std::vector<cv::Mat>& T_camera_to_camera_optical_vector);

	// acquire a single image and detect checkerboard points
	int acquireCalibrationImage(int& image_width, int& image_height, std::vector<cv::Point2f>& points_2d_per_image,
			const cv::Size pattern_size, const bool load_images, int& image_counter);

	// moves the robot to a desired location and adjusts the torso joints
	bool moveRobot(const RobotConfiguration& robot_configuration);

	// generates the 3d coordinates of the checkerboard in local checkerboard frame coordinates
	void computeCheckerboard3dPoints(std::vector< std::vector<cv::Point3f> >& pattern_points, const cv::Size pattern_size, const double chessboard_cell_size, const int number_images);

	// intrinsic camera calibration (+ distortion coefficients)
	void intrinsicCalibration(const std::vector< std::vector<cv::Point3f> >& pattern_points, const std::vector< std::vector<cv::Point2f> >& camera_points_2d_per_image, const cv::Size& image_size, std::vector<cv::Mat>& rvecs_jai, std::vector<cv::Mat>& tvecs_jai);

	void extrinsicCalibrationBaseToTorsoLower(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
			std::vector<cv::Mat>& T_base_to_checkerboard_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
			std::vector<cv::Mat>& T_camera_to_checkerboard_vector);

	void extrinsicCalibrationTorsoUpperToCamera(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
			std::vector<cv::Mat>& T_base_to_checkerboard_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
			std::vector<cv::Mat>& T_camera_to_checkerboard_vector);

	// computes the rigid transform between two sets of corresponding 3d points measured in different coordinate systems
	// the resulting 4x4 transformation matrix converts point coordinates from the target system into the source coordinate system
	cv::Mat computeExtrinsicTransform(const std::vector<cv::Point3d>& points_3d_source, const std::vector<cv::Point3d>& points_3d_target);

	// computes the transform from target_frame to source_frame (i.e. transform arrow is pointing from target_frame to source_frame)
	bool getTransform(const std::string& target_frame, const std::string& source_frame, cv::Mat& T);


	ros::NodeHandle node_handle_;
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter color_image_sub_; ///< Color camera image input topic
	boost::mutex camera_data_mutex_;	// secures read and write operations on camera data
	cv::Mat camera_image_;		// stores the latest camera image
	ros::Time latest_image_time_;	// stores time stamp of latest image
	bool capture_image_;

	ros::Publisher base_controller_;
	ros::Publisher tilt_controller_;
	ros::Publisher pan_controller_;

	tf::TransformListener transform_listener_;
	std::string torso_lower_frame_;
	std::string torso_upper_frame_;
	std::string camera_frame_;
	std::string camera_optical_frame_;
	std::string base_frame_;
	std::string checkerboard_frame_;

	cv::Mat K_;			// intrinsic matrix for camera
	cv::Mat distortion_;	// distortion parameters for camera

	cv::Mat T_base_to_torso_lower_;		// transformation to estimate from base to torso_lower
	cv::Mat T_torso_upper_to_camera_;		// transformation to estimate from torso_upper to camera

	bool calibrated_;	// only true if cameras were calibrated or a calibration was loaded before

	// parameters
	std::string camera_calibration_path_;	// path to data
	std::string tilt_controller_command_;
	std::string pan_controller_command_;

	double chessboard_cell_size_;	// cell side length in [m]
	cv::Size chessboard_pattern_size_;		// number of checkerboard corners in x and y direction

	int optimization_iterations_;	// number of iterations for optimization

	std::vector<RobotConfiguration> robot_configurations_;	// list of robot configurations for observing the checkerboard
};

#endif // __CAMERA_BASE_CALIBRATION_H__
