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
 * Date of creation: August 2016
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

#ifndef __CAMERA_BASE_CALIBRATION_PITAG_H__
#define __CAMERA_BASE_CALIBRATION_PITAG_H__

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

#include <robotino_calibration/calibration_utilities.h>


class CameraBaseCalibrationPiTag
{
public:

	CameraBaseCalibrationPiTag(ros::NodeHandle nh);

	~CameraBaseCalibrationPiTag();

	// starts the calibration between camera and base including data acquisition
	bool calibrateCameraToBase(const bool load_data);

	void setCalibrationStatus(bool calibrated)
	{
		calibrated_ = calibrated;
	}

	// load/save calibration data from/to file
	bool saveCalibration();
	bool loadCalibration();

	void getCalibration(cv::Mat& K, cv::Mat& distortion, cv::Mat& T_base_to_torso_lower, cv::Mat& T_torso_upper_to_camera);

protected:

	bool convertImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);

	void imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg);

	// acquires images automatically from all set up robot configurations and detects the checkerboard points
	// @param load_images loads calibration images and transformations from hard disk if set to true (images and transformations are stored automatically during recording from a real camera)
	// retrieves the image size, checkerboard points per image as well as all relevant transformations
	bool acquireCalibrationData(const std::vector<RobotConfiguration>& robot_configurations, const bool load_data,
			std::vector<cv::Mat>& T_base_to_marker_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
			std::vector<cv::Mat>& T_camera_to_marker_vector);

	// moves the robot to a desired location and adjusts the torso joints
	bool moveRobot(const RobotConfiguration& robot_configuration);

	void extrinsicCalibrationBaseToTorsoLower(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
			std::vector<cv::Mat>& T_base_to_marker_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
			std::vector<cv::Mat>& T_camera_to_marker_vector);

	void extrinsicCalibrationTorsoUpperToCamera(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
			std::vector<cv::Mat>& T_base_to_marker_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
			std::vector<cv::Mat>& T_camera_to_marker_vector);

	// computes the rigid transform between two sets of corresponding 3d points measured in different coordinate systems
	// the resulting 4x4 transformation matrix converts point coordinates from the target system into the source coordinate system
	cv::Mat computeExtrinsicTransform(const std::vector<cv::Point3d>& points_3d_source, const std::vector<cv::Point3d>& points_3d_target);

	// computes the transform from target_frame to source_frame (i.e. transform arrow is pointing from target_frame to source_frame)
	bool getTransform(const std::string& target_frame, const std::string& source_frame, cv::Mat& T);


	ros::NodeHandle node_handle_;

	ros::ServiceClient pitag_client_;

	ros::Publisher base_controller_;
	ros::Publisher tilt_controller_;
	ros::Publisher pan_controller_;

	tf::TransformListener transform_listener_;
	std::string torso_lower_frame_;
	std::string torso_upper_frame_;
	std::string camera_frame_;
	std::string camera_optical_frame_;
	std::string base_frame_;
	std::string marker_frame_base_name_;

	cv::Mat T_base_to_torso_lower_;		// transformation to estimate from base to torso_lower
	cv::Mat T_torso_upper_to_camera_;		// transformation to estimate from torso_upper to camera

	bool calibrated_;	// only true if cameras were calibrated or a calibration was loaded before

	// parameters
	std::string camera_calibration_path_;	// path to data
	std::string tilt_controller_command_;
	std::string pan_controller_command_;

	int optimization_iterations_;	// number of iterations for optimization

	std::vector<RobotConfiguration> robot_configurations_;	// list of robot configurations for observing the checkerboard
};

#endif // __CAMERA_BASE_CALIBRATION_PITAG_H__
