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

#ifndef ARM_BASE_CALIBRATION_H_
#define ARM_BASE_CALIBRATION_H_

// ROS
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
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


class ArmBaseCalibration
{
public:

	ArmBaseCalibration(ros::NodeHandle nh);
	~ArmBaseCalibration();
	bool calibrateArmToBase(const bool load_images);
	bool saveCalibration();
	bool loadCalibration();
	void getCalibration(cv::Mat& T_base_to_armbase, cv::Mat& T_endeff_to_checkerboard);
	void setCalibrationStatus(bool calibrated);


protected:

	bool moveArm(const RobotConfiguration& arm_configuration);

	void extrinsicCalibrationBaseToArm(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
			std::vector<cv::Mat>& T_base_to_checkerboard_vector, std::vector<cv::Mat>& T_armbase_to_endeff_vector);

	void extrinsicCalibrationEndeffToCheckerboard(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
				std::vector<cv::Mat>& T_base_to_checkerboard_vector, std::vector<cv::Mat>& T_armbase_to_endeff_vector);

	bool acquireCalibrationImages(const std::vector<RobotConfiguration>& robot_configurations,
			const cv::Size pattern_size, const bool load_images, int& image_width, int& image_height,
			std::vector< std::vector<cv::Point2f> >& points_2d_per_image, std::vector<cv::Mat>& T_base_to_checkerboard_vector,
			std::vector<cv::Mat>& T_armbase_to_endeff_vector);
	int acquireCalibrationImage(int& image_width, int& image_height,
			std::vector<cv::Point2f>& checkerboard_points_2d, const cv::Size pattern_size, const bool load_images, int& image_counter);

	void imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg);
	bool convertImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);

	// generates the 3d coordinates of the checkerboard in local checkerboard frame coordinates
	void computeCheckerboard3dPoints(std::vector< std::vector<cv::Point3f> >& pattern_points, const cv::Size pattern_size, const double chessboard_cell_size, const int number_images);

	// displays the calibration result in the urdf file's format and also stores the screen output to a file
	void displayAndSaveCalibrationResult(const cv::Mat& T_base_to_arm_);


private:

	ros::NodeHandle node_handle_;
	/*ros::Publisher base_controller_;
	ros::Publisher tilt_controller_;
	ros::Publisher pan_controller_;
	ros::Subscriber pan_tilt_state_;

	sensor_msgs::JointState* pan_tilt_joint_state_current_;
	boost::mutex pan_tilt_joint_state_data_mutex_;	// secures read operations on pan tilt joint state data
*/
	tf::TransformListener transform_listener_;
	//std::vector<std::string> arm_frames_; // list of all arms links
	std::string base_frame_;
	std::string checkerboard_frame_;
	std::string armbase_frame_;
	std::string endeff_frame_;

	cv::Mat T_base_to_armbase_;		// transformation to estimate from base to first link of arm
	cv::Mat T_endeff_to_checkerboard_;

	bool calibrated_;	// only true if cameras were calibrated or a calibration was loaded before

	// parameters
	std::string arm_calibration_path_;	// path to data
	/*std::string tilt_controller_command_;
	std::string pan_controller_command_;*/

	int optimization_iterations_;	// number of iterations for optimization

	std::vector<RobotConfiguration> arm_configurations_;	// list of robot configurations for observing the checkerboard

	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter color_image_sub_; ///< Color camera image input topic
	boost::mutex camera_data_mutex_;	// secures read and write operations on camera data
	cv::Mat camera_image_;		// stores the latest camera image
	ros::Time latest_image_time_;	// stores time stamp of latest image
	bool capture_image_;

	//cv::Mat K_;			// intrinsic matrix for camera
	//cv::Mat distortion_;	// distortion parameters for camera

	double chessboard_cell_size_;	// cell side length in [m]
	cv::Size chessboard_pattern_size_;		// number of checkerboard corners in x and y direction
};


#endif //ARM_BASE_CALIBRATION_H_
