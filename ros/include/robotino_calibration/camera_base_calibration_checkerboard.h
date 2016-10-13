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

#ifndef __CAMERA_BASE_CALIBRATION_CHECKERBOARD_H__
#define __CAMERA_BASE_CALIBRATION_CHECKERBOARD_H__


#include <robotino_calibration/camera_base_calibration_marker.h>


class CameraBaseCalibrationCheckerboard : public CameraBaseCalibrationMarker
{
public:

	CameraBaseCalibrationCheckerboard(ros::NodeHandle nh);
	~CameraBaseCalibrationCheckerboard();

	// starts the calibration between camera and base including data acquisition
	bool calibrateCameraToBase(const bool load_images);

	// load/save calibration data from/to file
	bool saveCalibration();
	bool loadCalibration();
	void getCalibration(cv::Mat& K, cv::Mat& distortion, cv::Mat& T_base_to_torso_lower, cv::Mat& T_torso_upper_to_camera);

	void undistort(const cv::Mat& image, cv::Mat& image_undistorted);


protected:

	void imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg);

	// acquires images automatically from all set up robot configurations and detects the checkerboard points
	// @param load_images loads calibration images and transformations from hard disk if set to true (images and transformations are stored automatically during recording from a real camera)
	// retrieves the image size, checkerboard points per image as well as all relevant transformations
	bool acquireCalibrationImages(const std::vector<calibration_utilities::RobotConfiguration>& robot_configurations, const cv::Size pattern_size, const bool load_images,
			int& image_width, int& image_height, std::vector< std::vector<cv::Point2f> >& points_2d_per_image,
			std::vector<cv::Mat>& T_base_to_checkerboard_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
			std::vector<cv::Mat>& T_camera_to_camera_optical_vector);

	// acquire a single image and detect checkerboard points
	int acquireCalibrationImage(int& image_width, int& image_height, std::vector<cv::Point2f>& points_2d_per_image,
			const cv::Size pattern_size, const bool load_images, int& image_counter);

	// intrinsic camera calibration (+ distortion coefficients)
	void intrinsicCalibration(const std::vector< std::vector<cv::Point3f> >& pattern_points, const std::vector< std::vector<cv::Point2f> >& camera_points_2d_per_image, const cv::Size& image_size, std::vector<cv::Mat>& rvecs_jai, std::vector<cv::Mat>& tvecs_jai);


	std::string checkerboard_frame_;

	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter color_image_sub_; ///< Color camera image input topic
	boost::mutex camera_data_mutex_;	// secures read and write operations on camera data
	cv::Mat camera_image_;		// stores the latest camera image
	ros::Time latest_image_time_;	// stores time stamp of latest image
	bool capture_image_;

	cv::Mat K_;			// intrinsic matrix for camera
	cv::Mat distortion_;	// distortion parameters for camera

	double chessboard_cell_size_;	// cell side length in [m]
	cv::Size chessboard_pattern_size_;		// number of checkerboard corners in x and y direction
};

#endif // __CAMERA_BASE_CALIBRATION_CHECKERBOARD_H__
