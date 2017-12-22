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

#ifndef CAMERA_BASE_CALIBRATION_MARKER_H_
#define CAMERA_BASE_CALIBRATION_MARKER_H_

// ROS
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

// image transport
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <robotino_calibration/calibration_utilities.h>
#include <robotino_calibration/timer.h>
#include <robotino_calibration/robot_calibration.h>

#define RefFrameHistorySize 10 // 10 entries used to build the average upon


class CameraBaseCalibrationMarker : public RobotCalibration
{
public:

	CameraBaseCalibrationMarker(ros::NodeHandle nh);
	virtual ~CameraBaseCalibrationMarker();

	// starts the calibration between camera and base including data acquisition
	virtual bool calibrateCameraToBase(const bool load_images) = 0;


protected:

	// moves the robot to a desired location and adjusts the torso joints
	bool moveRobot(const calibration_utilities::RobotConfiguration& robot_configuration);

	// Turn off base movement
	void turnOffBaseMotion();

	bool isReferenceFrameValid(cv::Mat &T); // Returns wether reference frame is valid -> if so, it is save to move the robot base, otherwise stop!

	void extrinsicCalibration(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
			std::vector<cv::Mat>& T_gapfirst_to_marker_vector, std::vector< std::vector<cv::Mat> > T_between_gaps_vector,
			std::vector<cv::Mat>& T_gaplast_to_marker_vector, int trafo_to_calibrate);

	bool calculateTransformationChains(cv::Mat& T_gapfirst_to_marker, std::vector<cv::Mat> T_between_gaps,
			cv::Mat& T_gaplast_to_marker, std::string marker_frame);

	// displays the calibration result in the urdf file's format and also stores the screen output to a file
	void displayAndSaveCalibrationResult();

	std::string camera_optical_frame_;

	std::vector<calibration_utilities::RobotConfiguration> robot_configurations_;  // wished robot configurations used for calibration
	double RefFrameHistory_[RefFrameHistorySize]; // History of base_frame to reference_frame squared lengths, used to get average squared length. Holds last <RefFrameHistorySize> measurements.
	int RefHistoryIndex_; // Current index of history building
};



#endif // CAMERA_BASE_CALIBRATION_MARKER_H_
