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


#include <robotino_calibration/camera_base_calibration_marker.h>
#include <robotino_calibration/transformation_utilities.h>
#include <robotino_calibration/timer.h>

#include <geometry_msgs/Twist.h>


// ToDo: Remove static camera angle link count of 2 [Done]
// ToDo: Pan_Range and Tilt_Range needs to be stored in one 3*X vector (X number of camera links and 3: min, step, end) [Done]
// ToDo: displayAndSaveCalibrationResult, alter behaviour so that it prints custom strings instead of hardcoded ones. [Done]
// ToDo: Stop robot immediately if reference frame gets lost or jumps around!!!! [Done]
// ToDo: Make that pitag/checkerboard/arm calibration calibration results will be stored to different subfolders [Done, as already possible]
// ToDo: Change convention of rotations from RPY to YPR inside transformations_utilities! Function says YPR already, but it is wrong! [Done, by removing function]
// ToDo: Port over more flexible calibration code to checkerboard calibration as well. [Done]
// ToDo: Remove unused attributes [Done]
// ToDo: Move optimization_iterations to robot_calibration mother class and set its value to one if there is only one transform to be calibrated [Done]
// ToDo: Add timer in moverobot/movearm and check if robots setup has changed since last time (maybe 1 sec), if not give a warning. [Discarded]
// ToDo: Cleanup yaml files [Done]
// ToDo: Port flexible calibration code over to arm calibration as well. [Done]
// ToDo: TF seems to use RPY convention instead of YPR. transform_utilities::rotationMatrixFromYPR is therefore wrong.
// ToDo: Split up yaml files into a calibrtion yaml and an inferface yaml [Done]
// ToDo: make moveCamera a method in base class as all calibration techniques need it [Done]

CameraBaseCalibrationMarker::CameraBaseCalibrationMarker(ros::NodeHandle nh, CalibrationInterface* interface) :
	RobotCalibration(nh, interface)
{
	// Debug how RVIZ rotations are defined
	/*cv::Mat T;

	transform_utilities::getTransform(transform_listener_, "arm_link5", base_frame_, T);
	std::cout << "TF: " << T << std::endl;
	std::cout << transform_utilities::YPRFromRotationMatrix( (cv::Mat_<double>(3,3) << T.at<double>(0,0), T.at<double>(0,1), T.at<double>(0,2),
				T.at<double>(1,0), T.at<double>(1,1), T.at<double>(1,2),
				T.at<double>(2,0), T.at<double>(2,1), T.at<double>(2,2)) ) << std::endl;
	std::vector<float> temp;
	node_handle_.getParam("T_initial", temp);
	T = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(temp[3], temp[4], temp[5]), cv::Mat(cv::Vec3d(temp[0], temp[1], temp[2])));
	std::cout << "BUILT: " << T << std::endl;
	std::cout << transform_utilities::YPRFromRotationMatrix( (cv::Mat_<double>(3,3) << T.at<double>(0,0), T.at<double>(0,1), T.at<double>(0,2),
				T.at<double>(1,0), T.at<double>(1,1), T.at<double>(1,2),
				T.at<double>(2,0), T.at<double>(2,1), T.at<double>(2,2)) ) << std::endl;*/

	// load parameters
	std::cout << "\n========== CameraBaseCalibrationMarker Parameters ==========\n";



	std::cout << "CameraBaseCalibrationMarker: init done." << std::endl;
}

CameraBaseCalibrationMarker::~CameraBaseCalibrationMarker()
{
}
