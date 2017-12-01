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

#ifndef _TRANSFORMATION_UTILITIES_H_
#define _TRANSFORMATION_UTILITIES_H_

// OpenCV
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include <tf/transform_listener.h>

namespace transform_utilities
{
	// compute rotation matrix from yaw, pitch, roll
	// (w, p, r) = (yaW, Pitch, Roll) with
	// 1. rotation = yaw around z
	// 2. rotation = pitch around y'
	// 3. rotation = roll around x''
	//cv::Mat rotationMatrixFromYPR(double yaw, double pitch, double roll);

	// computes yaw, pitch, roll angles from rotation matrix rot (can also be a 4x4 transformation matrix with rotation matrix at upper left corner)
	cv::Vec3d YPRFromRotationMatrix(const cv::Mat& rot);

	cv::Mat makeTransform(const cv::Mat& R, const cv::Mat& t);

	//bool stringToTransform(const std::string& values, cv::Mat& trafo); // Takes a string like "1,1,1,1,1,1" and creates a 4x4 transformation matrix out of it.

	// computes the transform from target_frame to source_frame (i.e. transform arrow is pointing from target_frame to source_frame)
	bool getTransform(const tf::TransformListener& transform_listener, const std::string& target_frame, const std::string& source_frame, cv::Mat& T);

	// computes the rigid transform between two sets of corresponding 3d points measured in different coordinate systems
	// the resulting 4x4 transformation matrix converts point coordinates from the target system into the source coordinate system
	cv::Mat computeExtrinsicTransform(const std::vector<cv::Point3d>& points_3d_source, const std::vector<cv::Point3d>& points_3d_target);

}

#endif	// _TRANSFORMATION_UTILITIES_H_
