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

#ifndef TRANSFORMATION_UTILITIES_H
#define TRANSFORMATION_UTILITIES_H

// OpenCV
#include <opencv/cv.h>

// Eigen
#include <Eigen/src/Core/Matrix.h>


// compute rotation matrix from yaw, pitch, roll
// (w, p, r) = (yaW, Pitch, Roll) with
// 1. rotation = yaw around z
// 2. rotation = pitch around y'
// 3. rotation = roll around x''
cv::Mat rotationMatrixFromYPR(double yaw, double pitch, double roll)
{
	double sy = sin(yaw);
	double cy = cos(yaw);
	double sp = sin(pitch);
	double cp = cos(pitch);
	double sr = sin(roll);
	double cr = cos(roll);
	cv::Mat rotation = (cv::Mat_<double>(3,3) <<
			cy*cp,		cy*sp*sr - sy*cr,		cy*sp*cr + sy*sr,
			sy*cp,		sy*sp*sr + cy*cr,		sy*sp*cr - cy*sr,
			-sp,		cp*sr,					cp*cr);

	return rotation;
}

// computes yaw, pitch, roll angles from rotation matrix rot (can also be a 4x4 transformation matrix with rotation matrix at upper left corner)
cv::Vec3d YPRFromRotationMatrix(const cv::Mat& rot)
{
	Eigen::Matrix3f rot_eigen;
	for (int i=0; i<3; ++i)
		for (int j=0; j<3; ++j)
			rot_eigen(i,j) = rot.at<double>(i,j);
	Eigen::Vector3f euler_angles = rot_eigen.eulerAngles(2,1,0);
	return cv::Vec3d(euler_angles(0), euler_angles(1), euler_angles(2));
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

#endif	// TRANSFORMATION_UTILITIES_H
