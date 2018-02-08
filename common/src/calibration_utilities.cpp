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
 * Date of creation: October 2016
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


#include <robotino_calibration/calibration_utilities.h>
#include <ros/ros.h>
#include <iostream>
#include <sstream>

namespace calibration_utilities
{
	BaseConfiguration::BaseConfiguration(const std::vector<double> config) :
					pose_x_(0.), pose_y_(0.), pose_phi_(0.)
	{
		if ( config.size() != NUM_BASE_PARAMS )
		{
			std::cout << "RobotConfiguration::BaseConfiguration: Error, passed vector does not have the correct size to build a base configuration!" << std::endl;
			return;
		}

		pose_x_ = config[0];
		pose_y_ = config[1];
		pose_phi_ = config[2];
	}

	void BaseConfiguration::assign(int idx, double value)
	{
		switch( idx )
		{
			case 0:	pose_x_ = value;
					break;
			case 1:	pose_y_ = value;
					break;
			case 2:	pose_phi_ = value;
		}
	}

	std::string BaseConfiguration::get()
	{
		std::stringstream result;
		result << pose_x_ << "/t" << pose_y_ << "/t" << pose_phi_;
		return result.str();
	}

	bool convertImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
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
		image = image_ptr->image.clone();

		return true;
	}

	// generates the 3d coordinates of the checkerboard in local checkerboard frame coordinates
	void computeCheckerboard3dPoints(std::vector< std::vector<cv::Point3f> >& pattern_points, const cv::Size pattern_size, const double chessboard_cell_size, const int number_images)
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

	double computeReprojectionError( const std::vector<std::vector<cv::Point3f> >& objectPoints,
														 const std::vector<std::vector<cv::Point2f> >& imagePoints,
														 const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
														 const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs)
	{
		std::vector<cv::Point2f> imagePoints2;
		size_t totalPoints = 0;
		double totalErr = 0, err = 0;

		for(size_t i = 0; i < objectPoints.size(); ++i )
		{
			cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);

			err = cv::norm(imagePoints[i], imagePoints2, cv::NORM_L2);
			size_t n = objectPoints[i].size();
			//double perViewError = (float) std::sqrt(err*err/n);
			//std::cout << "View error " << (i+1) << ": " << perViewError << std::endl;
			totalErr        += err*err;
			totalPoints     += n;
		}
		return std::sqrt(totalErr/totalPoints);
	}
}























