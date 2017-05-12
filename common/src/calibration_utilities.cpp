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

namespace calibration_utilities
{
	RobotConfiguration::RobotConfiguration(const double pose_x, const double pose_y, const double pose_phi, const double pan_angle, const double tilt_angle)
	{
		pose_x_ = pose_x;
		pose_y_ = pose_y;
		pose_phi_ = pose_phi;
		pan_angle_ = pan_angle;
		tilt_angle_ = tilt_angle;
	}

	/*EndeffectorConfiguration::EndeffectorConfiguration(const double pose_x, const double pose_y, const double pose_z)
	{
		pose_x_ = pose_x;
		pose_y_ = pose_y;
		pose_z_ = pose_z;
	}*/

	AngleConfiguration::AngleConfiguration(const std::vector<double> angles)
	{
		angles_.clear();
		angles_.insert(angles_.end(), angles.begin(), angles.end());
	}

	/*CameraConfiguration::CameraConfiguration(const double pan_angle, const double tilt_angle)
	{
		pan_angle_ = pan_angle;
		tilt_angle_ = tilt_angle;
	}*/

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
		image = image_ptr->image;

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
}























