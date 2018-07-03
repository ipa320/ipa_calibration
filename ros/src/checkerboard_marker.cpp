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
 * Date of creation: June 2018
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


#include <calibration_interface/checkerboard_marker.h>


CheckerboardMarker::CheckerboardMarker()
{

}

void CheckerboardMarker::initialize(ros::NodeHandle nh)
{
	CalibrationMarker::initialize(nh);

	node_handle_.param("checkerboard_cell_size", checkerboard_cell_size_, 0.0);
	std::cout << "checkerboard_cell_size: " << checkerboard_cell_size_ << std::endl;
	checkerboard_pattern_size_ = cv::Size(6,4);
	std::vector<double> temp;
	node_handle_.getParam("checkerboard_pattern_size", temp);
	if (temp.size() == 2)
		checkerboard_pattern_size_ = cv::Size(temp[0], temp[1]);
	std::cout << "pattern: " << checkerboard_pattern_size_ << std::endl;
}

CheckerboardMarker::~CheckerboardMarker()
{

}

void CheckerboardMarker::getPatternPoints3D(std::vector<cv::Point3f> &pattern_points_3d)
{
	pattern_points_3d.clear();
	pattern_points_3d.resize(checkerboard_pattern_size_.height*checkerboard_pattern_size_.width);
	for (int v=0; v<checkerboard_pattern_size_.height; ++v)
	{
		for (int u=0; u<checkerboard_pattern_size_.width; ++u)
			pattern_points_3d[v*checkerboard_pattern_size_.width + u] = cv::Point3f(u*checkerboard_cell_size_, v*checkerboard_cell_size_, 0.f);
	}
}

// static version, so we don't need an object
void CheckerboardMarker::getPatternPoints3D(std::vector<cv::Point3f> &pattern_points_3d, cv::Size pattern_size, double cell_size)
{
	pattern_points_3d.clear();
	pattern_points_3d.resize(pattern_size.height*pattern_size.width);
	for (int v=0; v<pattern_size.height; ++v)
	{
		for (int u=0; u<pattern_size.width; ++u)
			pattern_points_3d[v*pattern_size.width + u] = cv::Point3f(u*cell_size, v*cell_size, 0.f);
	}
}

std::string CheckerboardMarker::getString()
{
	return "checkerboard";
}
