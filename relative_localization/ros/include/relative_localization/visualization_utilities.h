/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2015 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
* \note
* Repository name: squirrel_calibration
* \note
* ROS package name: relative_localization
 *
 * \author
 * Author: Richard Bormann
 * \author
 * Supervised by:
 *
 * \date Date of creation: 10.08.2016
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef VISUALIZATION_UTILITIES_H
#define VISUALIZATION_UTILITIES_H

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

// ROS
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"

class VisualizationUtilities
{
public:
	static void publishWallVisualization(const std_msgs::Header& header, const std::string& name_space, const double px, const double py, const double n0x, const double n0y, const ros::Publisher& marker_pub)
	{
		visualization_msgs::Marker marker;
		//marker.header = header;
		marker.header.frame_id = "base_link";
		marker.ns = name_space;
		marker.id = 0;
		marker.type = visualization_msgs::Marker::LINE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		marker.color.a = 1.0;
		marker.scale.x = 0.025;
		geometry_msgs::Point point;
		point.x = px - 5*n0y;
		point.y = py + 5*n0x;
		point.z = 0;
		marker.points.push_back(point);
		point.x = px + 5*n0y;
		point.y = py - 5*n0x;
		point.z = 0;
		marker.points.push_back(point);
		marker_pub.publish(marker);
	}

	static void publishPointsVisualization(const std_msgs::Header& header, const std::string& name_space, const std::vector<cv::Point2d>& points, const ros::Publisher& marker_pub)
	{
		// display points of box segment
		visualization_msgs::Marker marker;
		//marker.header = header;
		marker.header.frame_id = "base_link";
		marker.ns = name_space;
		marker.id = 1;
		marker.type = visualization_msgs::Marker::SPHERE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
		marker.color.a = 1.0;
		marker.scale.x = 0.05;
		for (size_t i=0; i<points.size(); ++i)
		{
			geometry_msgs::Point point;
			point.x = points[i].x;
			point.y = points[i].y;
			point.z = 0;
			marker.points.push_back(point);
		}
		marker_pub.publish(marker);
	}

	static void publishDetectionPolygon(const std_msgs::Header& header, const std::string& name_space, const std::vector<cv::Point2f>& points, const float height,
			const ros::Publisher& marker_pub, const double red=1.0, const double green=0.0, const double blue=0.0)
	{
		visualization_msgs::Marker marker;
		//marker.header = header;
		marker.header.frame_id = "base_link";
		marker.ns = name_space;
		marker.id = 2;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.color.r = red;
		marker.color.g = green;
		marker.color.b = blue;
		marker.color.a = 1.0;
		marker.scale.x = 0.025;
		for (size_t i=0; i<points.size(); ++i)
		{
			geometry_msgs::Point point;
			point.x = points[i].x;
			point.y = points[i].y;
			point.z = height;
			marker.points.push_back(point);
		}
		marker_pub.publish(marker);
	}
};

#endif	// VISUALIZATION_UTILITIES_H
