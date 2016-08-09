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
* Repository name: squirrel_robotino
* \note
* ROS package name: checkerboard_localisation
 *
 * \author
 * Author: Richard Bormann
 * \author
 * Supervised by:
 *
 * \date Date of creation: 11.03.2015
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

#ifndef CHECKERBOARD_LOCALISATION_H
#define CHECKERBOARD_LOCALISATION_H

#include <iostream>

// ROS
#include "ros/ros.h"

// messages
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"

// tf
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <robotino_calibration/CheckerboardLocalisationConfig.h>

// OpenCV
#include <opencv/cv.h>


class CheckerboardLocalization
{
public:
	CheckerboardLocalization(ros::NodeHandle& nh);
	~CheckerboardLocalization();

	struct Point2d
	{
		double x;
		double y;

		Point2d()
		{
			x = 0.;
			y = 0.;
		}

		Point2d(double x_, double y_)
		{
			x = x_;
			y = y_;
		}

		friend std::ostream& operator<<(std::ostream& os, const Point2d& p)
		{
			os << "(" << p.x << "," << p.y << ")";
			return os;
		}

		void operator+=(const Point2d& p)
		{
			x += p.x;
			y += p.y;
		}

		void operator/=(const double div)
		{
			x /= div;
			y /= div;
		}

		void operator*=(const double fac)
		{
			x *= fac;
			y *= fac;
		}
	};


private:

	void callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg);
	void dynamicReconfigureCallback(robotino_calibration::CheckerboardLocalisationConfig& config, uint32_t level);
	void fitLine(const std::vector<cv::Point2d>& points, cv::Vec4d& line, double inlier_ratio, double success_probability, double max_inlier_distance, bool draw_from_both_halves_of_point_set=false);
	void publishWallVisualization(const std_msgs::Header& header, const double px, const double py, const double n0x, const double n0y);
	void publishBoxPoints(const std_msgs::Header& header, const std::vector< std::vector<cv::Point2d> >& segments, const size_t largest_segment);

	ros::NodeHandle node_handle_;
	ros::Subscriber laser_scan_sub_;
	ros::Publisher marker_pub_;

	tf::TransformBroadcaster transform_broadcaster_;

	dynamic_reconfigure::Server<robotino_calibration::CheckerboardLocalisationConfig> dynamic_reconfigure_server_;
	tf::Vector3 avg_translation_;
	tf::Quaternion avg_orientation_;
	double update_rate_;
	std::string child_frame_name_;

	// parameters
	double wall_length_left_;		// the length of the wall segment left of the checkerboard's origin, in[m]
	double wall_length_right_;		// the length of the wall segment right of the checkerboard's origin, in[m]
	double box_search_width_;		// the maximum +/-y coordinate in laser scan to search for the localization box, in[m]
};

#endif // CHECKERBOARD_LOCALISATION_H
