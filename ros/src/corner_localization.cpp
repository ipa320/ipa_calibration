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

#include <relative_localization/corner_localization.h>
#include <relative_localization/visualization_utilities.h>
#include <relative_localization/relative_localization_utilities.h>

CornerLocalization::CornerLocalization(ros::NodeHandle& nh)
		: ReferenceLocalization(nh)
{
	// load subclass parameters
	node_handle_.param("max_wall_side_distance", max_wall_side_distance_, 0.5);
	std::cout << "max_wall_side_distance: " << max_wall_side_distance_ << std::endl;

	ROS_INFO("CornerLocalization: Initialized.");
}

CornerLocalization::~CornerLocalization()
{
}

//#define DEBUG_OUTPUT
void CornerLocalization::callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg)
{
	// Retrieve points from side and front wall and put each of those in separate lists
	std::vector<cv::Point2d> scan_front;
	std::vector<cv::Point2d> scan_all;

	for ( size_t i=0; i<laser_scan_msg->ranges.size(); i++ )
	{
		double angle = laser_scan_msg->angle_min + i * laser_scan_msg->angle_increment; // [rad]
		double dist = laser_scan_msg->ranges[i];
		cv::Point2d point(dist*cos(angle), dist*sin(angle));

		if (point.y > -wall_length_right_ && point.y < wall_length_left_) // front wall points
			scan_front.push_back(point);

		// store all points in here. Use distant measure to front wall later on to extract side wall points
		if (point.y > -2*wall_length_right_ && point.y < 2*wall_length_left_) // front wall points
			scan_all.push_back(point);
	}

	// match line to scan_front
	const double inlier_distance = 0.01;
	cv::Vec4d line_front;
	RelativeLocalizationUtilities::fitLine(scan_front, line_front, 0.1, 0.99999, inlier_distance, false);
	if (line_front.val[0] != line_front.val[0] || line_front.val[1] != line_front.val[1] || line_front.val[2] != line_front.val[2] || line_front.val[3] != line_front.val[3])
	{
		ROS_WARN("CornerLocalization::callback: frontal wall could not be estimated.");
		return;
	}

	const double px_f = line_front.val[0];	// coordinates of a point on front the wall
	const double py_f = line_front.val[1];
	const double n0x_f = line_front.val[2];	// normal direction on the front wall (in floor plane x-y)
	const double n0y_f = line_front.val[3];

	if (marker_pub_.getNumSubscribers() > 0)
		VisualizationUtilities::publishWallVisualization(laser_scan_msg->header, "wall_front", px_f, py_f, n0x_f, n0y_f, marker_pub_);

	std::vector<cv::Point2d> scan_side;
	for ( size_t i=0; i<scan_all.size(); i++ )
	{
		const double d = RelativeLocalizationUtilities::distanceToLine(px_f, py_f, n0x_f, n0y_f, scan_all[i].x, scan_all[i].y);	// distance to front wall line
		if (d > 2*inlier_distance)
			scan_side.push_back(scan_all[i]);
	}

	if (scan_side.size() < 2)
	{
		ROS_WARN("CornerLocalization::callback: no points left for estimating side wall.");
		return;
	}

	// search a side wall until one is found, which is not too distant and approximately perpendicular to the first
	cv::Vec4d line_side;
	for (int i=0; i<10; ++i)
	{
		// match line to scan_side
		RelativeLocalizationUtilities::fitLine(scan_side, line_side, 0.1, 0.99999, inlier_distance, false);
		if (line_side.val[0] != line_side.val[0] || line_side.val[1] != line_side.val[1] || line_side.val[2] != line_side.val[2] || line_side.val[3] != line_side.val[3])
		{
			ROS_WARN("CornerLocalization::callback: side wall could not be estimated in trial %i. Trying next.", i);
			continue;
		}

		// check if line is good enough
		const double wall_distance_to_laser_scanner = RelativeLocalizationUtilities::distanceToLine(line_side.val[0], line_side.val[1], line_side.val[2], line_side.val[3], 0., 0.);
		const double scalar_product = n0x_f*line_side.val[2] + n0y_f*line_side.val[3];
		if (wall_distance_to_laser_scanner < max_wall_side_distance_ && fabs(scalar_product) < 0.05)
			break;

		// remove points from last line
		for (std::vector<cv::Point2d>::iterator it = scan_side.begin(); it!=scan_side.end(); )
		{
			const double dist = RelativeLocalizationUtilities::distanceToLine(line_side.val[0], line_side.val[1], line_side.val[2], line_side.val[3], it->x, it->y);
			if (dist <= inlier_distance)
				it = scan_side.erase(it);
			else
				++it;
		}
	}

	const double px_s = line_side.val[0];	// coordinates of a point on the side wall
	const double py_s = line_side.val[1];
	const double n0x_s = line_side.val[2];	// normal direction on the side wall (in floor plane x-y)
	const double n0y_s = line_side.val[3];

	// display line
	if (marker_pub_.getNumSubscribers() > 0)
		VisualizationUtilities::publishWallVisualization(laser_scan_msg->header, "wall_side", px_s, py_s, n0x_s, n0y_s, marker_pub_);

	// compute intersection of two wall segments
	cv::Point2d corner_point;
	const double a = n0x_f*px_f+n0y_f*py_f;
	const double b = n0x_s*px_s+n0y_s*py_s;
	corner_point.y = (a*n0x_s-b*n0x_f) / (n0y_f*n0x_s-n0x_f*n0y_s);
	if (fabs(n0x_f) > fabs(n0x_s))
		corner_point.x = (a-n0y_f*corner_point.y) / n0x_f;
	else
		corner_point.x = (b-n0y_s*corner_point.y) / n0x_s;
	if (corner_point.x == 0 && corner_point.y == 0)
		return;

	// display points of box segment
	std::vector<cv::Point2d> corner_point_vec(1, corner_point);
	if (marker_pub_.getNumSubscribers() > 0)
		VisualizationUtilities::publishPointsVisualization(laser_scan_msg->header, "corner_point", corner_point_vec, marker_pub_);

#ifdef DEBUG_OUTPUT
	std::cout << "Corner point: " << corner_point << std::endl;
#endif

	// determine coordinate system generated by corner at the intersection of two walls (laser scanner coordinate system: x=forward, y=left, z=up)
	// corner coordinate system is attached to the corner of the the walls
	bool publish_tf = true;
	tf::StampedTransform transform_table_reference;
	// offset to laser scanner coordinate system
	// intersection of line from left block point in normal direction with wall line
	double j = ((corner_point.x-px_f)*n0y_f + (py_f-corner_point.y)*n0x_f) / (n0x_f*n0x_f + n0y_f*n0y_f);
	double x = px_f + j*n0y_f;
	double y = py_f - j*n0x_f;
	tf::Vector3 translation(x, y, 0.);
	// direction of x-axis in laser scanner coordinate system
	cv::Point2d normal(n0x_f, n0y_f);
	if (normal.x*translation.getX() + normal.y*translation.getY() < 0)
		normal *= -1.;
	double angle = atan2(normal.y, normal.x);
	tf::Quaternion orientation(tf::Vector3(0,0,1), angle);

	// update transform
	if (avg_translation_.isZero())
	{
		// use value directly on first message
		avg_translation_ = translation;
		avg_orientation_ = orientation;
	}
	else
	{
		// update value
		avg_translation_ = (1.0 - update_rate_) * avg_translation_ + update_rate_ * translation;
		avg_orientation_.setW((1.0 - update_rate_) * avg_orientation_.getW() + update_rate_ * orientation.getW());
		avg_orientation_.setX((1.0 - update_rate_) * avg_orientation_.getX() + update_rate_ * orientation.getX());
		avg_orientation_.setY((1.0 - update_rate_) * avg_orientation_.getY() + update_rate_ * orientation.getY());
		avg_orientation_.setZ((1.0 - update_rate_) * avg_orientation_.getZ() + update_rate_ * orientation.getZ());
	}

	// transform
	transform_table_reference.setOrigin(avg_translation_);
	transform_table_reference.setRotation(avg_orientation_);
	tf::StampedTransform tf_msg(transform_table_reference, laser_scan_msg->header.stamp, laser_scan_msg->header.frame_id, child_frame_name_);
	if (reference_coordinate_system_at_ground_ == true)
		ShiftReferenceFrameToGround(tf_msg);

	// publish coordinate system on tf
	if (publish_tf == true)
	{
		transform_broadcaster_.sendTransform(tf_msg);
	}
}

void CornerLocalization::dynamicReconfigureCallback(robotino_calibration::RelativeLocalizationConfig &config, uint32_t level)
{
	ReferenceLocalization::dynamicReconfigureCallback(config, level);
	max_wall_side_distance_ = config.max_wall_side_distance;
	std::cout << "max_wall_side_distance_=" << max_wall_side_distance_ << "\n";
}
