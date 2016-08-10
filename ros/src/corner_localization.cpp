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
		: node_handle_(nh)
{
	// load parameters
	std::cout << "\n========== Corner Localization Parameters ==========\n";
	node_handle_.param("update_rate", update_rate_, 0.75);
	std::cout << "update_rate: " << update_rate_ << std::endl;
	node_handle_.param<std::string>("child_frame_name", child_frame_name_);
	std::cout << "child_frame_name: " << child_frame_name_ << std::endl;
	node_handle_.param("wall_length_left", wall_length_left_, 0.75);
	std::cout << "wall_length_left: " << wall_length_left_ << std::endl;
	node_handle_.param("wall_length_right", wall_length_right_, 0.75);
	std::cout << "wall_length_right: " << wall_length_right_ << std::endl;
	node_handle_.param("max_wall_side_distance", max_wall_side_distance_, 0.5);
	std::cout << "max_wall_side_distance: " << max_wall_side_distance_ << std::endl;

	// publishers
	marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("wall_marker", 1);

	// subscribers
	laser_scan_sub_ = node_handle_.subscribe("laser_scan_in", 0, &CornerLocalization::callback, this);

	// dynamic reconfigure
	dynamic_reconfigure_server_.setCallback(boost::bind(&CornerLocalization::dynamicReconfigureCallback, this, _1, _2));
	avg_translation_.setZero();
}

//#define DEBUG_OUTPUT
void CornerLocalization::callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg)
{
	const double inlier_distance = 0.01;

	// 1. frontal wall
	// convert scan to x-y coordinates
	std::vector<cv::Point2d> scan_frontal;
	for (unsigned int i = 0; i < laser_scan_msg->ranges.size(); ++i)
	{
		double angle = laser_scan_msg->angle_min + i * laser_scan_msg->angle_increment; //[rad]
		double dist = laser_scan_msg->ranges[i];
		cv::Point2d point(dist*cos(angle), dist*sin(angle));
		if (point.y > -wall_length_right_ && point.y < wall_length_left_)
			scan_frontal.push_back(point);
	}

	// match line to scan_frontal
	cv::Vec4d line_front;
	RelativeLocalizationUtilities::fitLine(scan_frontal, line_front, 0.1, 0.99999, inlier_distance, false);
	if (line_front.val[0] != line_front.val[0] || line_front.val[1] != line_front.val[1] || line_front.val[2] != line_front.val[2] || line_front.val[3] != line_front.val[3])
		return;

	// display line
	const double px = line_front.val[0];	// coordinates of a point on the wall
	const double py = line_front.val[1];
	const double n0x = line_front.val[2];	// normal direction on the wall (in floor plane x-y)
	const double n0y = line_front.val[3];
	if (marker_pub_.getNumSubscribers() > 0)
		VisualizationUtilities::publishWallVisualization(laser_scan_msg->header, "wall_front", px, py, n0x, n0y, marker_pub_);

	// 2. side wall
	// convert scan to x-y coordinates
	std::vector<cv::Point2d> scan;
	for (unsigned int i = 0; i < laser_scan_msg->ranges.size(); ++i)
	{
		double angle = laser_scan_msg->angle_min + i * laser_scan_msg->angle_increment; //[rad]
		double dist = laser_scan_msg->ranges[i];
		cv::Point2d point(dist*cos(angle), dist*sin(angle));
		const double d = RelativeLocalizationUtilities::distanceToLine(px, py, n0x, n0y, point.x, point.y);		// remove frontal wall from set
		if (d > 2*inlier_distance)
			scan.push_back(point);
	}

	// search a side wall until one is found, which is not too distant and approximately perpendicular to the first
	cv::Vec4d line_side;
	for (int i=0; i<10; ++i)
	{
		// match line to scan_frontal
		RelativeLocalizationUtilities::fitLine(scan, line_side, 0.1, 0.99999, inlier_distance, false);
		if (line_side.val[0] != line_side.val[0] || line_side.val[1] != line_side.val[1] || line_side.val[2] != line_side.val[2] || line_side.val[3] != line_side.val[3])
			return;

		// check line
		const double wall_distance_to_laser_scanner = RelativeLocalizationUtilities::distanceToLine(line_side.val[0], line_side.val[1], line_side.val[2], line_side.val[3], 0., 0.);
		const double scalar_product = n0x*line_side.val[2] + n0y*line_side.val[3];
		if (wall_distance_to_laser_scanner < max_wall_side_distance_ && fabs(scalar_product) < 0.05)
			break;

		// remove points from last line
		for (std::vector<cv::Point2d>::iterator it = scan.begin(); it!=scan.end(); )
		{
			const double dist = RelativeLocalizationUtilities::distanceToLine(line_side.val[0], line_side.val[1], line_side.val[2], line_side.val[3], it->x, it->y);
			if (dist <= inlier_distance)
				it = scan.erase(it);
			else
				++it;
		}
	}
	// display line
	if (marker_pub_.getNumSubscribers() > 0)
		VisualizationUtilities::publishWallVisualization(laser_scan_msg->header, "wall_side", line_side.val[0], line_side.val[1], line_side.val[2], line_side.val[3], marker_pub_);


	// compute intersection of two wall segments
	cv::Point2d corner_point;
	const double a = n0x*px+n0y*py;
	const double b = line_side.val[2]*line_side.val[0] + line_side.val[3]*line_side.val[1];
	corner_point.y = (a*line_side.val[2]-b*n0x) / (n0y*line_side.val[2]-n0x*line_side.val[3]);
	if (fabs(n0x) > fabs(line_side.val[2]))
		corner_point.x = (a-n0y*corner_point.y) / n0x;
	else
		corner_point.x = (b-line_side.val[3]*corner_point.y) / line_side.val[2];
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
	double j = ((corner_point.x-px)*n0y + (py-corner_point.y)*n0x) / (n0x*n0x + n0y*n0y);
	double x = px + j*n0y;
	double y = py - j*n0x;
	tf::Vector3 translation(x, y, 0.);
	// direction of x-axis in laser scanner coordinate system
	cv::Point2d normal(n0x, n0y);
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

	// publish coordinate system on tf
	if (publish_tf == true)
	{
		transform_broadcaster_.sendTransform(tf::StampedTransform(transform_table_reference, laser_scan_msg->header.stamp, laser_scan_msg->header.frame_id, child_frame_name_));
	}
}

void CornerLocalization::dynamicReconfigureCallback(robotino_calibration::CheckerboardLocalisationConfig &config, uint32_t level)
{
	update_rate_ = config.update_rate;
	child_frame_name_ = config.child_frame_name;
	wall_length_left_ = config.wall_length_left;
	wall_length_right_ = config.wall_length_right;
	max_wall_side_distance_ = config.max_wall_side_distance;
	std::cout << "Reconfigure request with\n update_rate=" << update_rate_
			<< "\n child_frame_name=" << child_frame_name_
			<< "\n wall_length_left=" << wall_length_left_
			<< "\n wall_length_right=" << wall_length_right_
			<< "\n max_wall_side_distance_=" << max_wall_side_distance_ << "\n";
}
