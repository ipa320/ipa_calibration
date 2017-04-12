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

#include <relative_localization/box_localization.h>
#include <relative_localization/visualization_utilities.h>
#include <relative_localization/relative_localization_utilities.h>

BoxLocalization::BoxLocalization(ros::NodeHandle& nh)
		: ReferenceLocalization(nh)
{
	// read out user-defined box search polygon
	std::vector<double> temp;
	node_handle_.getParam("box_search_polygon", temp);
	const int num_points = temp.size()/2;
	if (temp.size()%2 != 0 || temp.size() < 3*2)
	{
		ROS_ERROR("The box_search_polygon vector should contain at least 3 points with 2 values (x,y) each.");
		return;
	}
	std::cout << "Box Search Polygon Points:\n";
	for (int i=0; i<num_points; ++i)
	{
		box_search_polygon_.push_back(cv::Point2f(temp[2*i], temp[2*i+1]));
		std::cout << temp[2*i] << "\t" << temp[2*i+1] << std::endl;
	}

	ROS_INFO("BoxLocalization: Initialized.");
	initialized_ = true;
}

BoxLocalization::~BoxLocalization()
{
}

//#define DEBUG_OUTPUT
void BoxLocalization::callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg)
{
	if (initialized_ == false)
		return;

	if (marker_pub_.getNumSubscribers() > 0)
	{
		VisualizationUtilities::publishDetectionPolygon(laser_scan_msg->header, "front_wall_polygon", front_wall_polygon_, 0, marker_pub_);
		VisualizationUtilities::publishDetectionPolygon(laser_scan_msg->header, "box_polygon", box_search_polygon_, 0, marker_pub_, 1.0, 0.5, 0.0);
	}

	// ---------- 1. data preparation ----------
	// retrieve transform from laser scanner to base
	cv::Mat T;
	bool received_transform = RelativeLocalizationUtilities::getTransform(transform_listener_, base_frame_, laser_scan_msg->header.frame_id, T);
	if (received_transform==false)
 	{
		ROS_WARN("BoxLocalization::callback: Could not determine transform T between laser scanner and base.");
		return;
	}

	// convert scan to x-y coordinates
	std::vector<cv::Point2d> scan_front;
	std::vector<cv::Point2d> scan_all;
	for (unsigned int i = 0; i < laser_scan_msg->ranges.size(); ++i)
	{
		double angle = laser_scan_msg->angle_min + i * laser_scan_msg->angle_increment; //[rad]
		double dist = laser_scan_msg->ranges[i];

		// transform laser scanner points to base frame
		cv::Mat point_laser(cv::Vec4d(dist*cos(angle), dist*sin(angle), 0, 1.0));
		cv::Mat point_base_mat = T*point_laser;
		cv::Point2f point_2d_base(point_base_mat.at<double>(0), point_base_mat.at<double>(1));

		// Check if point is inside polygon and push to scan if that's the case
		if (cv::pointPolygonTest(front_wall_polygon_, point_2d_base, false) >= 0.f) // front wall points
			scan_front.push_back(point_2d_base);

		scan_all.push_back(point_2d_base);
	}

	// ---------- 2. front wall estimation ----------
	// search for front wall until a suitable estimate is found, i.e. when scalar product of line normal and robot's x-axis do not differ by more than 45deg angle
	const double inlier_distance = 0.01;
	cv::Vec4d line_front;
	bool found_front_line = estimateFrontWall(scan_front, line_front, 0.1, 0.99999, inlier_distance, 10);
	if (found_front_line == false)
	{
		ROS_WARN("BoxLocalization::callback: front wall could not be estimated.");
		return;
	}
	// display line
	const double px = line_front.val[0];	// coordinates of a point on the wall
	const double py = line_front.val[1];
	const double n0x = line_front.val[2];	// normal direction on the wall (in floor plane x-y)
	const double n0y = line_front.val[3];

	if (marker_pub_.getNumSubscribers() > 0)
		VisualizationUtilities::publishWallVisualization(laser_scan_msg->header, "wall_front", px, py, n0x, n0y, marker_pub_);

	// ---------- 3. box localization ----------
	// find blocks in front of the wall
	std::vector< std::vector<cv::Point2d> > segments;
	std::vector<cv::Point2d> segment;
	bool in_reflector_segment = false;
	for (unsigned int i = 0; i < scan_all.size(); ++i)
	{
		//double distance_to_robot = scan[i].x*scan[i].x + scan[i].y*scan[i].y;
		if (cv::pointPolygonTest(box_search_polygon_, scan_all[i], false) >= 0 )	// only search for block inside search polygon
		{
			double d = fabs(n0x*(scan_all[i].x-px) + n0y*(scan_all[i].y-py));		// distance to wall
			if (d<0.1 && in_reflector_segment==true)
			{
				// finish segment
				segments.push_back(segment);
				segment.clear();
				in_reflector_segment = false;
			}
			if (d >= 0.1)
			{
				segment.push_back(scan_all[i]);
				in_reflector_segment = true;
			}
		}
	}
	segments.push_back(segment);

	// select largest group, which is far away from the wall, as the calibration box
	cv::Point2d corner_point;
	size_t largest_segment_size = 0;
	size_t largest_segment = 0;
	for (size_t i=0; i<segments.size(); ++i)
	{
		if (segments[i].size()>largest_segment_size)
		{
			largest_segment_size = segments[i].size();
			largest_segment = i;
			corner_point = segments[i][segments[i].size()-1];
		}
	}
	if (corner_point.x == 0 && corner_point.y == 0)
		return;

	// display points of box segment
	if (marker_pub_.getNumSubscribers() > 0)
		VisualizationUtilities::publishPointsVisualization(laser_scan_msg->header, "box_points", segments[largest_segment], marker_pub_);

#ifdef DEBUG_OUTPUT
	std::cout << "Corner point: " << corner_point << std::endl;
#endif

	// ---------- 4. publish tf ----------
	// determine coordinate system generated by block in front of a wall
	// block coordinate system is attached at the left corner of the block directly on the wall surface
	computeAndPublishChildFrame(line_front, corner_point, laser_scan_msg->header.stamp);
}

// reflector-based
//void CheckerboardLocalization::callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg)
//{
//	// detect reflector markers (by intensity thresholding)
//	std::vector<std::vector<Point2d> > segments;
//	std::vector<Point2d> segment;
//	bool in_reflector_segment = false;
//	for (unsigned int i = 0; i < laser_scan_msg->ranges.size(); ++i)
//	{
//		double angle = laser_scan_msg->angle_min + i * laser_scan_msg->angle_increment; //[rad]
//		double dist = laser_scan_msg->ranges[i];
//		double y = dist * sin(angle);
//		double x = dist * cos(angle);
//		if (laser_scan_msg->intensities[i] < 4048.0 && in_reflector_segment==true)
//		{
//			// finish reflector segment
//			segments.push_back(segment);
//			segment.clear();
//			in_reflector_segment = false;
//		}
//		if (laser_scan_msg->intensities[i] >= 4048.0)
//		{
//			segment.push_back(Point2d(x, y));
//			in_reflector_segment = true;
//		}
//	}
//
//#ifdef DEBUG_OUTPUT
//	std::cout << "Segments with high intensity:\n";
//	for (size_t i=0; i<segments.size(); ++i)
//	{
//		for (size_t j=0; j<segments[i].size(); ++j)
//			std::cout << segments[i][j] << "\t";
//		std::cout << std::endl;
//	}
//#endif
//
//	// compute center coordinates of reflectors
//	std::vector<Point2d> corners(segments.size());
//	for (size_t i=0; i<segments.size(); ++i)
//	{
//		for (size_t j=0; j<segments[i].size(); ++j)
//			corners[i] += segments[i][j];
//		corners[i] /= (double)segments[i].size();
//	}
//
//#ifdef DEBUG_OUTPUT
//	std::cout << "Corner centers:\n";
//	for (size_t i=0; i<corners.size(); ++i)
//	{
//		std::cout << corners[i] << std::endl;
//	}
//#endif
//
//	// determine coordinate system generated by reflectors (laser scanner coordinate system: x=forward, y=left, z=up)
//	bool publish_tf = false;
//	tf::StampedTransform transform_table_reference;
//	if (corners.size()==2)
//	{
//		publish_tf = true;
//		// offset in laser scanner coordinate system
//		tf::Vector3 translation((corners[0].x+corners[1].x)*0.5, (corners[0].y+corners[1].y)*0.5, 0.);
//		// direction of x-axis in laser scanner coordinate system
//		Point2d normal(corners[1].y-corners[0].y, corners[0].x-corners[1].x);
//		if (normal.x*translation.getX() + normal.y*translation.getY() < 0)
//			normal *= -1.;
//		double angle = atan2(normal.y, normal.x);
//		tf::Quaternion orientation(tf::Vector3(0,0,1), angle);
//
//		// update transform
//		if (avg_translation_.isZero())
//		{
//			// use value directly on first message
//			avg_translation_ = translation;
//			avg_orientation_ = orientation;
//		}
//		else
//		{
//			// update value
//			avg_translation_ = (1.0 - update_rate_) * avg_translation_ + update_rate_ * translation;
//			avg_orientation_.setW((1.0 - update_rate_) * avg_orientation_.getW() + update_rate_ * orientation.getW());
//			avg_orientation_.setX((1.0 - update_rate_) * avg_orientation_.getX() + update_rate_ * orientation.getX());
//			avg_orientation_.setY((1.0 - update_rate_) * avg_orientation_.getY() + update_rate_ * orientation.getY());
//			avg_orientation_.setZ((1.0 - update_rate_) * avg_orientation_.getZ() + update_rate_ * orientation.getZ());
//		}
//
//		// transform
//		transform_table_reference.setOrigin(avg_translation_);
//		transform_table_reference.setRotation(avg_orientation_);
//	}
//	else if (corners.size() > 2)
//	{
//		ROS_WARN("Regression with >2 reflectors not implemented");
//	}
//
//	// publish coordinate system on tf
//	if (publish_tf == true)
//	{
//		transform_broadcaster_.sendTransform(tf::StampedTransform(transform_table_reference.inverse(), laser_scan_msg->header.stamp, child_frame_name_, laser_scan_msg->header.frame_id));
//	}
//}

void BoxLocalization::dynamicReconfigureCallback(robotino_calibration::RelativeLocalizationConfig &config, uint32_t level)
{
	ReferenceLocalization::dynamicReconfigureCallback(config, level); // call to base class
	//box_search_width_ = config.box_search_width;
	//std::cout << "box_search_width=" << box_search_width_ << "\n";
}
