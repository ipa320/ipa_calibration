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
* Repository name: ipa_calibration
* \note
* ROS package name: relative_localization
 *
 * \author
 * Author: Marc Riedlinger
 * \author
 * Supervised by:
 *
 * \date Date of creation: 25.08.2016
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


#include <relative_localization/reference_localization.h>
#include <relative_localization/relative_localization_utilities.h>


ReferenceLocalization::ReferenceLocalization(ros::NodeHandle& nh)
		: node_handle_(nh), transform_listener_(nh), initialized_(false), publish_polygon_frame_(false), reference_frame_ready_(false), publish_time_(ros::Time(0.f))
{
	// load parameters
	std::cout << "\n========== Reference Localization Parameters ==========\n";
	node_handle_.param("update_rate", update_rate_, 0.75);
	std::cout << "update_rate: " << update_rate_ << std::endl;
	node_handle_.param<std::string>("reference_frame", reference_frame_, "");
	std::cout << "reference_frame: " << reference_frame_ << std::endl;
	node_handle_.param<std::string>("laser_scanner_topic_in", laser_scanner_topic_in_, "");
	std::cout << "laser_scanner_topic_in: " << laser_scanner_topic_in_ << std::endl;
	node_handle_.param("base_height", base_height_, 0.0);
	std::cout << "base_height: " << base_height_ << std::endl;

	node_handle_.param<std::string>("base_frame", base_frame_, "");
	std::cout << "base_frame: " << base_frame_ << std::endl;
	node_handle_.param<std::string>("polygon_frame", polygon_frame_, "");
	std::cout << "polygon_frame: " << polygon_frame_ << std::endl;

	// check if detection_base_frame_ exists. If that is not the case, spawn it after a successful reference detection
	bool success = transform_listener_.waitForTransform(base_frame_, polygon_frame_, ros::Time(0), ros::Duration(2.0));
	if ( !success )
	{
		std::cout << polygon_frame_ << " frame does not exist, it will be created..." << std::endl;
		polygon_frame_default_ = polygon_frame_;
		polygon_frame_ = base_frame_;  // use base_frame until reference_frame has been found once
		publish_polygon_frame_ = true;
	}

	// read out user-defined polygon that defines the area of laser scanner points being taken into account for front wall detection
	std::vector<double> temp;
	node_handle_.getParam("front_wall_polygon", temp);
	const int num_points = temp.size()/2;
	if (temp.size()%2 != 0 || temp.size() < 4*2)
	{
		ROS_ERROR("ReferenceLocalization::ReferenceLocalization - The front_wall_polygon vector should contain at least 4 points with 2 values (x,y) each.");
		return;
	}
	std::cout << "Front wall polygon points:\n";
	for (int i=0; i<num_points; ++i)
	{
		front_wall_polygon_.push_back(cv::Point2f(temp[2*i], temp[2*i+1]));
		std::cout << temp[2*i] << "\t" << temp[2*i+1] << std::endl;
	}

	// publishers
	marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("wall_marker", 1);

	// subscribers
	laser_scan_sub_ = node_handle_.subscribe(laser_scanner_topic_in_, 0, &ReferenceLocalization::callback, this);

	// dynamic reconfigure
	dynamic_reconfigure_server_.setCallback(boost::bind(&ReferenceLocalization::dynamicReconfigureCallback, this, _1, _2));
	avg_translation_.setZero();

	ROS_INFO("ReferenceLocalization::ReferenceLocalization - Initialized.");
}

ReferenceLocalization::~ReferenceLocalization()
{
}

void ReferenceLocalization::dynamicReconfigureCallback(relative_localization::RelativeLocalizationConfig &config, uint32_t level)
{
	update_rate_ = config.update_rate;
	reference_frame_ = config.reference_frame;
	std::cout << "Reconfigure request with\n update_rate=" << update_rate_
			<< "\n reference_frame=" << reference_frame_ << "\n";
}

bool ReferenceLocalization::estimateFrontWall(std::vector<cv::Point2d>& scan_front, cv::Vec4d& line_front, const double inlier_ratio, const double success_probability,
		const double inlier_distance, const int repetitions)
{
	// search for front wall until a suitable estimate is found, i.e. when scalar product of line normal and robot's x-axis do not differ by more than 45deg angle
	bool found_front_line = false;
	for (int i=0; i<repetitions; ++i)
	{
		if (scan_front.size() < 2)
		{
			ROS_WARN("ReferenceLocalization::estimateFrontWall - No points left for estimating front wall.");
			return false;
		}

		// match line to scan_front
		bool result = RelativeLocalizationUtilities::fitLine(scan_front, line_front, 0.1, 0.99999, inlier_distance, false);
		if (!result || line_front.val[0] != line_front.val[0] || line_front.val[1] != line_front.val[1] || line_front.val[2] != line_front.val[2] || line_front.val[3] != line_front.val[3]) // check for NaN
		{
			ROS_WARN("ReferenceLocalization::estimateFrontWall - Front wall could not be estimated in trial %i. Trying next.", i);
			continue;
		}

		// check if line is good enough, i.e. if the angle between the front wall line normal and the robot base' x-axis (i.e. the axis pointing towards the front wall) is below 45deg
		const double scalar_product = line_front.val[2];	// = 1*line_front.val[2]+0*line_front.val[3]

		if (fabs(scalar_product) > 0.707)
		{
			found_front_line = true;
			break;
		}

		// remove points from last line
		for (std::vector<cv::Point2d>::iterator it = scan_front.begin(); it!=scan_front.end(); )
		{
			const double dist = RelativeLocalizationUtilities::distanceToLine(line_front.val[0], line_front.val[1], line_front.val[2], line_front.val[3], it->x, it->y);
			if (dist <= inlier_distance)
				it = scan_front.erase(it);
			else
				++it;
		}
	}
	if (found_front_line == false)
		ROS_WARN("ReferenceLocalization::estimateFrontWall - Front wall could not be estimated.");

	return found_front_line;
}

void ReferenceLocalization::computeAndPublishChildFrame(const cv::Vec4d& line, const cv::Point2d& corner_point, const ros::Time& time_stamp)
{
	// block coordinate system is attached at the left corner of the block directly on the wall surface
	const double px = line.val[0];	// coordinates of a point on the wall
	const double py = line.val[1];
	const double n0x = line.val[2];	// normal direction on the wall (in floor plane x-y)
	const double n0y = line.val[3];
	tf::StampedTransform transform_table_reference;
	// offset to base frame
	// intersection of line from left block point in normal direction with wall line
	// calculate projection (j*n0') of corner to wall: n0' = n0 rotated by 90°, to get the normal vector in direction of the wall
	// j = projection length
	// retrieve translation p' from base frame to corner frame
	// n0' = [n0y; -n0x]
	// p' = p + j*n0'
	double j = ((corner_point.x-px)*n0y + (py-corner_point.y)*n0x) / (n0x*n0x + n0y*n0y); // j = (corner - p) dot n0'
	double x = px + j*n0y;
	double y = py - j*n0x;
	tf::Vector3 translation(x, y, 0.);
	// direction of x-axis in base frame
	cv::Point2d normal(n0x, n0y);
	if (normal.x*translation.getX() + normal.y*translation.getY() < 0)
		normal *= -1.;
	double angle = atan2(normal.y, normal.x);
	tf::Quaternion orientation(tf::Vector3(0,0,1), angle); // rotation around z by value of angle

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
	tf::StampedTransform tf_msg(transform_table_reference, time_stamp, base_frame_, reference_frame_);
	shiftReferenceFrameToGround(tf_msg);

	transform_broadcaster_.sendTransform(tf_msg);
}

// only works for laser scanners mounted parallel to the ground, assuming that laser scanner frame and base_link have the same z-axis
void ReferenceLocalization::shiftReferenceFrameToGround(tf::StampedTransform& reference_frame)
{
	tf::Vector3 trans = reference_frame.getOrigin();
	reference_frame.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()-base_height_));
}

void ReferenceLocalization::setupPolygonFrame(const ros::Time& time_stamp)
{
	if ( !reference_frame_ready_ )  // check if reference_frame has been set up
	{
		bool success = assignPolygonFrame();
		if ( success )
		{
			reference_frame_ready_ = true;
			polygon_frame_ = polygon_frame_default_;
			publishPolygonFrame(time_stamp);
		}
	}
	else
	{
		publishPolygonFrame(time_stamp);
	}
}

bool ReferenceLocalization::assignPolygonFrame()
{
	try
	{
		transform_listener_.waitForTransform(reference_frame_, base_frame_, ros::Time(0), ros::Duration(1.0));
		transform_listener_.lookupTransform(reference_frame_, base_frame_, ros::Time(0), ref_to_base_initial_);
		return true;
	}
	catch (tf::TransformException& ex)
	{
		ROS_WARN("%s",ex.what());
		return false;
	}
}

void ReferenceLocalization::publishPolygonFrame(const ros::Time& time_stamp)
{
	tf::StampedTransform tf_msg(ref_to_base_initial_, time_stamp, reference_frame_, polygon_frame_);
	transform_broadcaster_.sendTransform(tf_msg);
	publish_time_ = time_stamp;
}

bool ReferenceLocalization::applyPolygonFilters(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg, const std::vector<cv::Point2f> &polygon_1, const std::vector<cv::Point2f> &polygon_2, std::vector<cv::Point2d> &scan_1, std::vector<cv::Point2d> &scan_2)
{
	cv::Mat T_base_to_laser;
	bool received_transform = RelativeLocalizationUtilities::getTransform(transform_listener_, base_frame_, std::string(laser_scan_msg->header.frame_id), T_base_to_laser);
	if (received_transform==false)
	{
		ROS_WARN("CornerLocalization::applyPolygonFilters - Could not determine transform between laser scanner and %s.", base_frame_.c_str());
		return false;
	}

	cv::Mat T_polygon_to_base;
	received_transform = RelativeLocalizationUtilities::getTransform(transform_listener_, polygon_frame_, base_frame_, T_polygon_to_base);
	if (received_transform==false)
	{
		ROS_WARN("CornerLocalization::applyPolygonFilters - Could not determine transform between %s and %s.", polygon_frame_.c_str(), base_frame_.c_str());
		return false;
	}
	cv::Mat T_polygon_to_laser = T_polygon_to_base*T_base_to_laser;

	/*received_transform = RelativeLocalizationUtilities::getTransformAdv(transform_listener_, polygon_frame_, std::string(laser_scan_msg->header.frame_id), base_frame_, T_polygon_to_laser, publish_time_, ros::Time(0.f));
	if (received_transform==false)
	{
		ROS_WARN("CornerLocalization::applyPolygonFilters - Could not determine transform between laser scanner and %s.", polygon_frame_.c_str());
		return false;
	}*/

	// retrieve points from side and front wall and put each of those in separate lists
	// scan_X vectors are relative to the base_frame, but the detection part (valid laserscanner points) is relative to the detection_base_frame
	for ( size_t i=0; i<laser_scan_msg->ranges.size(); i++ )
	{
		double angle = laser_scan_msg->angle_min + i * laser_scan_msg->angle_increment; // [rad]
		double dist = laser_scan_msg->ranges[i]; // [m]

		// transform laser scanner points to polygon_frame and base_frame
		cv::Mat point_laser(cv::Vec4d(dist*cos(angle), dist*sin(angle), 0, 1.0));
		cv::Mat point_polygon = T_polygon_to_laser*point_laser;  // laserscanner points in detection_base_frame coordinates
		cv::Point2f point_2d_polygon(point_polygon.at<double>(0), point_polygon.at<double>(1));
		cv::Mat point_base = T_base_to_laser*point_laser;  // laserscanner point in base_frame coords
		cv::Point2f point_2d_base(point_base.at<double>(0), point_base.at<double>(1));

		// check if point is inside front wall polygon and push to scan_front if that's the case
		// the reference frame will always be built relative to the base_frame, only the detection part is outsourced to the detection_base_frame
		if (cv::pointPolygonTest(polygon_1, point_2d_polygon, false) >= 0.f) // check if point in polygon_frame coords is within front wall polygon
			scan_1.push_back(point_2d_base);

		// store all points from the side polygon in here, use distance measure to front wall later to exclude front wall points
		if (cv::pointPolygonTest(polygon_2, point_2d_polygon, false) >= 0.f) // check if point in polygon_frame coords is within side wall polygon
			scan_2.push_back(point_2d_base);
	}

	return true;
}
