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
//#include <relative_localization/visualization_utilities.h>
#include <relative_localization/relative_localization_utilities.h>


ReferenceLocalization::ReferenceLocalization(ros::NodeHandle& nh)
		: node_handle_(nh), transform_listener_(nh)//, ref_frame_init_(false)//, laser_scanner_mounting_height_(0.), laser_scanner_mounting_height_received_(false)
{
	// load parameters
	std::cout << "\n========== Reference Localization Parameters ==========\n";
	node_handle_.param("update_rate", update_rate_, 0.75);
	std::cout << "update_rate: " << update_rate_ << std::endl;
	node_handle_.param<std::string>("child_frame_name", child_frame_name_, "");
	std::cout << "child_frame_name: " << child_frame_name_ << std::endl;
	node_handle_.param("reference_coordinate_system_at_ground", reference_coordinate_system_at_ground_, false);
	std::cout << "reference_coordinate_system_at_ground: " << reference_coordinate_system_at_ground_ << std::endl;
	//node_handle_.param("wall_length_left", wall_length_left_, 0.75);
	//std::cout << "wall_length_left: " << wall_length_left_ << std::endl;
	//node_handle_.param("wall_length_right", wall_length_right_, 0.75);
	//std::cout << "wall_length_right: " << wall_length_right_ << std::endl;
	node_handle_.param<std::string>("laser_scanner_command", laser_scanner_command_, "/laser_scanner_in");
	std::cout << "laser_scanner_command: " << laser_scanner_command_ << std::endl;
	node_handle_.param<std::string>("base_frame", base_frame_, "base_link");
	std::cout << "base_frame: " << base_frame_ << std::endl;
	node_handle_.param("base_height", base_height_, 0.0);
	std::cout << "base_height: " << base_height_ << std::endl;

	// read out user-defined polygon that defines the area of laser scanner points being taken into account for wall detection
	std::vector<double> temp;
	node_handle_.getParam("front_wall_polygon", temp);
	const int num_points = temp.size()/2;
	if (temp.size()%2 != 0 || temp.size() < 3*2)
	{
		ROS_ERROR("The front_wall_polygon vector should contain at least 3 points with 2 values (x,y) each.");
		return;
	}
	std::cout << "Front Wall Polygon Points:\n";
	for (int i=0; i<num_points; ++i)
	{
		front_wall_polygon_.push_back(cv::Point2f(temp[2*i], temp[2*i+1]));
		std::cout << temp[2*i] << "\t" << temp[2*i+1] << std::endl;
	}

	// publishers
	marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("wall_marker", 1);

	// services
	get_frame_state_ = node_handle_.advertiseService("get_frame_state", &StateGetter::GetFrameStateSrv, &frame_state_srv_);

	// subscribers
	laser_scan_sub_ = node_handle_.subscribe(laser_scanner_command_, 0, &ReferenceLocalization::callback, this);

	// dynamic reconfigure
	dynamic_reconfigure_server_.setCallback(boost::bind(&ReferenceLocalization::dynamicReconfigureCallback, this, _1, _2));
	avg_translation_.setZero();

	ROS_INFO("ReferenceLocalization: Initialized.");
}

ReferenceLocalization::~ReferenceLocalization()
{
}

void ReferenceLocalization::dynamicReconfigureCallback(robotino_calibration::RelativeLocalizationConfig &config, uint32_t level)
{
	update_rate_ = config.update_rate;
	child_frame_name_ = config.child_frame_name;
	//wall_length_left_ = config.wall_length_left;
	//wall_length_right_ = config.wall_length_right;
	std::cout << "Reconfigure request with\n update_rate=" << update_rate_
			<< "\n child_frame_name=" << child_frame_name_ << "\n";
			//<< "\n wall_length_left=" << wall_length_left_
			//<< "\n wall_length_right=" << wall_length_right_ << "\n";
}

// only works for laser scanners mounted parallel to the ground, assuming that laser scanner frame and base_link have the same z-axis
void ReferenceLocalization::ShiftReferenceFrameToGround(tf::StampedTransform& reference_frame)
{
	cv::Mat T;
	/*if (laser_scanner_mounting_height_received_==false && RelativeLocalizationUtilities::getTransform(transform_listener_, base_frame_, reference_frame.child_frame_id_, T) == true)
	{
		laser_scanner_mounting_height_ = T.at<double>(2,3);
		laser_scanner_mounting_height_received_ = true;
	}*/

	tf::Vector3 trans = reference_frame.getOrigin();
	reference_frame.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()-base_height_));
}

void ReferenceLocalization::ComputeChildFrame(const cv::Vec4d& line, const cv::Point2d& corner_point, const std_msgs::Header::_stamp_type& time_stamp)
{
	// block coordinate system is attached at the left corner of the block directly on the wall surface
	bool publish_tf = true;
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
	tf::StampedTransform tf_msg(transform_table_reference, time_stamp, base_frame_, child_frame_name_);
	if (reference_coordinate_system_at_ground_ == true)
		ShiftReferenceFrameToGround(tf_msg);

	// publish coordinate system on tf
	if (publish_tf == true)
	{
		transform_broadcaster_.sendTransform(tf_msg);
	}

	frame_state_srv_.frame_state_ = true; // ComputeChildFrame will be called when the reference frame has sucessfully been detected. A good place to also set this flag.
}
