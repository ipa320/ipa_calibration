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
#include <relative_localization/visualization_utilities.h>
#include <relative_localization/relative_localization_utilities.h>


ReferenceLocalization::ReferenceLocalization(ros::NodeHandle& nh)
		: node_handle_(nh), transform_listener_(nh), laser_scanner_mounting_height_(0.)
{
	// load parameters
	std::cout << "\n========== Box Localization Parameters ==========\n";
	node_handle_.param("update_rate", update_rate_, 0.75);
	std::cout << "update_rate: " << update_rate_ << std::endl;
	node_handle_.param<std::string>("child_frame_name", child_frame_name_, "");
	std::cout << "child_frame_name: " << child_frame_name_ << std::endl;
	node_handle_.param("reference_coordinate_system_at_ground", reference_coordinate_system_at_ground_, false);
	std::cout << "reference_coordinate_system_at_ground: " << reference_coordinate_system_at_ground_ << std::endl;
	node_handle_.param("wall_length_left", wall_length_left_, 0.75);
	std::cout << "wall_length_left: " << wall_length_left_ << std::endl;
	node_handle_.param("wall_length_right", wall_length_right_, 0.75);
	std::cout << "wall_length_right: " << wall_length_right_ << std::endl;

	// publishers
	marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("wall_marker", 1);

	// subscribers
	laser_scan_sub_ = node_handle_.subscribe("laser_scan_in", 0, &ReferenceLocalization::callback, this);

	// dynamic reconfigure
	dynamic_reconfigure_server_.setCallback(boost::bind(&ReferenceLocalization::dynamicReconfigureCallback, this, _1, _2));
	avg_translation_.setZero();
}

ReferenceLocalization::~ReferenceLocalization()
{
}

void ReferenceLocalization::dynamicReconfigureCallback(robotino_calibration::RelativeLocalizationConfig &config, uint32_t level)
{
	update_rate_ = config.update_rate;
	child_frame_name_ = config.child_frame_name;
	wall_length_left_ = config.wall_length_left;
	wall_length_right_ = config.wall_length_right;
	std::cout << "Reconfigure request with\n update_rate=" << update_rate_
			<< "\n child_frame_name=" << child_frame_name_
			<< "\n wall_length_left=" << wall_length_left_
			<< "\n wall_length_right=" << wall_length_right_ << "\n";
}

// computes the transform from target_frame to source_frame (i.e. transform arrow is pointing from target_frame to source_frame)
bool ReferenceLocalization::getTransform(const std::string& target_frame, const std::string& source_frame, cv::Mat& T)
{
	try
	{
		tf::StampedTransform Ts;
		transform_listener_.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
		transform_listener_.lookupTransform(target_frame, source_frame, ros::Time(0), Ts);
		const tf::Matrix3x3& rot = Ts.getBasis();
		const tf::Vector3& trans = Ts.getOrigin();
		cv::Mat rotcv(3,3,CV_64FC1);
		cv::Mat transcv(3,1,CV_64FC1);
		for (int v=0; v<3; ++v)
			for (int u=0; u<3; ++u)
				rotcv.at<double>(v,u) = rot[v].m_floats[u];
		for (int v=0; v<3; ++v)
			transcv.at<double>(v) = trans.m_floats[v];
		T = robotino_calibration::makeTransform(rotcv, transcv);
		//std::cout << "Transform from " << source_frame << " to " << target_frame << ":\n" << T << std::endl;
	}
	catch (tf::TransformException& ex)
	{
		ROS_WARN("%s",ex.what());
		return false;
	}

	return true;
}

cv::Mat ReferenceLocalization::makeTransform(const cv::Mat& R, const cv::Mat& t)
{
	cv::Mat T = (cv::Mat_<double>(4,4) <<
			R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
			R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
			R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2),
			0., 0., 0., 1);
	return T;
}
