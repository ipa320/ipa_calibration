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

#ifndef REFERENCE_LOCALIZATION_H_
#define REFERENCE_LOCALIZATION_H_

#include <iostream>
#include <vector>

// ROS
#include <ros/ros.h>

// messages
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <relative_localization/RelativeLocalizationConfig.h>

// OpenCV
#include <opencv2/opencv.hpp>


class ReferenceLocalization
{
public:

	ReferenceLocalization(ros::NodeHandle& nh);
	virtual ~ReferenceLocalization();

protected:

	virtual void callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg) = 0;
	virtual void dynamicReconfigureCallback(relative_localization::RelativeLocalizationConfig& config, uint32_t level);

	// search for front wall until a suitable estimate is found, i.e. when scalar product of line normal and robot's x-axis do not differ by more than 45deg angle
	// scan_front the laser scan which contains all relevant points of the front wall (and possibly more points), this vector will be modified within the function
	bool estimateFrontWall(std::vector<cv::Point2d>& scan_front, cv::Vec4d& line_front, const double inlier_ratio=0.1, const double success_probability=0.99999,
			const double inlier_distance=0.01, const int repetitions=10);

	void computeAndPublishChildFrame(const cv::Vec4d& line, const cv::Point2d& corner_point, const std_msgs::Header::_stamp_type& time_stamp);

	// only works for laser scanners mounted parallel to the ground, assuming that laser scanner frame and base_link have the same z-axis
	void shiftReferenceFrameToGround(tf::StampedTransform& reference_frame);

	void setupPolygonFrame(const ros::Time& time_stamp);

	bool assignPolygonFrame();

	void publishPolygonFrame(const ros::Time& time_stamp);

	bool applyPolygonFilters(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg, const std::vector<cv::Point2f> &polygon_1, const std::vector<cv::Point2f> &polygon_2, std::vector<cv::Point2d> &scan_1, std::vector<cv::Point2d> &scan_2);


	bool initialized_;

	ros::NodeHandle node_handle_;
	ros::Subscriber laser_scan_sub_;
	ros::Publisher marker_pub_;

	tf::TransformBroadcaster transform_broadcaster_;
	tf::TransformListener transform_listener_;

	dynamic_reconfigure::Server<relative_localization::RelativeLocalizationConfig> dynamic_reconfigure_server_;
	tf::Vector3 avg_translation_;
	tf::Quaternion avg_orientation_;
	double base_height_;

	// parameters
	double update_rate_;
	std::string laser_scanner_topic_in_;
	std::string reference_frame_;
	std::vector<cv::Point2f> front_wall_polygon_;

	// frame which is used to filter out wall points, it can be assigned with an existing frame or a yet unknown frame
	// Existing frame: Polygons for detection wall points will be build upon the assigned existing frame (e.g. robot's base)
	// Unknown frame: A new frame will be generated at startup upon the first reference_frame detection (from reference_frame to base_frame)
	// The frame will be set up once at startup and stays fiixed from that time on which prevents that robot rotations mess up the reference frame detection.
	std::string polygon_frame_;
	std::string polygon_frame_default_;  // saving original value of detection_base_frame_
	std::string base_frame_;  // needed to check whether detection_base_frame_ exists
	tf::StampedTransform ref_to_base_initial_;  // transform between reference_frame and base_frame_ at startup
	bool publish_polygon_frame_;
	bool reference_frame_ready_;  // whether the detection_frame can be build
	ros::Time publish_time_;  // ros time of currently published transforms
};


#endif // REFERENCE_LOCALIZATION_H_
