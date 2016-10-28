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

#ifndef REFERENCE_LOCALIZATION_H_
#define REFERENCE_LOCALIZATION_H_

#include <iostream>
#include <vector>

// ROS
#include "ros/ros.h"

// messages
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <relative_localization/RelativeLocalizationConfig.h>

// OpenCV
#include <opencv/cv.h>

class ReferenceLocalization
{
public:

	ReferenceLocalization(ros::NodeHandle& nh);
	virtual ~ReferenceLocalization();

	// only works for laser scanners mounted parallel to the ground, assuming that laser scanner frame and base_link have the same z-axis
	void ShiftReferenceFrameToGround(tf::StampedTransform& reference_frame);


protected:

	virtual void callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg) = 0;
	virtual void dynamicReconfigureCallback(robotino_calibration::RelativeLocalizationConfig& config, uint32_t level);

	ros::NodeHandle node_handle_;
	ros::Subscriber laser_scan_sub_;
	ros::Publisher marker_pub_;

	tf::TransformBroadcaster transform_broadcaster_;
	tf::TransformListener transform_listener_;

	dynamic_reconfigure::Server<robotino_calibration::RelativeLocalizationConfig> dynamic_reconfigure_server_;
	tf::Vector3 avg_translation_;
	tf::Quaternion avg_orientation_;
	double laser_scanner_mounting_height_;
	bool laser_scanner_mounting_height_received_;

	// parameters
	double update_rate_;
	std::string base_frame_;
	std::string laser_scanner_command_;
	std::string child_frame_name_;
	bool reference_coordinate_system_at_ground_;	// if the laser scanner is mounted parallel to the ground plane and this flag is activated, the reference coordinate system child_frame_name will be established at ground height (instead of laser scanner mounting height)
	//double wall_length_left_;		// the length of the wall segment left of the checkerboard's origin, in[m]
	//double wall_length_right_;		// the length of the wall segment right of the checkerboard's origin, in[m]
	std::vector<cv::Point2f> front_wall_polygon_;
};


#endif // REFERENCE_LOCALIZATION_H_
