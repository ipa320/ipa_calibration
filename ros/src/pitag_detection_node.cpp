/****************************************************************
 *
 * Copyright (c) 2015
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: squirrel
 * ROS stack name: squirrel_calibration
 * ROS package name: robotino_calibration
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Marc Riedlinger, email:marc.riedlinger@ipa.fraunhofer.de
 *
 * Date of creation: June 2018
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/


#include <ros/ros.h>
#include <cob_object_detection_msgs/DetectObjects.h>


// detect and publish pitag marker
int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "pitag_detection");

	// Create a handle for this node, initialize node
	ros::NodeHandle node_handle("~");

	// Load necessary parameters
	std::cout << "\n========== Pitag Detection Parameters ==========\n";
	std::string get_fiducials_topic;
	node_handle.param<std::string>("get_fiducials_topic", get_fiducials_topic, "");
	std::cout << "get_fiducials_topic: " << get_fiducials_topic << std::endl;

	double update_rate;
	node_handle.param("update_rate", update_rate, 0.5);
	std::cout << "update_rate: " << update_rate << std::endl;

	ros::ServiceClient pitag_client = node_handle.serviceClient<cob_object_detection_msgs::DetectObjects>(get_fiducials_topic);

	// Cyclically detect pitags and publish resulting frames to tf
	while ( ros::ok() )
	{
		ros::spinOnce();

		// detect tags, but as results will be published to tf, we don't need to process any results here anymore
		cob_object_detection_msgs::DetectObjects detect;
		pitag_client.call(detect);

		ros::Duration(update_rate).sleep();
	}
}
