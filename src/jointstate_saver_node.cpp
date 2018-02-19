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
 * Date of creation: June 2017
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
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <limits>

#include <boost/filesystem.hpp>


// Global variables to store results of callbacks
std::vector<double> current_arm_state;
std::vector<double> current_cam_state;

// Callbacks
void armStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	if ( current_arm_state.size() != msg->position.size())
	{
		current_arm_state.clear();
		current_arm_state.resize(msg->position.size());
	}

	for ( size_t i=0; i<current_arm_state.size(); ++i )
		current_arm_state[i] = msg->position[i];
}

void cameraStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	if ( current_cam_state.size() != msg->position.size())
	{
		current_cam_state.clear();
		current_cam_state.resize(msg->position.size());
	}

	for ( size_t i=0; i<current_cam_state.size(); ++i )
		current_cam_state[i] = msg->position[i];
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jointstate_saver");

	ros::NodeHandle n("~");

	std::cout << "---------------- Joint Saver ----------------" << std::endl;
	std::string storage_path = "";
	n.param<std::string>("storage_path", storage_path, "jointstate_saver/output");
	std::cout << "storage_path: " << storage_path << std::endl;

	std::string file_name = "";
	n.param<std::string>("file_name", file_name, "RobotConfig.txt");
	std::cout << "file_name: " << file_name << std::endl;

	std::string jointstate_topic_arm = "";
	n.param<std::string>("jointstate_topic_arm", jointstate_topic_arm, "");
	std::cout << "jointstate_topic_arm: " << jointstate_topic_arm << std::endl;

	std::string jointstate_topic_camera = "";
	n.param<std::string>("jointstate_topic_camera", jointstate_topic_camera, "");
	std::cout << "jointstate_topic_camera: " << jointstate_topic_camera << std::endl;
	std::cout << "---------------------------------------------" << std::endl << std::endl;

	ros::Subscriber arm_state = n.subscribe<sensor_msgs::JointState>(jointstate_topic_arm, 1, armStateCallback);
	ros::Subscriber cam_state = n.subscribe<sensor_msgs::JointState>(jointstate_topic_camera, 1, cameraStateCallback);

	std::string path_file = storage_path + "/" + file_name;

	boost::filesystem::path boost_storage_path(storage_path);
	if (boost::filesystem::exists(boost_storage_path) == false)
	{
		if (boost::filesystem::create_directories(boost_storage_path) == false && boost::filesystem::exists(boost_storage_path) == false)
		{
			std::cout << "Error: Could not create storage directory " << storage_path << std::endl;
			return -1;
		}
	}

	while (ros::ok())
	{
		char input = '0';

		std::cout << "Press 'e' to exit, any other key will append the current states to the storage file." << std::endl;
		std::cin >> input;
		std::cin.clear();
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard further inputs to guarantee a halt at next cin
		std::cout << std::endl;

		if ( input == 'e' ) // Exit program
			break;

		ros::spinOnce(); // Get new data from callbacks

		if ( current_arm_state.size() > 0 && current_cam_state.size() > 0 ) // Append current states to text file
		{
			std::fstream file_output;
			std::string filecontent = "";

			// First, read current data from file
			file_output.open(path_file.c_str(), std::ios::in );
			if ( file_output.is_open() )
			{
				while ( !file_output.eof() )
				{
					std::string line;
					std::getline(file_output,line);
					filecontent += line;
					if ( !file_output.eof() ) // Restore newline characters which became lost by calling getline()
						filecontent += "\n";
				}

				file_output.close(); // Data has been read, now close
			}
			else
			{
				std::cout << "Warning, couldn't open storage file for reading! If it had not existed before, it will be created now." << std::endl << std::endl;
			}

			bool is_empty = filecontent.empty();

			if ( is_empty ) // Initialize file content
				filecontent = "robot_configurations: []";

			std::vector<double> robot_config;
			robot_config.insert(robot_config.begin(), current_arm_state.begin(), current_arm_state.end()); // First add arm configs
			robot_config.insert(robot_config.end(), current_cam_state.begin(), current_cam_state.end());   // Append camera configs to the end

			// Write current robot config to stringstream
			std::stringstream new_data("");
			if ( !is_empty )
				new_data << ",\n";

			for ( size_t i=0; i<robot_config.size(); ++i )
				new_data << robot_config[i] << (i == robot_config.size()-1 ? "]" : ", ");

			size_t idx = filecontent.find("]");

			if ( idx != std::string::npos ) // ] has been found
			{
				filecontent.replace(idx,1,new_data.str()); // Update content

				// Clear text file and write whole updated data to it afterwards
				file_output.open(path_file.c_str(), std::ios::out | std::ios::trunc);
				if ( !file_output.is_open() )
				{
					std::cout << "Error, cannot open storage file " << file_name << "!" << std::endl;
					if ( !is_empty )
					{
						std::cout << "Printing all data to screen to prevent data loss:" << std::endl;
						std::cout << filecontent << std::endl;
					}

					continue;
				}

				// Write to file
				file_output << filecontent;
				file_output.close();

				// Print to screen
				std::cout << "robot_config: [";
					for (size_t i=0; i<robot_config.size(); ++i)
						std::cout << robot_config[i] << (i==robot_config.size()-1 ? "]\n" : ", ");
				std::cout << "arm: [";
				for (size_t i=0; i<current_arm_state.size(); ++i)
					std::cout << current_arm_state[i] << (i==current_arm_state.size()-1 ? "]\n" : ", ");
				std::cout << "camera: [";
					for (size_t i=0; i<current_cam_state.size(); ++i)
						std::cout << current_cam_state[i] << (i==current_cam_state.size()-1 ? "]\n" : ", ");
				std::cout << std::endl;
			}
			else
			{
				std::cout << "Output file " << file_name << " is corrupted, please delete it!" << std::endl;
				return -1;
			}
		}
		else
		{
			std::cout << "Error, either camera or arm state vector is empty!" << std::endl;
		}
	}

	return 0;
}



