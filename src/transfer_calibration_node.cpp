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
 * Date of creation: September 2017
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

#include <iostream>
#include <sstream>
#include <fstream>

//#include <boost/filesystem.hpp>

bool readFileContent(const std::string path, const std::string file, std::string &content)
{
	std::fstream file_output;
	std::string path_file = path + "/" + file;

	content.clear();
	file_output.open(path_file.c_str(), std::ios::in );
	if ( file_output.is_open() )
	{
		content = "";

		while ( !file_output.eof() )
		{
			std::string line;
			std::getline(file_output,line);
			content += line;
			if ( !file_output.eof() )
				content += "\n";
		}

		file_output.close(); // Data has been read, now close
		return true;
	}
	else
	{
		std::cout << "Warning, couldn't " << path << "/" << file << " for reading!" << std::endl << std::endl;
		return false;
	}
}

bool writeFileContent(const std::string path, const std::string file, const std::string content)
{
	std::fstream file_output;
	std::string path_file = path + "/" + file;

	file_output.open(path_file.c_str(), std::ios::out | std::ios::trunc);
	if ( !file_output.is_open() )
	{
		std::cout << "Error, couldn't open " << path_file << "!" << std::endl;
		std::cout << "Printing all data to screen to prevent data loss:" << std::endl;
		std::cout << content << std::endl;

		return false;
	}

	file_output << content;
	file_output.close();
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "transfer_calibration");

	ros::NodeHandle n("~");

	std::cout << "---------------- Transfer Calibration ----------------" << std::endl;
	std::string calibration_path = "";
	n.param<std::string>("calibration_path", calibration_path, "");
	std::cout << "calibration_path: " << calibration_path << std::endl;

	std::string calibration_file = "";
	n.param<std::string>("calibration_file", calibration_file, "");
	std::cout << "calibration_file: " << calibration_file << std::endl;

	std::string target_path = "";
	n.param<std::string>("target_path", target_path, "");
	std::cout << "target_path: " << target_path << std::endl;

	std::string target_file = "";
	n.param<std::string>("target_file", target_file, "");
	std::cout << "target_file: " << target_file << std::endl;

	std::vector<std::string> parameters;
	n.getParam("parameters", parameters);
	for (int i=0; i<parameters.size(); ++i)
		std::cout << "Parameter " << (i+1) << ": " << parameters[i] << std::endl;

	std::cout << "------------------------------------------------------" << std::endl;

	std::string calib_content = "";
	std::string target_content = "";

	// Read in calibration results and target file
	readFileContent(calibration_path, calibration_file, calib_content);
	readFileContent(target_path, target_file, target_content);

	// Retrieve values from calibration result and store them
	std::vector<double> calib_data(parameters.size());
	for ( int i=0; i<parameters.size(); ++i )
	{
		if ( parameters[i].length() == 0 )
			continue;

		size_t idx_c = calib_content.find(parameters[i]);
		size_t idx_t = target_content.find(parameters[i]);

		if ( idx_c != std::string::npos && idx_t != std::string::npos ) // Valid parameter
		{
			// Retrieve calibration result
			size_t idx_left = calib_content.find("value", idx_c) + 7;
			size_t idx_right = calib_content.find("\"", idx_left);
			std::string value = calib_content.substr(idx_left, idx_right-idx_left);

			// Replace old value with result
			idx_left = target_content.find("value", idx_t) + 7;
			idx_right = target_content.find("\"", idx_left);
			target_content.replace(idx_left, idx_right-idx_left, value);
		}
		else
		{
			std::cout << "WARNING: Couldn't find parameter " << parameters[i] << " in either calibration file or target file." << std::endl;
		}
	}

	writeFileContent(target_path, target_file, target_content);

	return 0;
}



