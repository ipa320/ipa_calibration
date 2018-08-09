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
 * Date of creation: July 2018
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


#include <robotino_calibration/file_utilities.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>


namespace file_utilities
{
	// create data storage path if it does not yet exist
	void createStorageFolder(std::string path)
	{
		boost::filesystem::path storage_path(path);

		if (boost::filesystem::exists(storage_path) == false)
		{
			if (boost::filesystem::create_directories(storage_path) == false && boost::filesystem::exists(storage_path) == false)
			{
				ROS_ERROR("RobotCalibration: Could not create directory %s ", storage_path.c_str());
				return;
			}
		}
	}

	void saveCalibrationResult(std::string file_path, std::string content)
	{
		std::fstream file_output;
		file_output.open(file_path.c_str(), std::ios::out | std::ios::app);  // append new results at end of file
		if (file_output.is_open())
			file_output << content;
		else
			ROS_WARN("Failed to open %s, not saving calibration results!", file_path.c_str());
		file_output.close();
	}

	void saveSnapshots(const std::vector< std::vector<TFSnapshot> > &snapshots, std::string save_path)
	{
		std::stringstream stream;

		stream << snapshots.size() << std::endl;
		for ( int i=0; i<snapshots.size(); ++i )
		{
			stream << snapshots[i].size() << std::endl;
			for ( TFSnapshot snap : snapshots[i] )
			{
				formatBETMs(stream, snap.branch_ends_to_markers_);
				formatTFInfos(stream, snap.parent_branch_);
				formatTFInfos(stream, snap.child_branch_);
				stream << snap.valid_ << std::endl;
			}
		}

		std::fstream file_output;
		file_output.open(save_path.c_str(), std::ios::out | std::ios::trunc);  // append new results at end of file
		if (file_output.is_open())
			file_output << stream.str();
		else
			ROS_WARN("Failed to open %s, not saving snapshot data!", save_path.c_str());
		file_output.close();
	}

	void formatBETMs(std::stringstream &stream, const std::vector<TFBranchEndsToMarkers> &BETMs)
	{
		stream << BETMs.size() << std::endl;
		for ( int i=0; i<BETMs.size(); ++i )
		{
			formatTFInfos(stream, BETMs[i].branch_to_child_markers_);
			formatTFInfos(stream, BETMs[i].otherbranch_to_parent_markers_);
			stream << BETMs[i].corresponding_uncertainty_idx_ << std::endl;
		}
	}

	void formatTFInfos(std::stringstream &stream, const std::vector<TFInfo> &TFInfos)
	{
		stream << TFInfos.size() << std::endl;
		for ( TFInfo info : TFInfos )
		{
			stream << info.parent_ << ";" << info.child_ << ";" << info.transform_ << ";";
		}
		stream << std::endl;
	}

	void loadSnapshots(std::vector< std::vector<TFSnapshot> > &snapshots, std::string load_path)
	{
	}
}



