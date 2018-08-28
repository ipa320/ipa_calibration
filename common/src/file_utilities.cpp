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
#include <sstream>
#include <robotino_calibration/time_utilities.h>


namespace file_utilities
{
	// create data storage path if it does not yet exist
	void createStorageFolder(const std::string &path)
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

	void saveCalibrationResult(const std::string &file_path, const std::string &content)
	{
		std::fstream file_output;
		file_output.open(file_path.c_str(), std::ios::out | std::ios::app);  // append new results at end of file
		if (file_output.is_open())
			file_output << content;
		else
			ROS_WARN("Failed to open %s, can't save calibration results!", file_path.c_str());
		file_output.close();
	}

	void saveSnapshots(const std::vector< std::vector<TFSnapshot> > &snapshots, const std::string &save_path, const std::string &file_name)
	{
		std::stringstream stream("");

		for ( int i=0; i<snapshots.size(); ++i )  // go through robot setups (each robot config has a snapshot for each calibration setup)
		{
			stream << "a{" << std::endl;
			for ( TFSnapshot snap : snapshots[i] )  // go through calibration setups
			{
				stream << "b{" << std::endl;
				formatBETMs(stream, snap.branch_ends_to_markers_);
				formatTFInfos(stream, snap.parent_branch_);
				formatTFInfos(stream, snap.child_branch_);
			}
			stream << "a}" << std::endl;
		}

		std::fstream file_output;
		std::string file_path = save_path+"/"+file_name;
		file_output.open(file_path.c_str(), std::ios::out | std::ios::app);  // append to current file
		if (file_output.is_open())
			file_output << stream.str();
		else
			ROS_WARN("Failed to open %s, can't save snapshot data!", file_path.c_str());
		file_output.close();
	}

	void formatBETMs(std::stringstream &stream, const std::vector<TFBranchEndsToMarkers> &BETMs)
	{
		for ( int i=0; i<BETMs.size(); ++i )
		{
			stream << "c{" << std::endl;
			formatTFInfos(stream, BETMs[i].branch_to_child_markers_);
			formatTFInfos(stream, BETMs[i].otherbranch_to_parent_markers_);
			stream << BETMs[i].corresponding_uncertainty_idx_ << std::endl;
		}

		stream << "c}" << std::endl;
	}

	void formatTFInfos(std::stringstream &stream, const std::vector<TFInfo> &TFInfos)
	{
		for ( TFInfo info : TFInfos )
		{
			std::string trafo = trafoToString(info.transform_);

			if ( trafo.compare("") == 0 )
			{
				ROS_WARN("Bad transformation, skipping to save entry.");
				return;
			}

			stream << info.parent_ << std::endl << info.child_ << std::endl << trafo << std::endl;
		}
		stream << "{}" << std::endl;
	}

	std::string trafoToString(const cv::Mat &trafo)
	{
		std::stringstream result("");

		if ( trafo.dims != 2 || trafo.cols*trafo.rows != 16 )
			return "";

		for ( int i=0; i<trafo.rows; ++i )
		{
			for ( int j=0; j<trafo.cols; ++j )
			{
				result << trafo.at<double>(i,j);

				if ( i < trafo.cols-1 || j < trafo.rows-1 )  // no "," after last element
					result << ",";
			}
		}

		return result.str();
	}

	bool loadSnapshots(std::vector< std::vector<TFSnapshot> > &snapshots, const std::string &load_path, const std::string &file_name)
	{
		bool result = true;
		std::fstream file_input;
		std::string file_path = load_path+"/"+file_name;
		file_input.open(file_path.c_str(), std::ios::in);  // read snapshot content
		if ( file_input.is_open() )
		{
			while ( !file_input.eof() )
			{
				std::string line = "";
				std::getline(file_input, line);

				if ( line.compare("a{") == 0 )
				{
					std::vector<TFSnapshot> snaps;

					while ( !file_input.eof() )
					{
						line = "";
						std::getline(file_input, line);

						if ( line.compare("a}") == 0 )
							break;
						else if ( line.compare("b{") == 0 )
						{
							TFSnapshot snap;

							while ( !file_input.eof() )
							{
								line = "";
								std::getline(file_input, line);

								if ( line.compare("c{") == 0 )
								{
									TFBranchEndsToMarkers BEM;

									buildTFInfos(file_input, BEM.branch_to_child_markers_);
									buildTFInfos(file_input, BEM.otherbranch_to_parent_markers_);
									std::getline(file_input, line);
									BEM.corresponding_uncertainty_idx_ = std::stoi(line);

									snap.branch_ends_to_markers_.push_back(BEM);
								}
								else if ( line.compare("c}") == 0 )
								{
									buildTFInfos(file_input, snap.parent_branch_);
									buildTFInfos(file_input, snap.child_branch_);
									break;
								}
							}

							snap.valid_ = true;
							snaps.push_back(snap);
						}
					}

					if ( snaps.size() > 0 )
						snapshots.push_back(snaps);
				}
			}
		}
		else
		{
			ROS_WARN("Failed to open %s, can't load snapshot data!", file_path.c_str());
			result = false;
		}

		file_input.close();
		return result;
	}

	void buildTFInfos(std::fstream &file, std::vector<TFInfo> &TFInfos)
	{
		while ( !file.eof() )
		{
			std::string line = "";
			std::getline(file, line);

			if ( line.compare("{}") == 0 )
				break;
			else
			{
				TFInfo info;

				info.parent_ = line;
				std::getline(file, info.child_);
				std::string trafo = "";
				std::getline(file, trafo);
				stringToTrafo(trafo, info.transform_);

				TFInfos.push_back(info);
			}
		}
	}

	void stringToTrafo(std::string str, cv::Mat &trafo)
	{
		const std::string delimiter = ",";
		size_t npos = 0;
		std::vector<double> nums;

		trafo.release();

		while ( (npos = str.find(delimiter)) != std::string::npos )
		{
			double temp = std::stod(str.substr(0, npos));
			nums.push_back(temp);
			str = str.substr(npos+1, (int)*str.end()-npos);
		}
		nums.push_back(std::stod(str));  // add rest as well

		if ( nums.size() == 16 )
		{
			trafo = trafo = ( cv::Mat_<double>(4,4) <<
					nums[0], nums[1], nums[2], nums[3],
					nums[4], nums[5], nums[6], nums[7],
					nums[8], nums[9], nums[10], nums[11],
					nums[12], nums[13], nums[14], nums[15]);
		}
		else
		{
			ROS_WARN("String does not contain amount of values for a transformation (exactly 16 values needed).");

			trafo = ( cv::Mat_<double>(4,4) <<
							1., 0., 0., 0.,
							0., 1., 0., 0.,
							0., 0., 1., 0.,
							0., 0., 0., 1.);
		}
	}

	void saveCalibrationSetups(const std::vector<CalibrationSetup> &calibration_setups, const std::string &save_path, const std::string &file_name)
	{
		std::stringstream stream("");

		for ( int i=0; i<calibration_setups.size(); ++i )
		{
			stream << "1{" << std::endl;
			stream << calibration_setups[i].origin_ << std::endl;

			for ( CalibrationInfo info : calibration_setups[i].uncertainties_list_ )
			{
				stream << "2{" << std::endl;
				stream << info.parent_ << std::endl;
				stream << info.child_ << std::endl;
				formatStringVector(stream, info.parent_markers_);
				formatStringVector(stream, info.child_markers_);
				stream << trafoToString(info.current_trafo_) << std::endl;
				stream << info.parent_branch_uncertainty_ << std::endl;
			}

			stream << "2}" << std::endl;
			formatStringVector(stream, calibration_setups[i].parent_branch_);
			formatStringVector(stream, calibration_setups[i].child_branch_);
		}

		std::fstream file_output;
		std::string file_path = save_path+"/"+file_name;
		file_output.open(file_path.c_str(), std::ios::out | std::ios::trunc);  // replace old file -> every new calibration run generates fresh data
		if (file_output.is_open())
			file_output << "Created at: " << time_utilities::getCurrentTimeStamp() << std::endl << stream.str();
		else
			ROS_WARN("Failed to open %s, can't save calibration setups!", file_path.c_str());
		file_output.close();
	}

	void formatStringVector(std::stringstream &stream, const std::vector<std::string> string_vector)
	{
		for ( std::string str : string_vector )
			stream << str << std::endl;

		stream << "{}" << std::endl;
	}

	bool loadCalibrationSetups(std::vector<CalibrationSetup> &calibration_setups, const std::string &load_path, const std::string &file_name)
	{
		bool result = true;
		std::fstream file_input;
		std::string file_path = load_path+"/"+file_name;
		file_input.open(file_path.c_str(), std::ios::in);  // read calibration setup content
		if ( file_input.is_open() )
		{
			while ( !file_input.eof() )
			{
				std::string line = "";
				std::getline(file_input, line);

				if ( line.compare("1{") == 0 )
				{
					CalibrationSetup setup;

					std::getline(file_input, setup.origin_);
					bool corrupted = true;

					while ( !file_input.eof() )
					{
						line = "";
						std::getline(file_input, line);

						if ( line.compare("2{") == 0 )
						{
							CalibrationInfo info;

							std::getline(file_input, info.parent_);
							std::getline(file_input, info.child_);
							buildStringVector(file_input, info.parent_markers_);
							buildStringVector(file_input, info.child_markers_);
							std::string trafo = "";
							std::getline(file_input, trafo);
							stringToTrafo(trafo, info.current_trafo_);
							std::string parent_branch_uncertainty = "";
							std::getline(file_input, parent_branch_uncertainty);
							info.parent_branch_uncertainty_ = (bool)std::stoi(parent_branch_uncertainty);
							info.calibrated_ = false;

							setup.uncertainties_list_.push_back(info);
						}
						else if ( line.compare("2}") == 0 )
						{
							buildStringVector(file_input, setup.parent_branch_);
							buildStringVector(file_input, setup.child_branch_);
							corrupted = false;
							break;
						}
					}

					if ( !corrupted )
						calibration_setups.push_back(setup);
				}

			}
		}
		else
		{
			ROS_WARN("Failed to open %s, can't load calibration setups!", file_path.c_str());
			result = false;
		}

		return result;
	}

	void buildStringVector(std::fstream &file, std::vector<std::string> &string_vector)
	{
		while ( !file.eof() )
		{
			std::string line = "";
			std::getline(file, line);

			if ( line.compare("{}") == 0 )
				break;
			else
				string_vector.push_back(line);
		}
	}
}



