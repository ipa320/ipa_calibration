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

#ifndef FILE_UTILITIES_H_
#define FILE_UTILITIES_H_


#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>


struct CalibrationInfo  // defines one uncertain transform in the kinematic chain
{
    std::string parent_;  // parent frame: start point of the vector
    std::string child_;  // child frame: end point of the vector
	std::vector<std::string> parent_markers_;  // marker frame one reaches from parent_ frame backwards
	std::vector<std::string> child_markers_;  // marker_frame one reaches from child_ frame onwards
    cv::Mat current_trafo_;
    bool calibrated_;  // marks whether this uncertainty has already been calibrated
	bool parent_branch_uncertainty_;  // defines where this uncertainty lies: on parent- or child-branch
};

struct CalibrationSetup  // defines one calibration setup, consisting of x transforms to be calibrated via parent and child marker
{
	std::string origin_;  // this is not the robot's base, but the frame where two transformations chains meet
	std::vector<CalibrationInfo> uncertainties_list_;  // unsorted list of uncertainties
	std::vector<std::string> parent_branch_;  // contains all frames from origin up to the last parent-branch uncertainty's child
	std::vector<std::string> child_branch_;  // contains all frames from origin up to the last child-branch uncertainty's child
};

struct TFInfo  // used to make snapshots from tf tree
{
	std::string parent_;
	std::string child_;
	cv::Mat transform_;
};

// Used to snapshot the tf transform between the last frame of a branch to the corresponding markers
struct TFBranchEndsToMarkers
{
	std::vector<TFInfo> branch_to_child_markers_;  // child markers are always on the branch where the uncertainty lies
	std::vector<TFInfo> otherbranch_to_parent_markers_;  // parent markers are always on the other branch (which uncertainty is no part part of)
	int corresponding_uncertainty_idx_;
};

struct TFSnapshot
{
	std::vector<TFBranchEndsToMarkers> branch_ends_to_markers_;  // includes trafo between end of both parent- and child-branch to each child and parent marker of an uncertainty
	std::vector<TFInfo> parent_branch_;
	std::vector<TFInfo> child_branch_;
	bool valid_;  // whether this snapshot contains consistent data
};


namespace file_utilities
{

	void createStorageFolder(const std::string &folder_path);

	void saveCalibrationResult(const std::string &file_path, const std::string &content);

	void saveSnapshots(const std::vector< std::vector<TFSnapshot> > &snapshots, const std::string &save_path, const std::string &file_name);

	void formatBETMs(std::stringstream &stream, const std::vector<TFBranchEndsToMarkers> &BETMs);

	void formatTFInfos(std::stringstream &stream, const std::vector<TFInfo> &TFInfos);

	std::string trafoToString(const cv::Mat &trafo);

	void buildTFInfos(std::fstream &file, std::vector<TFInfo> &TFInfos);

	void stringToTrafo(std::string str, cv::Mat &trafo);

	bool loadSnapshots(std::vector< std::vector<TFSnapshot> > &snapshots, const std::string &load_path, const std::string &file_name);

	void saveCalibrationSetups(const std::vector<CalibrationSetup> &calibration_setups, const std::string &save_path, const std::string &file_name);

	void formatStringVector(std::stringstream &stream, const std::vector<std::string> string_vector);

	bool loadCalibrationSetups(std::vector<CalibrationSetup> &calibration_setups, const std::string &load_path, const std::string &file_name);

	void buildStringVector(std::fstream &file, std::vector<std::string> &string_vector);

}


#endif /* FILE_UTILITIES_H_ */
