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
 * Date of creation: October 2016
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

#ifndef ROBOT_CALIBRATION_H_
#define ROBOT_CALIBRATION_H_


// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <robotino_calibration/calibration_interface.h>
#include <opencv2/opencv.hpp>
#include <vector>


struct CalibrationInfo  // defines one uncertain transform in the kinematic chain
{
    std::string parent_;  // parent frame: start point of the vector
    std::string child_;  // child frame: end point of the vector
	std::vector<std::string> parent_markers_;  // marker frame one reaches from parent frame backwards
	std::vector<std::string> child_markers_;  // marker_frame one reaches from child frame onwards
    cv::Mat current_trafo_;
    //int weight_;  // determines the order of calibration. The highest weighted transform in a setup will be calibrated first.
    //int trafo_until_next_gap_idx_;  // index to between_gaps trafo
};

struct CalibrationSetup  // defines one calibration setup, consisting of x transforms to be calibrated via parent and child marker
{
	bool calibrated_;  // marks whether this setup has already been done
	std::string origin_;  // this is not the robot's base, but the frame where two transformations chains meet
	std::vector<CalibrationInfo> origin_to_parent_marker_uncertainties_;  // parent branch uncertainties, sorted
	std::vector<CalibrationInfo> origin_to_child_marker_uncertainties_;  // child branch uncertainties, sorted
	std::vector<CalibrationInfo> uncertainties_list_;  // unsorted list of uncertainties
	std::vector<std::string> parent_branch;  // contains all frames from origin up to the frame before parent_markers
	std::vector<std::string> child_branch;  // contains all frames from origin up to the frame before child_markers
};

struct TFInfo  // used to make snapshots from tf tree
{
	std::string parent_;
	std::string child_;
	cv::Mat transform_;
};

struct TFSnapshot
{
	std::vector< std::vector<TFInfo> > parent_branch_parent_markers_;  // each uncertain trafo on parent branch can have a different set of parent_markers
	std::vector< std::vector<TFInfo> > parent_branch_child_markers_;  // each uncertain trafo on parent branch can have a different set of child_markers
	std::vector< std::vector<TFInfo> > child_branch_parent_markers_;  // same for child branch
	std::vector< std::vector<TFInfo> > child_branch_child_markers_;
	std::vector<TFInfo> parent_branch_;
	std::vector<TFInfo> child_branch_;
};


class RobotCalibration
{
public:

    RobotCalibration(ros::NodeHandle nh, CalibrationInterface* interface);
    virtual ~RobotCalibration();

    bool startCalibration(const bool load_data_from_drive);  // starts the calibration process


protected:

    void createStorageFolder();

    bool getOrigin(const std::string parent_marker, const std::string child_marker, std::string &origin);  // returns mutual frame of parent_marker and child_marker back-chains

    bool getChain(const std::string parent_marker, const std::string child_marker, const std::string origin,
    				std::vector<std::string> &parent_marker_to_origin, std::vector<std::string> &child_marker_to_origin);

    void feedCalibrationSetup(CalibrationSetup &setup, const std::string parent, const std::string child,
    							const std::string parent_marker, const std::string child_marker);  // extend existing calibration setup by new information given

    bool acquireTFData(const bool load_data);

    void populateTFSnapshot(const CalibrationSetup &setup);

    bool isParentBranchUncertainty(const CalibrationSetup &setup, const int uncertainty_idx, int &branch_idx);  // returns whether uncertainty is part of parent branch, stores found index (for both parent and child branch)

    bool buildTransformChain(const std::string start, const std::string end, const std::vector<TFInfo> &branch, cv::Mat &trafo);  // returns the transform between two arbitrary points in a branch

    bool retrieveTransform(const std::string parent, const std::string child, const std::vector<TFInfo> &branch, cv::Mat &trafo);  // returns the transform between two points in a branch that are neighbours

    bool extrinsicCalibration(const CalibrationSetup &setup, const int current_uncertainty_idx);

    // displays the calibration result on the screen and also stores it to a file in the urdf file's format
    void displayAndSaveCalibrationResult(std::string output_file_name);


    int optimization_iterations_;	// number of iterations for optimization
    bool calibrated_;  // calibration has successfully been finished
    tf::TransformListener transform_listener_;
    ros::NodeHandle node_handle_;
    std::string calibration_storage_path_;  // path to data
    CalibrationInterface *calibration_interface_;
    std::vector<CalibrationSetup> calibration_setups_;
    std::vector<TFSnapshot> tf_snapshots;  // each robot configuration has its own TFSnapshot vector

};


#endif /* ROBOT_CALIBRATION_H_ */
