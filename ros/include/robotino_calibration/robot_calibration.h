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

#include <robotino_calibration/timer.h>
#include <robotino_calibration/file_utilities.h>


class RobotCalibration
{
public:

    RobotCalibration(ros::NodeHandle nh, CalibrationInterface* interface, const bool load_data_from_disk);
    ~RobotCalibration();

    bool startCalibration();  // starts the calibration process


protected:

    bool getOrigin(const std::string last_parent_branch_frame, const std::string last_child_branch_frame, std::string &origin);  // returns mutual frame of parent_marker and child_marker back-chains

    bool isPartOfCalibrationSetup(const std::string parent, const std::string child, const std::string origin, const CalibrationSetup &setup);  // returns whether passed uncertainty is part of passed calibration setup

    void feedCalibrationSetup(CalibrationSetup &setup, const std::string parent, const std::string child,
    							const std::string parent_marker, const std::string child_marker);  // extend existing calibration setup by new information given

    bool getBackChain(const std::string frame_start, const std::string frame_end, std::vector<std::string> &backchain);

    void getForwardChain(const std::vector<std::string> &backchain, std::vector<std::string> &forwardchain);  // takes backchain and reverses it

    void sortUncertainties(const bool parent_branch, CalibrationSetup &setup, std::vector<CalibrationInfo> &sorted_uncertainties);  // sorts parent- and child-branch uncertainties of passed calibration setup

    void truncateBranch(std::vector<std::string> &branch, std::vector<CalibrationInfo> &branch_uncertainties);

    bool acquireTFData();

    void populateTFSnapshot(const CalibrationSetup &setup, TFSnapshot &snapshot);

    std::vector<TFInfo>* getBranchEndToMarkers(const int uncertainty_index, const bool parent_markers, TFSnapshot &snapshot);

    bool buildTransformChain(const std::string start, const std::string end, const std::vector<TFInfo> &branch, cv::Mat &trafo);  // returns the transform between two arbitrary points in a branch

    bool retrieveTransform(const std::string parent, const std::string child, const std::vector<TFInfo> &branch, cv::Mat &trafo);  // returns the transform between two points in a branch that are neighbours

    bool extrinsicCalibration(const int current_setup_idx, const int current_uncertainty_idx);

    // displays the calibration result on the screen and also stores it to a file in the urdf file's format
    void displayAndSaveCalibrationResult();


    int optimization_iterations_;	// number of iterations for optimization
    bool calibrated_;  // calibration has successfully been finished
    bool load_data_from_disk_;
    tf::TransformListener transform_listener_;
    ros::NodeHandle node_handle_;
    std::string calibration_storage_path_;  // path to data
    std::string calib_data_folder_;
    std::string calib_data_file_name_;
    CalibrationInterface *calibration_interface_;
    std::vector<CalibrationSetup> calibration_setups_;
    std::vector< std::vector<TFSnapshot> > tf_snapshots_;  // each robot configuration has calibration setup count snapshopts


};


#endif /* ROBOT_CALIBRATION_H_ */
