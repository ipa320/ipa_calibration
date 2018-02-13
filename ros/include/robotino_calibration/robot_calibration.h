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


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <robotino_calibration/calibration_utilities.h>
#include <robotino_calibration/calibration_interface.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#define NUM_MOVE_TRIES 5


struct CalibrationInfo
{
    std::string parent_;
    std::string child_;
    cv::Mat current_trafo_;
    int trafo_until_next_gap_idx_; // index to between_gaps trafo
};

class RobotCalibration
{
public:

    RobotCalibration(ros::NodeHandle nh, CalibrationInterface* interface);
    virtual ~RobotCalibration();


protected:

    void createStorageFolder();

    // displays the calibration result in the urdf file's format and also stores the screen output to a file
    void displayAndSaveCalibrationResult(std::string output_file_name);

    void extrinsicCalibration(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
                              std::vector<cv::Mat>& T_gapfirst_to_marker_vector, std::vector< std::vector<cv::Mat> > T_between_gaps_vector,
                              std::vector<cv::Mat>& T_gaplast_to_marker_vector, int trafo_to_calibrate);

    bool calculateTransformationChains(cv::Mat& T_gapfirst_to_marker, std::vector<cv::Mat> T_between_gaps,
                                       cv::Mat& T_gaplast_to_marker, std::string marker_frame);

    virtual void moveRobot(int config_index);
    bool moveCamera(const std::vector<double>& cam_configuration);

    int camera_dof_;		// degrees of freedom the camera has
    int optimization_iterations_;	// number of iterations for optimization
    bool calibrated_;
    tf::TransformListener transform_listener_;
    ros::NodeHandle node_handle_;
    std::string camera_optical_frame_;  // name of camera optical frame
    std::string calibration_storage_path_;  // path to data
    std::string child_frame_name_;  // name of reference frame
    CalibrationInterface *calibration_interface_;
    std::vector<CalibrationInfo> transforms_to_calibrate_;
    std::vector<int> calibration_order_;
    std::vector< std::vector<double> > camera_configurations_; // wished camera configurations. Can be used to calibrate the whole workspace of the arm. Extracted from robot_configurations (yaml)

};


#endif /* ROBOT_CALIBRATION_H_ */
