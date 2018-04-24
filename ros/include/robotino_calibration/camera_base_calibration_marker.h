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
 * Date of creation: August 2016
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

#ifndef CAMERA_BASE_CALIBRATION_MARKER_H_
#define CAMERA_BASE_CALIBRATION_MARKER_H_


#include <robotino_calibration/calibration_utilities.h>
#include <robotino_calibration/robot_calibration.h>

#define REF_FRAME_HISTORY_SIZE 15 // 15 entries used to build the moving average upon


class CameraBaseCalibrationMarker : public RobotCalibration
{
public:

    CameraBaseCalibrationMarker(ros::NodeHandle nh, CalibrationInterface* interface);
    virtual ~CameraBaseCalibrationMarker();

    // starts the calibration between camera and base including data acquisition
    virtual bool calibrateCameraToBase(const bool load_images) = 0;


protected:

    // moves the robot to a desired location and adjusts the torso joints
    void moveRobot(int config_index); // derived from parent
    bool moveBase(const calibration_utilities::BaseConfiguration &base_configuration);

    // Turn off base movement
    void turnOffBaseMotion();

    bool isReferenceFrameValid(cv::Mat &T); // Returns wether reference frame is valid -> if so, it is save to move the robot base, otherwise stop!

    std::string base_frame_;        // Name of base frame, needed for security measure
    std::string child_frame_name_;  // name of reference frame, needed for security measure

    //std::vector<calibration_utilities::RobotConfiguration> robot_configurations_;  // wished robot configurations used for calibration
    double ref_frame_history_[REF_FRAME_HISTORY_SIZE]; // History of base_frame to reference_frame squared lengths, used to get average squared length. Holds last <REF_FRAME_HISTORY_SIZE> measurements.
    int ref_history_index_; // Current index of history building
    double max_ref_frame_distance_;

    std::vector<calibration_utilities::BaseConfiguration> base_configurations_;  // wished base configurations used for calibration


private:

    double last_ref_history_update_;  // used to update the ref_frame_history_ array cyclically and not upon every call of isReferenceFrameValid()
};



#endif // CAMERA_BASE_CALIBRATION_MARKER_H_
