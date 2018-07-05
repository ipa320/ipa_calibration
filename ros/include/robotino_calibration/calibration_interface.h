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
 * Date of creation: January 2017
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

#ifndef CALIBRATION_INTERFACE_H_
#define CALIBRATION_INTERFACE_H_


#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>


class CalibrationInterface
{

protected:

	ros::NodeHandle node_handle_;


public:

	CalibrationInterface();
	CalibrationInterface(ros::NodeHandle nh);
	virtual ~CalibrationInterface();

	// apply new configuration to robot
	virtual bool moveRobot(int current_index) = 0;

	// get the amount of robot (movement) configurations that have been created by user
	virtual int getConfigurationCount() = 0;

	// give user the chance to execute some code before tf tree will be snapshotted (e.g. wait for transforms to be ready, wait to mitigate shaking effects in robot's kinematic after moving)
	virtual void preSnapshot(int current_index) = 0;

	// get the pattern points (in 3 dimensions) for each marker in local marker's frame. markers can have different patterns, hence one can mix pitags, checkerboards, etc...
	virtual void getPatternPoints3D(const std::string marker_frame, std::vector<cv::Point3f> &pattern_points_3d) = 0;

	// returns the uncertainties list which has to be set up like this: [parent frame, child frame, parent marker frame, child marker frame]
	virtual void getUncertainties(std::vector<std::string> &uncertainties_list) = 0;

	// returns the file name (including file extension) in which the calibration results will be stored to
	virtual std::string getResultFileName() = 0;

};


#endif /* CALIBRATION_INTERFACE_H_ */
