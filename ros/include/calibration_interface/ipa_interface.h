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
 * Date of creation: January 2018
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

#ifndef CUSTOM_INTERFACE_H_
#define CUSTOM_INTERFACE_H_


#include <robotino_calibration/calibration_interface.h>
#include <calibration_interface/calibration_type.h>
#include <calibration_interface/calibration_marker.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <vector>


// robot types
enum RobotTypes
{
    ROB_ROBOTINO	= 0,
    ROB_RAW_3_1		= 1,
    ROB_COB			= 2
};


class IPAInterface : public CalibrationInterface
{

protected:

	ros::NodeHandle node_handle_;
	CalibrationType* calibration_type_;
	CalibrationMarker* calibration_marker_;
	bool arm_calibration_;
	bool load_data_;  // load stored calibration data from disk -> offline calibration


public:

	IPAInterface();
	IPAInterface(ros::NodeHandle nh, CalibrationType* calib_type, CalibrationMarker* calib_marker, bool do_arm_calibration, bool load_data);
	virtual ~IPAInterface();

	static CalibrationInterface* createInterfaceByID(int ID, ros::NodeHandle nh, CalibrationType* calib_type, CalibrationMarker* calib_marker, bool do_arm_calibration, bool load_data); //Create corresponding robot interface by a user-defined ID.

	bool moveRobot(int config_index);
	int getConfigurationCount();

	void preSnapshot(int current_index);
	void getPatternPoints3D(const std::string marker_frame, std::vector<cv::Point3f> &pattern_points_3d);
	void getUncertainties(std::vector<std::string> &uncertainties_list);
	std::string getFileName(const std::string &appendix, const bool file_extension);

	virtual std::string getRobotName();

	// camera calibration interface
	virtual void assignNewRobotVelocity(geometry_msgs::Twist newVelocity) = 0;
	virtual void assignNewCameraAngles(std_msgs::Float64MultiArray newAngles) = 0;
	virtual std::vector<double>* getCurrentCameraState() = 0;

	// arm calibration interface
	virtual void assignNewArmJoints(std_msgs::Float64MultiArray newJointConfig) = 0;
	virtual std::vector<double>* getCurrentArmState() = 0;
};


#endif /* CUSTOM_INTERFACE_H_ */
