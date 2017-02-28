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

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

class CalibrationInterface
{
protected:
	ros::NodeHandle node_handle_;

public:
	CalibrationInterface();
	CalibrationInterface(ros::NodeHandle nh);
	virtual ~CalibrationInterface();
	static CalibrationInterface* CreateInterfaceByID(int ID, ros::NodeHandle nh, bool bArmCalibration); //Create corresponding robot interface by a user-defined ID.

	// camera calibration interface
	virtual void assignNewRobotVelocity(geometry_msgs::Twist newVelocity) = 0;
	virtual void assignNewCamaraPanAngle(std_msgs::Float64 newPan) = 0;
	virtual void assignNewCamaraTiltAngle(std_msgs::Float64 newTilt) = 0;
	virtual double getCurrentCameraTiltAngle() = 0;
	virtual double getCurrentCameraPanAngle() = 0;

	// arm calibration interface
	virtual void assignNewArmJoints(std_msgs::Float64MultiArray newJointConfig) = 0;
};


#endif /* CALIBRATION_INTERFACE_H_ */
