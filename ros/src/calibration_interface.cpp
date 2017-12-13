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

/* This class represents the interface between your robot and the calibration code.
 * Adjust the interface functions so that the calibration code will work correctly with your robot environment.
*/

#include <robotino_calibration/calibration_interface.h>
#include <robotino_calibration/robotino_interface.h>
#include <robotino_calibration/raw_interface.h>
#include <robotino_calibration/cob_interface.h>

// Robot types
#define Robotino	0
#define RobAtWork	1
#define CareOBot	2


CalibrationInterface::CalibrationInterface()
{
}

CalibrationInterface::CalibrationInterface(ros::NodeHandle nh) :
				node_handle_(nh)
{
}

CalibrationInterface::~CalibrationInterface()
{
}

// You can add further interfaces for other robots in here.
CalibrationInterface* CalibrationInterface::createInterfaceByID(int ID, ros::NodeHandle nh, bool bArmCalibration)
{
	switch(ID)
	{
		case Robotino:
				return (new RobotinoInterface(nh, bArmCalibration));
				break;
		case RobAtWork:
				return (new RAWInterface(nh, bArmCalibration));
				break;
		case CareOBot:
				return (new CobInterface(nh, bArmCalibration));
				break;
		default:
				return 0;
	}
}
