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

/* This class represents the interface between your robot and the calibration code.
 * Adjust the interface functions so that the calibration code will work correctly with your robot environment.
*/

#include <calibration_interface/ipa_interface.h>
#include <calibration_interface/robotino_interface.h>
#include <calibration_interface/raw_interface.h>
#include <calibration_interface/cob_interface.h>


IPAInterface::IPAInterface()
{
}

IPAInterface::IPAInterface(ros::NodeHandle nh, bool do_arm_calibration) :
				CalibrationInterface(nh), arm_calibration_(do_arm_calibration)
{
}

IPAInterface::~IPAInterface()
{
}

// You can add further interfaces for other robots in here.
CalibrationInterface* IPAInterface::createInterfaceByID(int ID, ros::NodeHandle nh, bool do_arm_calibration)
{
	switch(ID)
	{
		case ROB_ROBOTINO:
				return (new RobotinoInterface(nh, do_arm_calibration));
				break;
		case ROB_RAW_3_1:
				return (new RAWInterface(nh, do_arm_calibration));
				break;
		case ROB_COB:
				return (new CobInterface(nh, do_arm_calibration));
				break;
		default:
				return 0;
	}
}

bool IPAInterface::moveRobot(int index)
{
	return true;
}

bool IPAInterface::lastConfigurationReached(int index)
{
	return true;
}
