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
#include <exception>
#include <iostream>


IPAInterface::IPAInterface()
{
}

IPAInterface::IPAInterface(ros::NodeHandle* nh, CalibrationType* calib_type, CalibrationMarker* calib_marker, bool do_arm_calibration, bool load_data) :
				CalibrationInterface(nh), calibration_type_(calib_type), calibration_marker_(calib_marker), arm_calibration_(do_arm_calibration), load_data_(load_data)
{
	if ( !load_data_ )  // calibration_type holds code for moving the robot which is in case of offline calibration not needed
	{
		if ( calibration_type_ != 0 )
			calibration_type_->initialize(node_handle_, this);
		else
		{
			ROS_FATAL("IPAInterface::IPAInterface - Calibration type has not been created!");
			throw std::exception();
		}
	}

	if ( calibration_marker_ != 0 )
		calibration_marker_->initialize(node_handle_);
	else
	{
		ROS_FATAL("IPAInterface::IPAInterface - Calibration marker has not been created!");
		throw std::exception();
	}
}

IPAInterface::~IPAInterface()
{
	if ( calibration_type_ != 0 )
		delete calibration_type_;

	if ( calibration_marker_ != 0 )
		delete calibration_marker_;
}

// You can add further interfaces for other robots in here.
CalibrationInterface* IPAInterface::createInterfaceByID(int ID, ros::NodeHandle* nh, CalibrationType* calib_type, CalibrationMarker* calib_marker, bool do_arm_calibration, bool load_data)
{
	switch(ID)
	{
		case ROB_ROBOTINO:
				return (new RobotinoInterface(nh, calib_type, calib_marker, do_arm_calibration, load_data));
				break;
		case ROB_RAW_3_1:
				return (new RAWInterface(nh, calib_type, calib_marker, do_arm_calibration, load_data));
				break;
		case ROB_COB:
				return (new CobInterface(nh, calib_type, calib_marker, do_arm_calibration, load_data));
				break;
		default:
				return 0;
	}
}

bool IPAInterface::moveRobot(int config_index)
{
	if ( calibration_type_ != 0 )
		return calibration_type_->moveRobot(config_index);
	else
	{
		ROS_ERROR("IPAInterface::moveRobot - Calibration type has not been created!");
		return false;
	}
}

int IPAInterface::getConfigurationCount()
{
	if ( calibration_type_ != 0 )
		return calibration_type_->getConfigurationCount();
	else
	{
		ROS_ERROR("IPAInterface::getConfigurationCount - Calibration type has not been created!");
		return 0;
	}
}

void IPAInterface::preSnapshot(int current_index)
{
	ros::Duration(calibration_marker_->getWaitTime()).sleep();  // wait for markers being detected properly
}

// we are not making use of marker_frame, as we do either use pitags or checkerboards throughout the whole calibration, so we do not mix markers
void IPAInterface::getPatternPoints3D(const std::string marker_frame, std::vector<cv::Point3f> &pattern_points_3d)
{
	if ( calibration_marker_ != 0 )
		calibration_marker_->getPatternPoints3D(pattern_points_3d);
	else
		ROS_ERROR("IPAInterface::getPatternPoints3D - Calibration marker has not been created!");
}

void IPAInterface::getUncertainties(std::vector<std::string> &uncertainties_list)
{
	if ( calibration_type_ != 0 )
		calibration_type_->getUncertainties(uncertainties_list);
	else
		ROS_ERROR("IPAInterface::getUncertainties - Calibration type has not been created!");
}

std::string IPAInterface::getFileName(const std::string &appendix, const bool file_extension)
{
	std::string file_name;

	file_name = getRobotName();
	file_name += "_";

	if ( calibration_type_ != 0 )
		file_name += calibration_type_->getString();
	else
		file_name += "unspecified";

	if ( calibration_marker_ != 0 )
		file_name += ("_"+calibration_marker_->getString());

	if ( !appendix.empty() )
		file_name += ("_"+appendix);

	if ( file_extension )
		file_name += ".txt";

	return file_name;
}

std::string IPAInterface::getRobotName()
{
	return "IPA-Robot";
}
