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


#include <calibration_interface/pose_definition.h>
#include <iostream>
#include <sstream>


namespace pose_definition
{
	RobotConfiguration::RobotConfiguration(const std::vector<double> &config) :
					pose_x_(0.), pose_y_(0.), pose_phi_(0.)
	{
		if ( config.size() != NUM_POSE_PARAMS )
		{
			std::cout << "pose_definition::RobotConfiguration: Error, passed vector does not have the correct size ("
					<< config.size() << ", needed " << NUM_POSE_PARAMS << ") to build a robot configuration!";
			return;
		}

		pose_x_ = config[0];
		pose_y_ = config[1];
		pose_phi_ = config[2];
	}

	void RobotConfiguration::assign(const int idx, const double value)
	{
		switch( idx )
		{
			case 0:	pose_x_ = value;
					break;
			case 1:	pose_y_ = value;
					break;
			case 2:	pose_phi_ = value;
		}
	}

	std::string RobotConfiguration::getString()
	{
		std::stringstream result;
		result << pose_x_ << "\t" << pose_y_ << "\t" << pose_phi_;
		return result.str();
	}
}























