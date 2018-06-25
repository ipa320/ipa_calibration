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
 * Date of creation: June 2018
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

#ifndef CALIBRATION_TYPE_H_
#define CALIBRATION_TYPE_H_


#include <calibration_interface/ipa_interface.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>
#include <std_msgs/Float64MultiArray.h>


#define NUM_MOVE_TRIES 4


class CalibrationType
{

protected:

	bool moveCamera(const std::vector<double> &cam_configuration);


	ros::NodeHandle node_handle_;
    tf::TransformListener transform_listener_;
	IPAInterface *calibration_interface_;

    int camera_dof_;  // degrees of freedom the camera has
    std::vector< std::vector<double> > camera_configurations_;  // wished camera configurations. Can be used to calibrate the whole workspace of the arm. Extracted from robot_configurations (yaml)
	std::vector<std::string> uncertainties_list_;
    bool initialized_;


public:

	CalibrationType();
	virtual void initialize(ros::NodeHandle nh, IPAInterface* calib_interface);
	virtual ~CalibrationType();

	virtual bool moveRobot(int config_index);
	int getConfigurationCount();
	void getUncertainties(std::vector<std::string> &uncertainties_list);
};


#endif /* CALIBRATION_TYPE_H_ */
