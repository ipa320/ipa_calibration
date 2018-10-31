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


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>


#define NUM_MOVE_TRIES 4

enum MoveErrorCode
{
    MOV_NO_ERR		= 0,	// Everthing is ok
    MOV_ERR_SOFT	= 1,	// Retry to move after a delay
    MOV_ERR_FATAL	= 2		// No retry
};


class IPAInterface;  // forward declaration

struct camera_description
{
	std::string camera_name_;
	int dof_count_;
	double max_delta_angle_;  // in [rad]
	std::vector< std::vector<double> > configurations_;  // wished camera configurations. Can be used to calibrate the whole workspace of the arm.
};

class CalibrationType
{

protected:

	virtual bool moveCameras(int config_index) = 0;  // determines when and how cameras will be moved, the actual movement however is done in moveCamera(). Has to be implemented in child classes
	unsigned short moveCamera(const camera_description &camera, const std::vector<double> &cam_configuration);
	bool generateConfigs(const std::vector< std::vector<double> > &param_vector, std::vector< std::vector<double> > &configs);
	bool passesMaxDeltaAngleCheck(const std::vector<double> &state, const std::vector<double> &target, const double max_angle, int &bad_idx);


	ros::NodeHandle node_handle_;
    tf::TransformListener transform_listener_;
	IPAInterface *calibration_interface_;

	std::vector<std::string> uncertainties_list_;

	int max_configuration_count;  // set in child classes. it's used as exit condition for the calibration library
	std::vector<camera_description> cameras_;  // list that holds all robot cameras that are involved in calibration

    bool initialized_;  // set this in child class once everything has initialized


public:

	CalibrationType();
	virtual void initialize(ros::NodeHandle nh, IPAInterface* calib_interface);
	virtual ~CalibrationType();

	virtual bool moveRobot(int config_index);
	virtual std::string getString() = 0;
	int getConfigurationCount();
	void getUncertainties(std::vector<std::string> &uncertainties_list);
};


#endif /* CALIBRATION_TYPE_H_ */
