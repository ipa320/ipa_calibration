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

#ifndef CAMERA_ARM_TYPE_H_
#define CAMERA_ARM_TYPE_H_


#include <calibration_interface/calibration_type.h>


struct arm_description
{
	std::string arm_name_;
	int dof_count_;
	double max_delta_angle_;  // in [rad]
	std::vector< std::vector<double> > configurations_;  // wished arm configurations used for calibration
};

class CameraArmType : public CalibrationType
{

public:

	CameraArmType();
	~CameraArmType();

	bool moveRobot(int config_index);
	std::string getString();


protected:

	void initialize(ros::NodeHandle* nh, IPAInterface* calib_interface);

	bool moveCameras(int config_index);
	unsigned short moveArm(const arm_description &arm, const std::vector<double>& arm_configuration);
	std::vector<arm_description> arms_;


private:

	int current_arm_index_;  // mapped index that accesses the correct vector element of the base_configurations_ vector
	int current_camera_index_;  // mapped index that accesses the correct vector element of the currently moved camera


};


#endif /* CAMERA_ARM_TYPE_H_ */
