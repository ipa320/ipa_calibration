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
 * Date of creation: September 2017
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

#ifndef COB_INTERFACE_H_
#define COB_INTERFACE_H_

#include <calibration_interface/custom_interface.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread/mutex.hpp>

class CobInterface : public CustomInterface
{
protected:
	std::string arm_left_command_;
	std::string arm_left_state_topic_;
	std::string arm_right_command_;
	std::string arm_right_state_topic_;
	std::string base_velocity_command_;
	std::string camera_joint_controller_command_;
	std::string camera_joint_state_topic_;
	ros::Subscriber arm_left_state_;
	ros::Subscriber arm_right_state_;
	ros::Subscriber camera_state_;
	ros::Publisher arm_left_controller_;
	ros::Publisher arm_right_controller_;
	ros::Publisher base_velocity_controller_;
	ros::Publisher camera_joint_controller_;

public:
	CobInterface(ros::NodeHandle nh, bool do_arm_calibration);
	~CobInterface();

	// camera calibration interface
	void assignNewRobotVelocity(geometry_msgs::Twist newVelocity);
	void assignNewCameraAngles(std_msgs::Float64MultiArray newAngles);
	std::vector<double>* getCurrentCameraState();

	// callbacks
	void cameraStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
	void armLeftStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
	void armRightStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

	// arm calibration interface
	void assignNewArmJoints(std_msgs::Float64MultiArray newJointConfig);
	std::vector<double>* getCurrentArmState();
};

#endif /* COB_INTERFACE_H_ */
