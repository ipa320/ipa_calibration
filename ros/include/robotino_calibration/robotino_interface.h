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
 * Date of creation: February 2017
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

#ifndef ROBOTINO_INTERFACE_H_
#define ROBOTINO_INTERFACE_H_

#include <robotino_calibration/calibration_interface.h>
#include <dynamixel_msgs/JointState.h>
#include <boost/thread/mutex.hpp>

class RobotinoInterface : public CalibrationInterface
{
protected:
	ros::Publisher arm_joint_controller_;
	std::string arm_joint_controller_command_;
	ros::Subscriber pan_state_;
	ros::Subscriber tilt_state_;
	ros::Publisher tilt_controller_;
	ros::Publisher pan_controller_;
	std::string tilt_controller_command_;
	std::string pan_controller_command_;

	double pan_joint_state_current_;
	double tilt_joint_state_current_;
	boost::mutex pan_tilt_joint_state_data_mutex_;	// secures read operations on pan tilt joint state data
	std::string tilt_joint_state_topic_;
	std::string pan_joint_state_topic_;

public:
	RobotinoInterface(ros::NodeHandle nh, bool bArmCalibration);
	~RobotinoInterface();

	// camera calibration interface
	void assignNewRobotVelocity(geometry_msgs::Twist newVelocity);
	void assignNewCamaraPanAngle(std_msgs::Float64 newPan);
	void assignNewCamaraTiltAngle(std_msgs::Float64 newTilt);
	double getCurrentCameraTiltAngle();
	double getCurrentCameraPanAngle();

	// callbacks
	void panJointStateCallback(const dynamixel_msgs::JointState::ConstPtr& msg);
	void tiltJointStateCallback(const dynamixel_msgs::JointState::ConstPtr& msg);

	// arm calibration interface
	void assignNewArmJoints(std_msgs::Float64MultiArray newJointConfig);
};


#endif /* ROBOTINO_INTERFACE_H_ */
