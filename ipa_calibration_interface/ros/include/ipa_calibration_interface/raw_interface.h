/****************************************************************
 *
 * Copyright (c) 2017
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: ipa_calibration
 * ROS stack name: ipa_calibration
 * ROS package name: ipa_calibration_interface
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Marc Riedlinger, email: m.riedlinger@live.de
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

#ifndef RAW_CALIBRATION_H_
#define RAW_CALIBRATION_H_

#include <ipa_calibration_interface/ipa_interface.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread/mutex.hpp>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


class RAWInterface : public IPAInterface
{
protected:
	ros::Publisher arm_joint_controller_;
	std::string arm_joint_controller_command_;
	ros::Publisher camera_joint_controller_;
	std::string camera_joint_controller_command_;
	std::string base_controller_topic_name_;
	ros::Publisher base_controller_;

	sensor_msgs::JointState* camera_state_current_;
	boost::mutex camera_state_data_mutex_;	// secures read operations on pan tilt joint state data
	std::string joint_state_topic_;

	std::string camera_joint_state_topic_;
	ros::Subscriber camera_state_;
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> camera_action_client_;

	std::string arm_state_topic_;
	ros::Subscriber arm_state_;
	sensor_msgs::JointState* arm_state_current_;
	boost::mutex arm_state_data_mutex_;	// secures read operations on arm state data
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_action_client_;


public:
	RAWInterface(ros::NodeHandle* nh, CalibrationType* calib_type, CalibrationMarker* calib_marker, bool do_arm_calibration, bool load_data);
	~RAWInterface();

	std::string getRobotName();

	// camera calibration interface
	void assignNewRobotVelocity(geometry_msgs::Twist new_velocity);
	void assignNewCameraAngles(const std::string &camera_name, std_msgs::Float64MultiArray new_camera_config);
	std::vector<double>* getCurrentCameraState(const std::string &camera_name);

	// callbacks
	void cameraStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
	void armStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

	// arm calibration interface
	void assignNewArmJoints(const std::string &arm_name, std_msgs::Float64MultiArray new_arm_config);
	std::vector<double>* getCurrentArmState(const std::string &arm_name);
};


#endif /* RAW_CALIBRATION_H_ */
