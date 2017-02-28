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
 * Date of creation: August 2016
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


#include <robotino_calibration/camera_base_calibration_marker.h>
#include <robotino_calibration/transformation_utilities.h>

#include <std_msgs/Float64.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <sstream>
#include <fstream>

//#include <robotino_calibration/robotino_interface.h>
//#include <robotino_calibration/raw_interface.h>


CameraBaseCalibrationMarker::CameraBaseCalibrationMarker(ros::NodeHandle nh) :
			RobotCalibration(nh), counter(0)
{
	// load parameters
	std::cout << "\n========== CameraBaseCalibrationMarker Parameters ==========\n";
	// coordinate frame name parameters
	node_handle_.param<std::string>("torso_lower_frame", torso_lower_frame_, "base_pan_link");
	std::cout << "torso_lower_frame: " << torso_lower_frame_ << std::endl;
	node_handle_.param<std::string>("torso_upper_frame", torso_upper_frame_, "tilt_link");
	std::cout << "torso_upper_frame: " << torso_upper_frame_ << std::endl;
	node_handle_.param<std::string>("camera_frame", camera_frame_, "kinect_link");
	std::cout << "camera_frame: " << camera_frame_ << std::endl;
	node_handle_.param<std::string>("camera_optical_frame", camera_optical_frame_, "kinect_rgb_optical_frame");
	std::cout << "camera_optical_frame: " << camera_optical_frame_ << std::endl;
	node_handle_.param<std::string>("pan_controller_command", pan_controller_command_, "/pan_controller/command");
	std::cout << "pan_controller_command: " << pan_controller_command_ << std::endl;
	node_handle_.param<std::string>("tilt_controller_command", tilt_controller_command_, "/tilt_controller/command");
	std::cout << "tilt_controller_command: " << tilt_controller_command_ << std::endl;
	//node_handle_.param<std::string>("pan_joint_state_topic", pan_joint_state_topic_, "/pan_controller/state");
	//std::cout << "pan_joint_state_topic: " << pan_joint_state_topic_ << std::endl;
	//node_handle_.param<std::string>("tilt_joint_state_topic", tilt_joint_state_topic_, "/tilt_controller/state");
	//std::cout << "tilt_joint_state_topic: " << tilt_joint_state_topic_ << std::endl;
	node_handle_.param<std::string>("base_controller_topic_name", base_controller_topic_name_, "/cmd_vel");
	std::cout << "base_controller_topic_name: " << base_controller_topic_name_ << std::endl;

	// deprecated
	//node_handle_.param<std::string>("joint_state_topic", joint_state_topic_, "/pan_tilt_controller/joint_states");
	//std::cout << "joint_state_topic: " << joint_state_topic_ << std::endl;

	// initial parameters
	T_base_to_torso_lower_ = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(0.0, 0.0, 0.0), cv::Mat(cv::Vec3d(0.25, 0, 0.5)));
	T_torso_upper_to_camera_ = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(0.0, 0.0, -1.57), cv::Mat(cv::Vec3d(0.0, 0.065, 0.0)));
	std::vector<double>temp;
	node_handle_.getParam("T_base_to_torso_lower_initial", temp);
	if (temp.size()==6)
		T_base_to_torso_lower_ = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(temp[3], temp[4], temp[5]), cv::Mat(cv::Vec3d(temp[0], temp[1], temp[2])));
	std::cout << "T_base_to_torso_lower_initial:\n" << T_base_to_torso_lower_ << std::endl;
	temp.clear();
	node_handle_.getParam("T_torso_upper_to_camera_initial", temp);
	if (temp.size()==6)
		T_torso_upper_to_camera_ = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(temp[3], temp[4], temp[5]), cv::Mat(cv::Vec3d(temp[0], temp[1], temp[2])));
	std::cout << "T_torso_upper_to_camera_initial:\n" << T_torso_upper_to_camera_ << std::endl;

	bool use_range = false;
	node_handle_.param("use_range", use_range, false);
	std::cout << "use_range: " << use_range << std::endl;
	if (use_range == true)
	{
		// create robot configurations from regular grid
		std::vector<double> x_range;
		node_handle_.getParam("x_range", x_range);
		std::vector<double> y_range;
		node_handle_.getParam("y_range", y_range);
		std::vector<double> phi_range;
		node_handle_.getParam("phi_range", phi_range);
		std::vector<double> pan_range;
		node_handle_.getParam("pan_range", pan_range);
		std::vector<double> tilt_range;
		node_handle_.getParam("tilt_range", tilt_range);
		if (x_range.size()!=3 || y_range.size()!=3 || phi_range.size()!=3 || pan_range.size()!=3 || tilt_range.size()!=3)
		{
			ROS_ERROR("One of the range vectors has wrong size.");
			return;
		}
		if (x_range[0] == x_range[2] || x_range[1] == 0.)		// this sets the step to something bigger than 0
			x_range[1] = 1.0;
		if (y_range[0] == y_range[2] || y_range[1] == 0.)
			y_range[1] = 1.0;
		if (phi_range[0] == phi_range[2] || phi_range[1] == 0.)
			phi_range[1] = 1.0;
		if (pan_range[0] == pan_range[2] || pan_range[1] == 0.)
			pan_range[1] = 1.0;
		if (tilt_range[0] == tilt_range[2] || tilt_range[1] == 0.)
			tilt_range[1] = 1.0;
		for (double x=x_range[0]; x<=x_range[2]; x+=x_range[1])
			for (double y=y_range[0]; y<=y_range[2]; y+=y_range[1])
				for (double phi=phi_range[0]; phi<=phi_range[2]; phi+=phi_range[1])
					for (double pan=pan_range[0]; pan<=pan_range[2]; pan+=pan_range[1])
						for (double tilt=tilt_range[0]; tilt<=tilt_range[2]; tilt+=tilt_range[1])
							robot_configurations_.push_back(calibration_utilities::RobotConfiguration(x, y, phi, pan, tilt));
		std::cout << "Generated " << (int)robot_configurations_.size() << " robot configurations for calibration." << std::endl;
		if ((int)robot_configurations_.size() == 0)
			ROS_WARN("No robot configurations generated. Please check your ranges in the yaml file.");
	}
	else
	{
		// read out user-defined robot configurations
		std::vector<double> temp;
		node_handle_.getParam("robot_configurations", temp);
		const int number_configurations = temp.size()/5;
		if (temp.size()%5 != 0 || temp.size() < 3*5)
		{
			ROS_ERROR("The robot_configurations vector should contain at least 3 configurations with 5 values each.");
			return;
		}
		std::cout << "Robot configurations:\n";
		for (int i=0; i<number_configurations; ++i)
		{
			robot_configurations_.push_back(calibration_utilities::RobotConfiguration(temp[5*i], temp[5*i+1], temp[5*i+2], temp[5*i+3], temp[5*i+4]));
			std::cout << temp[5*i] << "\t" << temp[5*i+1] << "\t" << temp[5*i+2] << "\t" << temp[5*i+3] << "\t" << temp[5*i+4] << std::endl;
		}
	}

	// topics
	tilt_controller_ = node_handle_.advertise<std_msgs::Float64>(tilt_controller_command_, 1, false);
	pan_controller_ = node_handle_.advertise<std_msgs::Float64>(pan_controller_command_, 1, false);
	base_controller_ = node_handle_.advertise<geometry_msgs::Twist>(base_controller_topic_name_, 1, false);

	std::cout << "CameraBaseCalibrationMarker: init done." << std::endl;
}

CameraBaseCalibrationMarker::~CameraBaseCalibrationMarker()
{
	if ( calibration_interface_ != 0 )
		delete calibration_interface_;
	//if (pan_tilt_joint_state_current_!=0)
		//delete pan_tilt_joint_state_current_;
	//if (pan_joint_state_current_!=0)
	//	delete pan_joint_state_current_;
	//if (tilt_joint_state_current_!=0)
	//	delete tilt_joint_state_current_;
}

// old style controller
/*void CameraBaseCalibrationMarker::panTiltJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	ROS_INFO("Old style controller state received.");
	boost::mutex::scoped_lock lock(pan_tilt_joint_state_data_mutex_);
	pan_joint_state_current_ = new double;
	*pan_joint_state_current_ = msg->position[0];
	tilt_joint_state_current_ = new double;
	*tilt_joint_state_current_ = msg->position[1];
}*/

// new controller
/*void CameraBaseCalibrationMarker::panJointStateCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(pan_tilt_joint_state_data_mutex_);
	pan_joint_state_current_ = new double;
	*pan_joint_state_current_ = msg->current_pos;
}

void CameraBaseCalibrationMarker::tiltJointStateCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(pan_tilt_joint_state_data_mutex_);
	tilt_joint_state_current_ = new double;
	*tilt_joint_state_current_ = msg->current_pos;
}*/

bool CameraBaseCalibrationMarker::moveRobot(const calibration_utilities::RobotConfiguration& robot_configuration)
{
	const double k_base = 0.25;
	const double k_phi = 0.25;
	
	// move pan-tilt unit
	std_msgs::Float64 msg;
	msg.data = robot_configuration.pan_angle_;
	pan_controller_.publish(msg);
	msg.data = robot_configuration.tilt_angle_;
	tilt_controller_.publish(msg);
	
	// do not move if close to goal
	double error_phi = 10;
	double error_x = 10;
	double error_y = 10;
	cv::Mat T;
	if (!transform_utilities::getTransform(transform_listener_, "landmark_reference_nav", base_frame_, T))
		return false;
	cv::Vec3d ypr = transform_utilities::YPRFromRotationMatrix(T);
	double robot_yaw = ypr.val[0];
	geometry_msgs::Twist tw;
	error_phi = robot_configuration.pose_phi_ - robot_yaw;
	while (error_phi < -CV_PI*0.5)
		error_phi += CV_PI;
	while (error_phi > CV_PI*0.5)
		error_phi -= CV_PI;
	error_x = robot_configuration.pose_x_ - T.at<double>(0,3);
	error_y = robot_configuration.pose_y_ - T.at<double>(1,3);

	//std::cout << "Before control: error_x=" << error_x << "   error_y=" << error_y << "   error_phi=" << error_phi << std::endl;
	if (fabs(error_phi) > 0.03 || fabs(error_x) > 0.02 || fabs(error_y) > 0.02)
	{
		// control robot angle
		while(true)
		{
			if (!transform_utilities::getTransform(transform_listener_, "landmark_reference_nav", base_frame_, T))
				return false;
			cv::Vec3d ypr = transform_utilities::YPRFromRotationMatrix(T);
				double robot_yaw = ypr.val[0];
			geometry_msgs::Twist tw;
			error_phi = robot_configuration.pose_phi_ - robot_yaw;
			while (error_phi < -CV_PI*0.5)
				error_phi += CV_PI;
			while (error_phi > CV_PI*0.5)
				error_phi -= CV_PI;
			if (fabs(error_phi) < 0.02 || !ros::ok())
				break;
			tw.angular.z = std::min(0.05, k_phi*error_phi);
			base_controller_.publish(tw);
			ros::Rate(20).sleep();
		}

		// control position
		while(true)
		{
			if (!transform_utilities::getTransform(transform_listener_, "landmark_reference_nav", base_frame_, T))
				return false;
			geometry_msgs::Twist tw;
			error_x = robot_configuration.pose_x_ - T.at<double>(0,3);
			error_y = robot_configuration.pose_y_ - T.at<double>(1,3);
			if ((fabs(error_x) < 0.01 && fabs(error_y) < 0.01) || !ros::ok())
				break;
//			std::cout << "error_x: " << error_x << std::endl;
//			std::cout << "error_y: " << error_y << std::endl;
			tw.linear.x = std::min(0.05, k_base*error_x);
			tw.linear.y = std::min(0.05, k_base*error_y);
			base_controller_.publish(tw);
			ros::Rate(20).sleep();
		}

		// control robot angle
		while (true)
		{
			if (!transform_utilities::getTransform(transform_listener_, "landmark_reference_nav", base_frame_, T))
				return false;
			cv::Vec3d ypr = transform_utilities::YPRFromRotationMatrix(T);
				double robot_yaw = ypr.val[0];
			geometry_msgs::Twist tw;
			error_phi = robot_configuration.pose_phi_ - robot_yaw;
			while (error_phi < -CV_PI*0.5)
				error_phi += CV_PI;
			while (error_phi > CV_PI*0.5)
				error_phi -= CV_PI;
			if (fabs(error_phi) < 0.02 || !ros::ok())
				break;
			tw.angular.z = std::min(0.05, k_phi*error_phi);
			base_controller_.publish(tw);
			ros::Rate(20).sleep();
		}

		// turn off robot motion
		geometry_msgs::Twist tw;
		tw.linear.x = 0;
		tw.linear.y = 0;
		tw.angular.z = 0;
		base_controller_.publish(tw);
	}
	
	// wait for pan tilt to arrive at goal position
	double pan_joint_state_current = calibration_interface_->getCurrentCameraPanAngle();
	double tilt_joint_state_current = calibration_interface_->getCurrentCameraTiltAngle();
	if (pan_joint_state_current!=0 && tilt_joint_state_current!=0)
	{
		Timer timeout;
		while (timeout.getElapsedTimeInSec()<5.0)
		{
			boost::mutex::scoped_lock(pan_tilt_joint_state_data_mutex_);
			if (fabs(pan_joint_state_current-robot_configuration.pan_angle_)<0.01 && fabs(tilt_joint_state_current-robot_configuration.tilt_angle_)<0.01)
				break;
			ros::spinOnce();
		}
	}
	else
	{
		ros::Duration(1).sleep();
	}

	/*std::cout << "After control: error_x=" << error_x << "   error_y=" << error_y << "   error_phi=" << error_phi << std::endl;
	std::cout << "Positioning successful: x=" << robot_configuration.pose_x_ << ", y=" << robot_configuration.pose_y_
			<< ", phi=" << robot_configuration.pose_phi_ << ", pan=" << robot_configuration.pan_angle_
			<< ", tilt=" << robot_configuration.tilt_angle_
			<< "\n############################################################################### "
			<< counter++ << ". " << elapsed_time_since_start_.getElapsedTimeInSec() << "s"
			<< std::endl;*/
			
	ros::spinOnce();
	//ros::Duration(1).sleep();

	return true;
}

void CameraBaseCalibrationMarker::extrinsicCalibrationBaseToTorsoLower(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
		std::vector<cv::Mat>& T_base_to_marker_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
		std::vector<cv::Mat>& T_camera_to_marker_vector)
{
	// transform 3d marker points to respective coordinates systems (camera and torso_upper)
	std::vector<cv::Point3d> points_3d_torso_upper, points_3d_camera;
	for (size_t i=0; i<pattern_points_3d.size(); ++i)
	{
		cv::Mat T_torso_upper_to_marker = T_torso_lower_to_torso_upper_vector[i].inv() * T_base_to_torso_lower_.inv() * T_base_to_marker_vector[i];
//		std::cout << "T_camera_to_marker_vector[i]:\n" << T_camera_to_marker_vector[i] << std::endl;
//		std::cout << "T_torso_upper_to_marker:\n" << T_torso_upper_to_marker << std::endl;
		for (size_t j=0; j<pattern_points_3d[i].size(); ++j)
		{
			cv::Mat point = cv::Mat(cv::Vec4d(pattern_points_3d[i][j].x, pattern_points_3d[i][j].y, pattern_points_3d[i][j].z, 1.0));

			// to camera coordinate system
			cv::Mat point_camera = T_camera_to_marker_vector[i] * point;
			//std::cout << "point_camera=" << point_camera << std::endl;
			points_3d_camera.push_back(cv::Point3d(point_camera.at<double>(0), point_camera.at<double>(1), point_camera.at<double>(2)));

			// to torso_upper coordinate
			cv::Mat point_torso_upper = T_torso_upper_to_marker * point;
			//std::cout << "point_torso_upper=" << point_torso_upper << std::endl;
			points_3d_torso_upper.push_back(cv::Point3d(point_torso_upper.at<double>(0), point_torso_upper.at<double>(1), point_torso_upper.at<double>(2)));
		}
	}

	T_torso_upper_to_camera_ = transform_utilities::computeExtrinsicTransform(points_3d_torso_upper, points_3d_camera);
}

void CameraBaseCalibrationMarker::extrinsicCalibrationTorsoUpperToCamera(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
		std::vector<cv::Mat>& T_base_to_marker_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
		std::vector<cv::Mat>& T_camera_to_marker_vector)
{
	// transform 3d chessboard points to respective coordinates systems (base and torso_lower)
	std::vector<cv::Point3d> points_3d_base, points_3d_torso_lower;
	for (size_t i=0; i<pattern_points_3d.size(); ++i)
	{
		cv::Mat T_torso_lower_to_marker = T_torso_lower_to_torso_upper_vector[i] * T_torso_upper_to_camera_ * T_camera_to_marker_vector[i];
//		std::cout << "T_base_to_marker_vector[" << i << "]:\n" << T_base_to_marker_vector[i] << std::endl;
//		std::cout << "T_torso_lower_to_marker:\n" << T_torso_lower_to_marker << std::endl;
//		std::cout << "T_torso_lower_to_torso_upper_vector[i]:\n" << T_torso_lower_to_torso_upper_vector[i] << std::endl;
//		std::cout << "T_torso_upper_to_camera_:\n" << T_torso_upper_to_camera_ << std::endl;
//		std::cout << "T_camera_to_marker_vector[i]:\n" << T_camera_to_marker_vector[i] << std::endl;
		for (size_t j=0; j<pattern_points_3d[i].size(); ++j)
		{
			cv::Mat point = cv::Mat(cv::Vec4d(pattern_points_3d[i][j].x, pattern_points_3d[i][j].y, pattern_points_3d[i][j].z, 1.0));

			// to camera coordinate system
			cv::Mat point_base = T_base_to_marker_vector[i] * point;
			//std::cout << "point_base: " << pattern_points_3d[i][j].x <<", "<< pattern_points_3d[i][j].y <<", "<< pattern_points_3d[i][j].z << " --> " << point_base.at<double>(0,0) <<", "<< point_base.at<double>(1,0) << ", " << point_base.at<double>(2,0) << std::endl;
			points_3d_base.push_back(cv::Point3d(point_base.at<double>(0), point_base.at<double>(1), point_base.at<double>(2)));

			// to torso_upper coordinate
			cv::Mat point_torso_lower = T_torso_lower_to_marker * point;
			//std::cout << "point_torso_lower: " << pattern_points_3d[i][j].x <<", "<< pattern_points_3d[i][j].y <<", "<< pattern_points_3d[i][j].z << " --> " << point_torso_lower.at<double>(0) <<", "<< point_torso_lower.at<double>(1) << ", " << point_torso_lower.at<double>(2) << std::endl;
			points_3d_torso_lower.push_back(cv::Point3d(point_torso_lower.at<double>(0), point_torso_lower.at<double>(1), point_torso_lower.at<double>(2)));
		}
	}

	T_base_to_torso_lower_ = transform_utilities::computeExtrinsicTransform(points_3d_base, points_3d_torso_lower);
}

void CameraBaseCalibrationMarker::displayAndSaveCalibrationResult(const cv::Mat& T_base_to_torso_lower_, const cv::Mat& T_torso_upper_to_camera_)
{
	// display calibration parameters
	std::stringstream output;
	output << "\n\n\n----- Replace these parameters in your 'squirrel_robotino/robotino_bringup/robots/xyz_robotino/urdf/properties.urdf.xacro' file -----\n\n";
	cv::Vec3d ypr = transform_utilities::YPRFromRotationMatrix(T_base_to_torso_lower_);
	output << "  <!-- pan_tilt mount positions | handeye calibration | relative to base_link -->\n"
			  << "  <property name=\"pan_tilt_x\" value=\"" << T_base_to_torso_lower_.at<double>(0,3) << "\"/>\n"
			  << "  <property name=\"pan_tilt_y\" value=\"" << T_base_to_torso_lower_.at<double>(1,3) << "\"/>\n"
			  << "  <property name=\"pan_tilt_z\" value=\"" << T_base_to_torso_lower_.at<double>(2,3) << "\"/>\n"
			  << "  <property name=\"pan_tilt_roll\" value=\"" << ypr.val[2] << "\"/>\n"
			  << "  <property name=\"pan_tilt_pitch\" value=\"" << ypr.val[1] << "\"/>\n"
			  << "  <property name=\"pan_tilt_yaw\" value=\"" << ypr.val[0] << "\"/>\n\n";
	ypr = transform_utilities::YPRFromRotationMatrix(T_torso_upper_to_camera_);
	output << "  <!-- kinect mount positions | handeye calibration | relative to pan_tilt_link -->\n"
			  << "  <property name=\"kinect_x\" value=\"" << T_torso_upper_to_camera_.at<double>(0,3) << "\"/>\n"
			  << "  <property name=\"kinect_y\" value=\"" << T_torso_upper_to_camera_.at<double>(1,3) << "\"/>\n"
			  << "  <property name=\"kinect_z\" value=\"" << T_torso_upper_to_camera_.at<double>(2,3) << "\"/>\n"
			  << "  <property name=\"kinect_roll\" value=\"" << ypr.val[2] << "\"/>\n"
			  << "  <property name=\"kinect_pitch\" value=\"" << ypr.val[1] << "\"/>\n"
			  << "  <property name=\"kinect_yaw\" value=\"" << ypr.val[0] << "\"/>\n" << std::endl;
	std::cout << output.str();

	std::string path_file = calibration_storage_path_ + "camera_calibration_urdf.txt";
	std::fstream file_output;
	file_output.open(path_file.c_str(), std::ios::out);
	if (file_output.is_open())
		file_output << output.str();
	file_output.close();
}
