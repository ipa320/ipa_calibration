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


CameraBaseCalibrationMarker::CameraBaseCalibrationMarker(ros::NodeHandle nh) :
			node_handle_(nh), transform_listener_(nh), camera_calibration_path_("robotino_calibration/camera_calibration/"),
			tilt_controller_command_("/pan_tilt_controller/tilt_joint_position_controller/command"), pan_controller_command_("/pan_tilt_controller/pan_joint_position_controller/command"),
			calibrated_(false), counter(0), pan_tilt_joint_state_current_(0)
{
	// load parameters
	std::cout << "\n========== CameraBaseCalibration Parameters ==========\n";
	// coordinate frame name parameters
	node_handle_.param<std::string>("torso_lower_frame", torso_lower_frame_, "base_pan_link");
	std::cout << "torso_lower_frame: " << torso_lower_frame_ << std::endl;
	node_handle_.param<std::string>("torso_upper_frame", torso_upper_frame_, "tilt_link");
	std::cout << "torso_upper_frame: " << torso_upper_frame_ << std::endl;
	node_handle_.param<std::string>("camera_frame", camera_frame_, "kinect_link");
	std::cout << "camera_frame: " << camera_frame_ << std::endl;
	node_handle_.param<std::string>("camera_optical_frame", camera_optical_frame_, "kinect_rgb_optical_frame");
	std::cout << "camera_optical_frame: " << camera_optical_frame_ << std::endl;
	node_handle_.param<std::string>("base_frame", base_frame_, "base_link");
	std::cout << "base_frame: " << base_frame_ << std::endl;
	// initial parameters
	T_base_to_torso_lower_ = robotino_calibration::makeTransform(robotino_calibration::rotationMatrixFromYPR(0.0, 0.0, 0.0), cv::Mat(cv::Vec3d(0.25, 0, 0.5)));
	T_torso_upper_to_camera_ = robotino_calibration::makeTransform(robotino_calibration::rotationMatrixFromYPR(0.0, 0.0, -1.57), cv::Mat(cv::Vec3d(0.0, 0.065, 0.0)));
	std::vector<double>temp;
	node_handle_.getParam("T_base_to_torso_lower_initial", temp);
	if (temp.size()==6)
		T_base_to_torso_lower_ = robotino_calibration::makeTransform(robotino_calibration::rotationMatrixFromYPR(temp[3], temp[4], temp[5]), cv::Mat(cv::Vec3d(temp[0], temp[1], temp[2])));
	std::cout << "T_base_to_torso_lower_initial:\n" << T_base_to_torso_lower_ << std::endl;
	temp.clear();
	node_handle_.getParam("T_torso_upper_to_camera_initial", temp);
	if (temp.size()==6)
		T_torso_upper_to_camera_ = robotino_calibration::makeTransform(robotino_calibration::rotationMatrixFromYPR(temp[3], temp[4], temp[5]), cv::Mat(cv::Vec3d(temp[0], temp[1], temp[2])));
	std::cout << "T_torso_upper_to_camera_initial:\n" << T_torso_upper_to_camera_ << std::endl;
	// optimization parameters
	node_handle_.param("optimization_iterations", optimization_iterations_, 100);
	std::cout << "optimization_iterations: " << optimization_iterations_ << std::endl;
	// pan/tilt unit positions and robot base locations relative to marker
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
							robot_configurations_.push_back(RobotConfiguration(x, y, phi, pan, tilt));
	}
	else
	{
		// read out user-defined robot configurations
		temp.clear();
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
			robot_configurations_.push_back(RobotConfiguration(temp[5*i], temp[5*i+1], temp[5*i+2], temp[5*i+3], temp[5*i+4]));
			std::cout << temp[5*i] << "\t" << temp[5*i+1] << "\t" << temp[5*i+2] << "\t" << temp[5*i+3] << "\t" << temp[5*i+4] << std::endl;
		}
	}

	// topics
	pan_tilt_state_ = node_handle_.subscribe<sensor_msgs::JointState>("/pan_tilt_controller/joint_states", 0, &CameraBaseCalibrationMarker::panTiltJointStateCallback, this);
	tilt_controller_ = node_handle_.advertise<std_msgs::Float64>(tilt_controller_command_, 1, false);
	pan_controller_ = node_handle_.advertise<std_msgs::Float64>(pan_controller_command_, 1, false);
	base_controller_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, false);
}

CameraBaseCalibrationMarker::~CameraBaseCalibrationMarker()
{
	if (pan_tilt_joint_state_current_!=0)
		delete pan_tilt_joint_state_current_;
}

void CameraBaseCalibrationMarker::setCalibrationStatus(bool calibrated)
{
		calibrated_ = calibrated;
}

void CameraBaseCalibrationMarker::panTiltJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(pan_tilt_joint_state_data_mutex_);
	pan_tilt_joint_state_current_ = new sensor_msgs::JointState;
	*pan_tilt_joint_state_current_ = *msg;
}

bool CameraBaseCalibrationMarker::moveRobot(const RobotConfiguration& robot_configuration)
{
	const double k_base = 0.25;
	const double k_phi = 0.25;
	
	// move pan-tilt unit
	std_msgs::Float64 msg;
	msg.data = robot_configuration.pan_angle_;
	pan_controller_.publish(msg);
	msg.data = robot_configuration.tilt_angle_;
	tilt_controller_.publish(msg);
	
	if (pan_tilt_joint_state_current_!=0)
	{
		while (true)
		{
			boost::mutex::scoped_lock(pan_tilt_joint_state_data_mutex_);
			if (fabs(pan_tilt_joint_state_current_->position[0]-robot_configuration.pan_angle_)<0.001 && fabs(pan_tilt_joint_state_current_->position[1]-robot_configuration.tilt_angle_)<0.001)
				break;
			ros::spinOnce();
		}
	}
	else
	{
		ros::Duration(1).sleep();
	}
	
	// do not move if close to goal
	double error_phi = 10;
	double error_x = 10;
	double error_y = 10;
	cv::Mat T;
	if (!getTransform("landmark_reference_nav", "base_link", T))
		return false;
	cv::Vec3d ypr = robotino_calibration::YPRFromRotationMatrix(T);
	double robot_yaw = ypr.val[0];
	geometry_msgs::Twist tw;
	error_phi = robot_configuration.pose_phi_ - robot_yaw;
	while (error_phi < -CV_PI*0.5)
		error_phi += CV_PI;
	while (error_phi > CV_PI*0.5)
		error_phi -= CV_PI;
	error_x = robot_configuration.pose_x_ - T.at<double>(0,3);
	error_y = robot_configuration.pose_y_ - T.at<double>(1,3);

	std::cout << "Before control: error_x=" << error_x << "   error_y=" << error_y << "   error_phi=" << error_phi << std::endl;
	if (fabs(error_phi) > 0.03 || fabs(error_x) > 0.02 || fabs(error_y) > 0.02)
	{
		// control robot angle
		while(true)
		{
			if (!getTransform("landmark_reference_nav", "base_link", T))
				return false;
			cv::Vec3d ypr = robotino_calibration::YPRFromRotationMatrix(T);
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
			if (!getTransform("landmark_reference_nav", "base_link", T))
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
			if (!getTransform("landmark_reference_nav", "base_link", T))
				return false;
			cv::Vec3d ypr = robotino_calibration::YPRFromRotationMatrix(T);
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
	}
	
	std::cout << "After control: error_x=" << error_x << "   error_y=" << error_y << "   error_phi=" << error_phi << std::endl;
	std::cout << "Positioning successful: x=" << robot_configuration.pose_x_ << ", y=" << robot_configuration.pose_y_
			<< ", phi=" << robot_configuration.pose_phi_ << ", pan=" << robot_configuration.pan_angle_
			<< ", tilt=" << robot_configuration.tilt_angle_
			<< "\n############################################################################### "
			<< counter++ << ". " << elapsed_time_since_start_.getElapsedTimeInSec() << "s"
			<< std::endl;
			
	ros::spinOnce();

	return true;
}

void CameraBaseCalibrationMarker::extrinsicCalibrationBaseToTorsoLower(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
		std::vector<cv::Mat>& T_base_to_marker_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
		std::vector<cv::Mat>& T_camera_to_marker_vector)
{
	// transform 3d chessboard points to respective coordinates systems (camera and torso_upper)
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

	T_torso_upper_to_camera_ = computeExtrinsicTransform(points_3d_torso_upper, points_3d_camera);
}

void CameraBaseCalibrationMarker::extrinsicCalibrationTorsoUpperToCamera(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
		std::vector<cv::Mat>& T_base_to_marker_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
		std::vector<cv::Mat>& T_camera_to_marker_vector)
{
	// transform 3d chessboard points to respective coordinates systems (base and torso_lower)
	std::vector<cv::Point3d> points_3d_base, points_3d_torso_lower;
	for (size_t i=0; i<pattern_points_3d.size(); ++i)
	{
		cv::Mat T_torso_lower_to_checkerboard = T_torso_lower_to_torso_upper_vector[i] * T_torso_upper_to_camera_ * T_camera_to_marker_vector[i];
//		std::cout << "T_base_to_checkerboard_vector[" << i << "]:\n" << T_base_to_checkerboard_vector[i] << std::endl;
//		std::cout << "T_torso_lower_to_checkerboard:\n" << T_torso_lower_to_checkerboard << std::endl;
//		std::cout << "T_torso_lower_to_torso_upper_vector[i]:\n" << T_torso_lower_to_torso_upper_vector[i] << std::endl;
//		std::cout << "T_torso_upper_to_camera_:\n" << T_torso_upper_to_camera_ << std::endl;
//		std::cout << "T_camera_to_checkerboard_vector[i]:\n" << T_camera_to_checkerboard_vector[i] << std::endl;
		for (size_t j=0; j<pattern_points_3d[i].size(); ++j)
		{
			cv::Mat point = cv::Mat(cv::Vec4d(pattern_points_3d[i][j].x, pattern_points_3d[i][j].y, pattern_points_3d[i][j].z, 1.0));

			// to camera coordinate system
			cv::Mat point_base = T_base_to_marker_vector[i] * point;
			//std::cout << "point_base: " << pattern_points_3d[i][j].x <<", "<< pattern_points_3d[i][j].y <<", "<< pattern_points_3d[i][j].z << " --> " << point_base.at<double>(0,0) <<", "<< point_base.at<double>(1,0) << ", " << point_base.at<double>(2,0) << std::endl;
			points_3d_base.push_back(cv::Point3d(point_base.at<double>(0), point_base.at<double>(1), point_base.at<double>(2)));

			// to torso_upper coordinate
			cv::Mat point_torso_lower = T_torso_lower_to_checkerboard * point;
			//std::cout << "point_torso_lower: " << pattern_points_3d[i][j].x <<", "<< pattern_points_3d[i][j].y <<", "<< pattern_points_3d[i][j].z << " --> " << point_torso_lower.at<double>(0) <<", "<< point_torso_lower.at<double>(1) << ", " << point_torso_lower.at<double>(2) << std::endl;
			points_3d_torso_lower.push_back(cv::Point3d(point_torso_lower.at<double>(0), point_torso_lower.at<double>(1), point_torso_lower.at<double>(2)));
		}
	}

	T_base_to_torso_lower_ = computeExtrinsicTransform(points_3d_base, points_3d_torso_lower);
}

// computes the rigid transform between two sets of corresponding 3d points measured in different coordinate systems
// the resulting 4x4 transformation matrix converts point coordinates from the target system into the source coordinate system
cv::Mat CameraBaseCalibrationMarker::computeExtrinsicTransform(const std::vector<cv::Point3d>& points_3d_source, const std::vector<cv::Point3d>& points_3d_target)
{
	// from: http://nghiaho.com/?page_id=671 : ‘A Method for Registration of 3-D Shapes’, by Besl and McKay, 1992.
	cv::Point3d centroid_source, centroid_target;
	for (size_t i=0; i<points_3d_source.size(); ++i)
	{
		centroid_source += points_3d_source[i];
		centroid_target += points_3d_target[i];
	}
	centroid_source *= 1.0/(double)points_3d_source.size();
	centroid_target *= 1.0/(double)points_3d_target.size();

	// covariance matrix
	cv::Mat M = cv::Mat::zeros(3,3,CV_64FC1);
	for (size_t i=0; i<points_3d_source.size(); ++i)
		M += cv::Mat(points_3d_target[i] - centroid_target)*cv::Mat(points_3d_source[i] - centroid_source).t();

	// SVD on covariance matrix yields rotation
	cv::Mat w, u, vt;
	cv::SVD::compute(M, w, u, vt, cv::SVD::FULL_UV);
	cv::Mat R = vt.t()*u.t();

	// correct reflection matrix cases
	if (cv::determinant(R) < 0)
		for (int r=0; r<3; ++r)
			R.at<double>(r,2) *= -1;

	// translation
	cv::Mat t = -R*cv::Mat(centroid_target) + cv::Mat(centroid_source);

	return robotino_calibration::makeTransform(R, t);
}

// computes the transform from target_frame to source_frame (i.e. transform arrow is pointing from target_frame to source_frame)
bool CameraBaseCalibrationMarker::getTransform(const std::string& target_frame, const std::string& source_frame, cv::Mat& T)
{
	try
	{
		tf::StampedTransform Ts;
		transform_listener_.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
		transform_listener_.lookupTransform(target_frame, source_frame, ros::Time(0), Ts);
		const tf::Matrix3x3& rot = Ts.getBasis();
		const tf::Vector3& trans = Ts.getOrigin();
		cv::Mat rotcv(3,3,CV_64FC1);
		cv::Mat transcv(3,1,CV_64FC1);
		for (int v=0; v<3; ++v)
			for (int u=0; u<3; ++u)
				rotcv.at<double>(v,u) = rot[v].m_floats[u];
		for (int v=0; v<3; ++v)
			transcv.at<double>(v) = trans.m_floats[v];
		T = robotino_calibration::makeTransform(rotcv, transcv);
		//std::cout << "Transform from " << source_frame << " to " << target_frame << ":\n" << T << std::endl;
	}
	catch (tf::TransformException& ex)
	{
		ROS_WARN("%s",ex.what());
		return false;
	}

	return true;
}
