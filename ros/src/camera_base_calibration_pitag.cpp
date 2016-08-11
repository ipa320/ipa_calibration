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
 * Author: Richard Bormann, email:richard.bormann@ipa.fhg.de
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

#include <robotino_calibration/camera_base_calibration_pitag.h>
#include <robotino_calibration/transformation_utilities.h>

#include <std_msgs/Float64.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <sstream>

#include <cob_object_detection_msgs/DetectObjects.h>


CameraBaseCalibrationPiTag::CameraBaseCalibrationPiTag(ros::NodeHandle nh) :
			node_handle_(nh), transform_listener_(nh), camera_calibration_path_("robotino_calibration/camera_calibration/"),
			tilt_controller_command_("/tilt_controller/command"), pan_controller_command_("/pan_controller/command")
{
	// load parameters
	std::cout << "\n========== CameraBaseCalibrationPiTag Parameters ==========\n";
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
	node_handle_.param<std::string>("marker_frame_base_name", marker_frame_base_name_, "marker");
	std::cout << "marker_frame_base_name: " << marker_frame_base_name_ << std::endl;
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
	tilt_controller_ = node_handle_.advertise<std_msgs::Float64>(tilt_controller_command_, 1, false);
	pan_controller_ = node_handle_.advertise<std_msgs::Float64>(pan_controller_command_, 1, false);
	base_controller_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, false);

	pitag_client_ = node_handle_.serviceClient<cob_object_detection_msgs::DetectObjects>("get_fiducials");

	ROS_INFO("CameraBaseCalibration initialized.");
}

CameraBaseCalibrationPiTag::~CameraBaseCalibrationPiTag()
{
}


bool CameraBaseCalibrationPiTag::calibrateCameraToBase(const bool load_data)
{
	// setup storage folder
	int return_value = system("mkdir -p robotino_calibration/camera_calibration");

	// acquire images
	std::vector<cv::Mat> T_base_to_marker_vector;
	std::vector<cv::Mat> T_torso_lower_to_torso_upper_vector;
	std::vector<cv::Mat> T_camera_to_marker_vector;
	acquireCalibrationData(robot_configurations_, load_data,
			T_base_to_marker_vector, T_torso_lower_to_torso_upper_vector, T_camera_to_marker_vector);

	// prepare marker 3d points (actually only the point (0,0,0) in the marker coordinate system
	std::vector< std::vector<cv::Point3f> > pattern_points_3d(T_base_to_marker_vector.size(), std::vector<cv::Point3f>(1, cv::Point3f(0.f, 0.f, 0.f)));

	// extrinsic calibration between base and torso_lower as well ass torso_upper and camera
	for (int i=0; i<optimization_iterations_; ++i)
	{
//		std::cout << "\nExtrinsic optimization run " << i << ":" << std::endl;
		extrinsicCalibrationBaseToTorsoLower(pattern_points_3d, T_base_to_marker_vector, T_torso_lower_to_torso_upper_vector, T_camera_to_marker_vector);
//		cv::Vec3d ypr = YPRFromRotationMatrix(T_base_to_torso_lower_);
//		std::cout << "T_base_to_torso_lower:\n" << T_base_to_torso_lower_ << std::endl;
//		std::cout << "yaw=" << ypr.val[0] << "   pitch=" << ypr.val[1] << "   roll=" << ypr.val[2] << std::endl;
		extrinsicCalibrationTorsoUpperToCamera(pattern_points_3d, T_base_to_marker_vector, T_torso_lower_to_torso_upper_vector, T_camera_to_marker_vector);
//		ypr = YPRFromRotationMatrix(T_torso_upper_to_camera_);
//		std::cout << "T_torso_upper_to_camera:\n" << T_torso_upper_to_camera_ << std::endl;
//		std::cout << "yaw=" << ypr.val[0] << "   pitch=" << ypr.val[1] << "   roll=" << ypr.val[2] << std::endl;
	}

	// display calibration parameters
	std::cout << "\n\n\n----- Replace these parameters in your 'squirrel_robotino/robotino_bringup/robots/xyz_robotino/urdf/properties.urdf.xacro' file -----\n\n";
	cv::Vec3d ypr = robotino_calibration::YPRFromRotationMatrix(T_base_to_torso_lower_);
	std::cout << "  <!-- pan_tilt mount positions | handeye calibration | relative to base_link -->\n"
			  << "  <property name=\"pan_tilt_x\" value=\"" << T_base_to_torso_lower_.at<double>(0,3) << "\"/>\n"
			  << "  <property name=\"pan_tilt_y\" value=\"" << T_base_to_torso_lower_.at<double>(1,3) << "\"/>\n"
			  << "  <property name=\"pan_tilt_z\" value=\"" << T_base_to_torso_lower_.at<double>(2,3) << "\"/>\n"
			  << "  <property name=\"pan_tilt_roll\" value=\"" << ypr.val[2] << "\"/>\n"
			  << "  <property name=\"pan_tilt_pitch\" value=\"" << ypr.val[1] << "\"/>\n"
			  << "  <property name=\"pan_tilt_yaw\" value=\"" << ypr.val[0] << "\"/>\n\n";
	ypr = robotino_calibration::YPRFromRotationMatrix(T_torso_upper_to_camera_);
	std::cout << "  <!-- kinect mount positions | handeye calibration | relative to pan_tilt_link -->\n"
			  << "  <property name=\"kinect_x\" value=\"" << T_torso_upper_to_camera_.at<double>(0,3) << "\"/>\n"
			  << "  <property name=\"kinect_y\" value=\"" << T_torso_upper_to_camera_.at<double>(1,3) << "\"/>\n"
			  << "  <property name=\"kinect_z\" value=\"" << T_torso_upper_to_camera_.at<double>(2,3) << "\"/>\n"
			  << "  <property name=\"kinect_roll\" value=\"" << ypr.val[2] << "\"/>\n"
			  << "  <property name=\"kinect_pitch\" value=\"" << ypr.val[1] << "\"/>\n"
			  << "  <property name=\"kinect_yaw\" value=\"" << ypr.val[0] << "\"/>\n" << std::endl;

	// save calibration
	saveCalibration();
	calibrated_ = true;

	return true;
}

bool CameraBaseCalibrationPiTag::acquireCalibrationData(const std::vector<RobotConfiguration>& robot_configurations,
		const bool load_data, std::vector<cv::Mat>& T_base_to_marker_vector,
		std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector, std::vector<cv::Mat>& T_camera_to_marker_vector)
{
	std::stringstream path;
	path << camera_calibration_path_ << "pitag_data.yml";

	// capture images from different perspectives
	if (load_data == false)
	{
		const int number_images_to_capture = (int)robot_configurations.size();
		for (int image_counter = 0; image_counter < number_images_to_capture; ++image_counter)
		{
			moveRobot(robot_configurations[image_counter]);

			// extract marker points
			cob_object_detection_msgs::DetectObjects detect;
			pitag_client_.call(detect);
			if (detect.response.object_list.detections.size() == 0)
				continue;

			for (size_t detection=0; detection<detect.response.object_list.detections.size(); ++detection)
			{
				cob_object_detection_msgs::Detection& det = detect.response.object_list.detections[detection];
				std::string marker_frame = marker_frame_base_name_ + det.label.substr(3);	// yields e.g. "marker_18"

				// retrieve transformations
				cv::Mat T_base_to_marker, T_torso_lower_to_torso_upper, T_camera_to_camera_optical, T_camera_optical_to_marker, T_camera_to_marker;
				bool result = true;
				result &= getTransform(base_frame_, marker_frame, T_base_to_marker);
				result &= getTransform(torso_lower_frame_, torso_upper_frame_, T_torso_lower_to_torso_upper);
				result &= getTransform(camera_frame_, camera_optical_frame_, T_camera_to_camera_optical);
				if (result == false)
					continue;
				tf::Stamped<tf::Pose> pose;
				tf::poseStampedMsgToTF(det.pose, pose);
				const tf::Matrix3x3& rot = pose.getBasis();
				const tf::Vector3& trans = pose.getOrigin();
				cv::Mat rotcv(3,3,CV_64FC1);
				cv::Mat transcv(3,1,CV_64FC1);
				for (int v=0; v<3; ++v)
					for (int u=0; u<3; ++u)
						rotcv.at<double>(v,u) = rot[v].m_floats[u];
				for (int v=0; v<3; ++v)
					transcv.at<double>(v) = trans.m_floats[v];
				T_camera_optical_to_marker = robotino_calibration::makeTransform(rotcv, transcv);
				T_camera_to_marker = T_camera_to_camera_optical*T_camera_optical_to_marker;

				// attach data to array
				T_base_to_marker_vector.push_back(T_base_to_marker);
				T_torso_lower_to_torso_upper_vector.push_back(T_torso_lower_to_torso_upper);
				T_camera_to_marker_vector.push_back(T_camera_to_marker);
			}
		}

		// save transforms to file
		cv::FileStorage fs(path.str().c_str(), cv::FileStorage::WRITE);
		if (fs.isOpened())
		{
			fs << "T_base_to_marker_vector" << T_base_to_marker_vector;
			fs << "T_torso_lower_to_torso_upper_vector" << T_torso_lower_to_torso_upper_vector;
			fs << "T_camera_to_marker_vector" << T_camera_to_marker_vector;
		}
		else
		{
			ROS_WARN("Could not write transformations to file '%s'.", path.str().c_str());
		}
		fs.release();
	}
	else
	{
		cv::FileStorage fs(path.str().c_str(), cv::FileStorage::READ);
		if (fs.isOpened())
		{
			fs["T_base_to_marker_vector"] >> T_base_to_marker_vector;
			fs["T_torso_lower_to_torso_upper_vector"] >> T_torso_lower_to_torso_upper_vector;
			fs["T_camera_to_marker_vector"] >> T_camera_to_marker_vector;
		}
		else
		{
			ROS_WARN("Could not read transformations from file '%s'.", path.str().c_str());
		}
		fs.release();
	}

	std::cout << "Captured markers: " << T_camera_to_marker_vector.size() << std::endl;
	return true;
}


bool CameraBaseCalibrationPiTag::moveRobot(const RobotConfiguration& robot_configuration)
{
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

	std::cout << "error_x=" << error_x << "   error_y=" << error_y << "   error_phi=" << error_phi << std::endl;
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
			tw.angular.z = std::min(0.05, error_phi);
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
			tw.linear.x = std::min(0.05, error_x);
			tw.linear.y = std::min(0.05, error_y);
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
			tw.angular.z = std::min(0.05, error_phi);
			base_controller_.publish(tw);
			ros::Rate(20).sleep();
		}
	}

	std_msgs::Float64 msg;
	msg.data = robot_configuration.pan_angle_;
	pan_controller_.publish(msg);
	msg.data = robot_configuration.tilt_angle_;
	tilt_controller_.publish(msg);

	ros::Duration(3).sleep();

	std::cout << "Positioning successful: x=" << robot_configuration.pose_x_ << ", y=" << robot_configuration.pose_y_
			<< ", phi=" << robot_configuration.pose_phi_ << ", pan=" << robot_configuration.pan_angle_
			<< ", tilt=" << robot_configuration.tilt_angle_ << std::endl;

	return true;
}


void CameraBaseCalibrationPiTag::extrinsicCalibrationTorsoUpperToCamera(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
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

void CameraBaseCalibrationPiTag::extrinsicCalibrationBaseToTorsoLower(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
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

	T_base_to_torso_lower_ = computeExtrinsicTransform(points_3d_base, points_3d_torso_lower);
}

cv::Mat CameraBaseCalibrationPiTag::computeExtrinsicTransform(const std::vector<cv::Point3d>& points_3d_source, const std::vector<cv::Point3d>& points_3d_target)
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

bool CameraBaseCalibrationPiTag::getTransform(const std::string& target_frame, const std::string& source_frame, cv::Mat& T)
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

bool CameraBaseCalibrationPiTag::saveCalibration()
{
	bool success = true;

	// save calibration
	std::string filename = camera_calibration_path_ + "camera_calibration.yml";
	cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
	if (fs.isOpened() == true)
	{
		fs << "T_base_to_torso_lower" << T_base_to_torso_lower_;
		fs << "T_torso_upper_to_camera" << T_torso_upper_to_camera_;
	}
	else
	{
		std::cout << "Error: CameraBaseCalibration::saveCalibration: Could not write calibration to file.";
		success = false;
	}
	fs.release();

	return success;
}

bool CameraBaseCalibrationPiTag::loadCalibration()
{
	bool success = true;

	// load calibration
	std::string filename = camera_calibration_path_ + "camera_calibration.yml";
	cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
	if (fs.isOpened() == true)
	{
		fs["T_base_to_torso_lower"] >> T_base_to_torso_lower_;
		fs["T_torso_upper_to_camera"] >> T_torso_upper_to_camera_;
	}
	else
	{
		std::cout << "Error: CameraBaseCalibration::loadCalibration: Could not read calibration from file.";
		success = false;
	}
	fs.release();

	calibrated_ = true;

	return success;
}

void CameraBaseCalibrationPiTag::getCalibration(cv::Mat& K, cv::Mat& distortion, cv::Mat& T_base_to_torso_lower, cv::Mat& T_torso_upper_to_camera)
{
	if (calibrated_ == false && loadCalibration() == false)
	{
		std::cout << "Error: CameraBaseCalibration not calibrated and no calibration data available on disk." << std::endl;
		return;
	}

	T_base_to_torso_lower = T_base_to_torso_lower_.clone();
	T_torso_upper_to_camera = T_torso_upper_to_camera_.clone();
}
