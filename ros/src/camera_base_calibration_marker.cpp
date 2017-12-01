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

//#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <sstream>
#include <fstream>

// ToDo: Remove static camera angle link count of 2
// ToDo: Pan_Range and Tilt_Range needs to be stored in one 3*X vector (X number of camera links and 3: min, step, end)
// ToDo: displayAndSaveCalibrationResult, alter behaviour so that it prints custom strings instead of hardcoded ones. [Done]
// ToDo: Stop robot immediately if reference frame gets lost or jumps around!!!! [Done]
// ToDo: Make that pitag/checkerboard/arm calibration calibration results will be stored to different subfolders
// ToDo: Change convention of rotations from RPY to YPR inside transformations_utilities! Function says YPR already, but it is wrong! [Done, by removing function]

CameraBaseCalibrationMarker::CameraBaseCalibrationMarker(ros::NodeHandle nh) :
			RobotCalibration(nh, false), counter(0), RefHistoryIndex_(0)
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
	node_handle_.param("optimization_iterations", optimization_iterations_, 100);
	std::cout << "optimization_iterations: " << optimization_iterations_ << std::endl;
	node_handle_.param<std::string>("child_frame_name", child_frame_name_, "/landmark_reference_nav");
	std::cout << "child_frame_name: " << child_frame_name_ << std::endl;

	// initial parameters
	bool success = transform_utilities::getTransform(transform_listener_, base_frame_, torso_lower_frame_, T_base_to_torso_lower_);
	if ( success == false )
	{
		ROS_FATAL("Could not retrieve transform from %s to %s from TF!",base_frame_.c_str(),torso_lower_frame_.c_str());
		throw std::exception();
	}
	else
		std::cout << "T_base_to_torso_lower_initial:\n" << T_base_to_torso_lower_ << std::endl;

	success = transform_utilities::getTransform(transform_listener_, torso_upper_frame_, camera_frame_, T_torso_upper_to_camera_);
	if ( success == false )
	{
		ROS_FATAL("Could not retrieve transform from %s to %s from TF!",torso_upper_frame_.c_str(),camera_frame_.c_str());
		throw std::exception();
	}
	else
		std::cout << "T_torso_upper_to_camera_initial:\n" << T_torso_upper_to_camera_ << std::endl;

	/*T_base_to_torso_lower_ = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(0.0, 0.0, 0.0), cv::Mat(cv::Vec3d(0.25, 0, 0.5)));
	T_torso_upper_to_camera_ = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(0.0, 0.0, -1.57), cv::Mat(cv::Vec3d(0.0, 0.065, 0.0)));
	std::vector<double> temp;
	node_handle_.getParam("T_base_to_torso_lower_initial", temp);
	if (temp.size()==6)
		T_base_to_torso_lower_ = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(temp[3], temp[4], temp[5]), cv::Mat(cv::Vec3d(temp[0], temp[1], temp[2])));
	std::cout << "T_base_to_torso_lower_initial:\n" << T_base_to_torso_lower_ << std::endl;
	temp.clear();
	node_handle_.getParam("T_torso_upper_to_camera_initial", temp);
	if (temp.size()==6)
		T_torso_upper_to_camera_ = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(temp[3], temp[4], temp[5]), cv::Mat(cv::Vec3d(temp[0], temp[1], temp[2])));
	std::cout << "T_torso_upper_to_camera_initial:\n" << T_torso_upper_to_camera_ << std::endl;*/

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

	// Check whether relative_localization has initialized the reference frame yet.
	// Do not let the robot start driving when the reference frame has not been set up properly! Bad things could happen!
	Timer timeout;
	bool result = false;

	while ( timeout.getElapsedTimeInSec() < 10.f )
	{
		try
		{
			result = transform_listener_.waitForTransform(base_frame_, child_frame_name_, ros::Time(0), ros::Duration(1.f));
			if (result) // Everything is fine, exit loop
			{
				cv::Mat T;
				transform_utilities::getTransform(transform_listener_, child_frame_name_, base_frame_, T);

				for ( int i=0; i<RefFrameHistorySize; ++i ) // Initialize history array
				{
					RefFrameHistory_[i] = T.at<double>(0,3)*T.at<double>(0,3) + T.at<double>(1,3)*T.at<double>(1,3) + T.at<double>(2,3)*T.at<double>(2,3); // Squared norm is suffice here, no need to take root.
				}
				break;
			}
		}
		catch (tf::TransformException& ex)
		{
			ROS_WARN("%s", ex.what());
			// Continue with loop and try again
		}

		ros::Duration(0.1f).sleep(); //Wait for child_frame transform to register properly
	}

	//Failed to set up child frame, exit
	if ( !result )
	{
		ROS_FATAL("RobotCalibration::RobotCalibration: Reference frame has not been set up for 10 seconds.");
		throw std::exception();
	}

	std::cout << "CameraBaseCalibrationMarker: init done." << std::endl;
}

CameraBaseCalibrationMarker::~CameraBaseCalibrationMarker()
{
}

bool CameraBaseCalibrationMarker::isReferenceFrameValid(cv::Mat &T) // Safety measure, to avoid undetermined motion
{
	if (!transform_utilities::getTransform(transform_listener_, child_frame_name_, base_frame_, T))
		return false;

	double currentSqNorm = T.at<double>(0,3)*T.at<double>(0,3) + T.at<double>(1,3)*T.at<double>(1,3) + T.at<double>(2,3)*T.at<double>(2,3);

	double average = 0.0;
	for ( int i=0; i<RefFrameHistorySize; ++i )
		average += RefFrameHistory_[i];
	average /= RefFrameHistorySize;

	RefFrameHistory_[ RefHistoryIndex_ < RefFrameHistorySize-1 ? RefHistoryIndex_++ : (RefHistoryIndex_ = 0) ] = currentSqNorm; // Update with new measurement

	if ( average == 0.0 || abs(1.0 - (currentSqNorm/average)) > 0.15  ) // Up to 15% deviation to average is allowed.
	{
		ROS_WARN("Reference frame can't be detected reliably. It's current deviation to average to too great.");
		return false;
	}

	return true;
}

bool CameraBaseCalibrationMarker::moveRobot(const calibration_utilities::RobotConfiguration& robot_configuration)
{
	const double k_base = 0.25;
	const double k_phi = 0.25;
	
	//Avoid that robot moves, when there is an error with detecting the wall!

	// move pan-tilt unit
	std_msgs::Float64MultiArray angles;
	
	// to do: make the number of camera angles in robot_configuration variable, i.e. std::vector<double> camera_joints; instead of pan_angle/tilt_angle
	angles.data.resize(2);
	angles.data[0] = robot_configuration.pan_angle_;
	angles.data[1] = robot_configuration.tilt_angle_;

	calibration_interface_->assignNewCameraAngles(angles);

	// do not move if close to goal
	double error_phi = 10;
	double error_x = 10;
	double error_y = 10;

	cv::Mat T;

	if (!isReferenceFrameValid(T))
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
			if (!isReferenceFrameValid(T))
			{
				turnOffBaseMotion();
				return false;
			}

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
			calibration_interface_->assignNewRobotVelocity(tw);
			ros::Rate(20).sleep();
		}

		turnOffBaseMotion();

		// control position
		while(true)
		{
			if (!isReferenceFrameValid(T))
			{
				turnOffBaseMotion();
				return false;
			}

			geometry_msgs::Twist tw;
			error_x = robot_configuration.pose_x_ - T.at<double>(0,3);
			error_y = robot_configuration.pose_y_ - T.at<double>(1,3);
			if ((fabs(error_x) < 0.01 && fabs(error_y) < 0.01) || !ros::ok())
				break;
//			std::cout << "error_x: " << error_x << std::endl;
//			std::cout << "error_y: " << error_y << std::endl;
			tw.linear.x = std::min(0.05, k_base*error_x);
			tw.linear.y = std::min(0.05, k_base*error_y);
			calibration_interface_->assignNewRobotVelocity(tw);
			ros::Rate(20).sleep();
		}

		turnOffBaseMotion();

		// control robot angle
		while (true)
		{
			if (!isReferenceFrameValid(T))
			{
				turnOffBaseMotion();
				return false;
			}

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
			calibration_interface_->assignNewRobotVelocity(tw);
			ros::Rate(20).sleep();
		}

		// turn off robot motion
		turnOffBaseMotion();
	}
	
	// wait for pan tilt to arrive at goal position
	if ( (*calibration_interface_->getCurrentCameraState()).size() > 0 )//calibration_interface_->getCurrentCameraPanAngle()!=0 && calibration_interface_->getCurrentCameraTiltAngle()!=0)
	{
		Timer timeout;
		while (timeout.getElapsedTimeInSec()<5.0)
		{
			boost::mutex::scoped_lock(pan_tilt_joint_state_data_mutex_);
			std::vector<double> cur_state = *calibration_interface_->getCurrentCameraState();
			std::vector<double> difference(cur_state.size());
			for (int i = 0; i<cur_state.size(); ++i)
				difference[i] = angles.data[i]-cur_state[i];

			double length = std::sqrt(std::inner_product(difference.begin(), difference.end(), difference.begin(), 0.0)); //Length of difference vector in joint space

			if ( length < 0.01 ) //Close enough to goal configuration (~0.5° deviation allowed)
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

void CameraBaseCalibrationMarker::turnOffBaseMotion()
{
	geometry_msgs::Twist tw;
	tw.linear.x = 0;
	tw.linear.y = 0;
	tw.angular.z = 0;
	calibration_interface_->assignNewRobotVelocity(tw);
}

void CameraBaseCalibrationMarker::extrinsicCalibrationBaseToTorsoLower(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
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

			// to base coordinate system
			cv::Mat point_base = T_base_to_marker_vector[i] * point;
			//std::cout << "point_base: " << pattern_points_3d[i][j].x <<", "<< pattern_points_3d[i][j].y <<", "<< pattern_points_3d[i][j].z << " --> " << point_base.at<double>(0,0) <<", "<< point_base.at<double>(1,0) << ", " << point_base.at<double>(2,0) << std::endl;
			points_3d_base.push_back(cv::Point3d(point_base.at<double>(0), point_base.at<double>(1), point_base.at<double>(2)));

			// to torso_lower coordinate
			cv::Mat point_torso_lower = T_torso_lower_to_marker * point;
			//std::cout << "point_torso_lower: " << pattern_points_3d[i][j].x <<", "<< pattern_points_3d[i][j].y <<", "<< pattern_points_3d[i][j].z << " --> " << point_torso_lower.at<double>(0) <<", "<< point_torso_lower.at<double>(1) << ", " << point_torso_lower.at<double>(2) << std::endl;
			points_3d_torso_lower.push_back(cv::Point3d(point_torso_lower.at<double>(0), point_torso_lower.at<double>(1), point_torso_lower.at<double>(2)));
		}
	}

	T_base_to_torso_lower_ = transform_utilities::computeExtrinsicTransform(points_3d_base, points_3d_torso_lower);
}

void CameraBaseCalibrationMarker::extrinsicCalibrationTorsoUpperToCamera(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
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

void CameraBaseCalibrationMarker::extrinsicCalibration(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
		std::vector<cv::Mat>& T_base_to_marker_vector, std::vector< std::vector<cv::Mat> > T_between_gaps_vector,
		std::vector<cv::Mat>& T_camera_to_marker_vector, int trafo_to_calibrate)
{
	// transform 3d marker points to respective coordinates systems (camera and torso_upper)
	std::vector<cv::Point3d> points_3d_child, points_3d_parent;
	for (size_t i=0; i<pattern_points_3d.size(); ++i)
	{
		cv::Mat T_child_to_marker;

		// Iterate over uncertain trafos, add in between trafos as well
		// Forwards in chain from child frame on
		for ( int j=trafo_to_calibrate; j<transforms_to_calibrate_.size()-1; ++j )
		{
			if ( transforms_to_calibrate_[j].trafo_until_next_gap_idx_ > -1 )
				T_child_to_marker *= T_between_gaps_vector[i][transforms_to_calibrate_[j].trafo_until_next_gap_idx_];

			T_child_to_marker *= transforms_to_calibrate_[j+1].current_trafo_;
		}
		T_child_to_marker *= T_camera_to_marker_vector[i];

		cv::Mat T_parent_to_marker;
		// Backwards in chain from parent frame on
		for ( int j=trafo_to_calibrate-1; j>=0; --j )
		{
			if ( transforms_to_calibrate_[j].trafo_until_next_gap_idx_ > -1 )
				T_parent_to_marker *= T_between_gaps_vector[i][transforms_to_calibrate_[j].trafo_until_next_gap_idx_].inv();

			T_parent_to_marker *= transforms_to_calibrate_[j].current_trafo_.inv();
		}
		T_parent_to_marker *= T_base_to_marker_vector[i];


		for (size_t j=0; j<pattern_points_3d[i].size(); ++j)
		{
			cv::Mat point = cv::Mat(cv::Vec4d(pattern_points_3d[i][j].x, pattern_points_3d[i][j].y, pattern_points_3d[i][j].z, 1.0));

			// to child coordinate system
			cv::Mat point_child = T_child_to_marker * point;
			//std::cout << "point_camera=" << point_camera << std::endl;
			points_3d_child.push_back(cv::Point3d(point_child.at<double>(0), point_child.at<double>(1), point_child.at<double>(2)));

			// to parent coordinate system
			cv::Mat point_parent = T_parent_to_marker * point;
			//std::cout << "point_torso_upper=" << point_torso_upper << std::endl;
			points_3d_parent.push_back(cv::Point3d(point_parent.at<double>(0), point_parent.at<double>(1), point_parent.at<double>(2)));
		}
	}
}

void CameraBaseCalibrationMarker::displayAndSaveCalibrationResult(const std::vector<cv::Mat>& calibratedTransforms)//const cv::Mat& T_base_to_torso_lower_, const cv::Mat& T_torso_upper_to_camera_)
{
	std::vector<std::string> parameter_names;

	calibration_interface_->getParameterNames(parameter_names);

	int ps = parameter_names.size();
	int cs = calibratedTransforms.size();

	if ( ps < cs )
	{
		ROS_WARN("Parameter name size is smaller than amount of calibrated transforms, inserting default names...");

		for ( int i=0; i<(cs-ps); ++i )
		{
			std::stringstream ss;
			ss << "Param_" << (i+1);
			parameter_names.push_back(ss.str());
		}
	}
	else if ( ps > cs )
		ROS_WARN("Parameter name size is greater than amount of calibrated transforms, skipping those...");

	// display calibration parameters
	std::stringstream output;
	output << "\n\n\n----- Replace these parameters in your 'squirrel_robotino/robotino_bringup/robots/xyz_robotino/urdf/properties.urdf.xacro' file -----\n\n";
	for ( int i=0; i<calibratedTransforms.size(); ++i )
	{
		cv::Mat transform = calibratedTransforms[i];
		cv::Vec3d ypr = transform_utilities::YPRFromRotationMatrix(transform);

			output << "  <!-- " << parameter_names[i] <<" mount positions | camera base calibration | relative to base_link -->\n"
					<< "  <property name=\"" << parameter_names[i] << "_x\" value=\"" << transform.at<double>(0,3) << "\"/>\n"
					<< "  <property name=\"" << parameter_names[i] << "_y\" value=\"" << transform.at<double>(1,3) << "\"/>\n"
					<< "  <property name=\"" << parameter_names[i] << "_z\" value=\"" << transform.at<double>(2,3) << "\"/>\n"
					<< "  <property name=\"" << parameter_names[i] << "_roll\" value=\"" << ypr.val[2] << "\"/>\n"
					<< "  <property name=\"" << parameter_names[i] << "_pitch\" value=\"" << ypr.val[1] << "\"/>\n"
					<< "  <property name=\"" << parameter_names[i] << "_yaw\" value=\"" << ypr.val[0] << "\"/>\n\n";
	}
	/*ypr = transform_utilities::YPRFromRotationMatrix(T_torso_upper_to_camera_);
	output << "  <!-- kinect mount positions | camera base calibration | relative to neck_tilt_link -->\n"
			  << "  <property name=\"kinect_x\" value=\"" << T_torso_upper_to_camera_.at<double>(0,3) << "\"/>\n"
			  << "  <property name=\"kinect_y\" value=\"" << T_torso_upper_to_camera_.at<double>(1,3) << "\"/>\n"
			  << "  <property name=\"kinect_z\" value=\"" << T_torso_upper_to_camera_.at<double>(2,3) << "\"/>\n"
			  << "  <property name=\"kinect_roll\" value=\"" << ypr.val[2] << "\"/>\n"
			  << "  <property name=\"kinect_pitch\" value=\"" << ypr.val[1] << "\"/>\n"
			  << "  <property name=\"kinect_yaw\" value=\"" << ypr.val[0] << "\"/>\n" << std::endl;*/

	std::cout << output.str();

	if ( ros::ok() )
	{
		std::string path_file = calibration_storage_path_ + "camera_calibration_urdf.txt";
		std::fstream file_output;
		file_output.open(path_file.c_str(), std::ios::out);
		if (file_output.is_open())
			file_output << output.str();
		file_output.close();
	}
	else
	{
		ROS_INFO("Skipping to save calibration result.");
	}
}

void CameraBaseCalibrationMarker::displayAndSaveCalibrationResult()
{
	std::stringstream output;

	output << "\n\n\n----- Replace the follwing parameters within the urdf file of your robot ----- \n\n";
	for ( int i=0; i<transforms_to_calibrate_.size(); ++i )
	{
		cv::Vec3d ypr = transform_utilities::YPRFromRotationMatrix(transforms_to_calibrate_[i].current_trafo_);

		output << "<!-- " << transforms_to_calibrate_[i].child_ << " mount positions | camera_base_calibration | relative to " << transforms_to_calibrate_[i].parent_ << "-->\n"
				<< "  <property name=\"" << transforms_to_calibrate_[i].child_ << "_x\" value=\"" << transforms_to_calibrate_[i].current_trafo_.at<double>(0,3) << "\"/>\n"
				<< "  <property name=\"" << transforms_to_calibrate_[i].child_ << "_y\" value=\"" << transforms_to_calibrate_[i].current_trafo_.at<double>(1,3) << "\"/>\n"
				<< "  <property name=\"" << transforms_to_calibrate_[i].child_ << "_z\" value=\"" << transforms_to_calibrate_[i].current_trafo_.at<double>(2,3) << "\"/>\n"
				<< "  <property name=\"" << transforms_to_calibrate_[i].child_ << "_roll\" value=\"" << ypr.val[2] << "\"/>\n"
				<< "  <property name=\"" << transforms_to_calibrate_[i].child_ << "_pitch\" value=\"" << ypr.val[1] << "\"/>\n"
				<< "  <property name=\"" << transforms_to_calibrate_[i].child_ << "_yaw\" value=\"" << ypr.val[0] << "\"/>\n\n";
	}

	std::cout << output.str();

	if ( ros::ok() )
	{
		std::string path_file = calibration_storage_path_ + "camera_calibration_urdf.txt";
		std::fstream file_output;
		file_output.open(path_file.c_str(), std::ios::out);
		if (file_output.is_open())
			file_output << output.str();
		file_output.close();
	}
	else
	{
		ROS_INFO("Skipping to save calibration result.");
	}
}
