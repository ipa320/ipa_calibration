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
// ToDo: Make that pitag/checkerboard/arm calibration calibration results will be stored to different subfolders [Done, as already possible]
// ToDo: Change convention of rotations from RPY to YPR inside transformations_utilities! Function says YPR already, but it is wrong! [Done, by removing function]
// ToDo: Port over more flexible calibration code to checkerboard calibration as well. [Done]
// ToDo: Remove unused attributes [Done]
// ToDo: Move optimization_iterations to robot_calibration mother class and set its value to one if there is only one transform to be calibrated [Done]
// ToDo: Add timer in moverobot/movearm and check if robots setup has changed since last time (maybe 1 sec), if not give a warning.
// ToDo: Cleanup yaml files [Done]
// ToDo: Port flexible calibration code over to arm calibration as well.
// ToDo: TF seems to use RPY convention instead of YPR. transform_utilities::rotationMatrixFromYPR is therefore wrong.
// ToDo: Split up yaml files into a calibrtion yaml and an inferface yaml

CameraBaseCalibrationMarker::CameraBaseCalibrationMarker(ros::NodeHandle nh) :
			RobotCalibration(nh, false), RefHistoryIndex_(0)
{
	// load parameters
	std::cout << "\n========== CameraBaseCalibrationMarker Parameters ==========\n";

	// coordinate frame name parameters
	node_handle_.param<std::string>("camera_optical_frame", camera_optical_frame_, "");
	std::cout << "camera_optical_frame: " << camera_optical_frame_ << std::endl;
	node_handle_.param<std::string>("child_frame_name", child_frame_name_, "/landmark_reference_nav");
	std::cout << "child_frame_name: " << child_frame_name_ << std::endl;

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

	// Debug how RVIZ rotations are defined
	/*cv::Mat T;

	transform_utilities::getTransform(transform_listener_, "arm_link5", base_frame_, T);
	std::cout << "TF: " << T << std::endl;
	std::cout << transform_utilities::YPRFromRotationMatrix( (cv::Mat_<double>(3,3) << T.at<double>(0,0), T.at<double>(0,1), T.at<double>(0,2),
				T.at<double>(1,0), T.at<double>(1,1), T.at<double>(1,2),
				T.at<double>(2,0), T.at<double>(2,1), T.at<double>(2,2)) ) << std::endl;
	std::vector<float> temp;
	node_handle_.getParam("T_initial", temp);
	T = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(temp[3], temp[4], temp[5]), cv::Mat(cv::Vec3d(temp[0], temp[1], temp[2])));
	std::cout << "BUILT: " << T << std::endl;
	std::cout << transform_utilities::YPRFromRotationMatrix( (cv::Mat_<double>(3,3) << T.at<double>(0,0), T.at<double>(0,1), T.at<double>(0,2),
				T.at<double>(1,0), T.at<double>(1,1), T.at<double>(1,2),
				T.at<double>(2,0), T.at<double>(2,1), T.at<double>(2,2)) ) << std::endl;*/

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
			
	ros::spinOnce();
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

void CameraBaseCalibrationMarker::extrinsicCalibration(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
		std::vector<cv::Mat>& T_gapfirst_to_marker_vector, std::vector< std::vector<cv::Mat> > T_between_gaps_vector,
		std::vector<cv::Mat>& T_gaplast_to_marker_vector, int trafo_to_calibrate)
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
		T_child_to_marker *= T_gaplast_to_marker_vector[i];

		cv::Mat T_parent_to_marker;
		// Backwards in chain from parent frame on
		for ( int j=trafo_to_calibrate-1; j>=0; --j )
		{
			if ( transforms_to_calibrate_[j].trafo_until_next_gap_idx_ > -1 )
				T_parent_to_marker *= T_between_gaps_vector[i][transforms_to_calibrate_[j].trafo_until_next_gap_idx_].inv();

			T_parent_to_marker *= transforms_to_calibrate_[j].current_trafo_.inv();
		}
		T_parent_to_marker *= T_gapfirst_to_marker_vector[i];


		for (size_t j=0; j<pattern_points_3d[i].size(); ++j)
		{
			cv::Mat point = cv::Mat(cv::Vec4d(pattern_points_3d[i][j].x, pattern_points_3d[i][j].y, pattern_points_3d[i][j].z, 1.0));

			// to child coordinate system
			cv::Mat point_child = T_child_to_marker * point;
			points_3d_child.push_back(cv::Point3d(point_child.at<double>(0), point_child.at<double>(1), point_child.at<double>(2)));

			// to parent coordinate system
			cv::Mat point_parent = T_parent_to_marker * point;
			points_3d_parent.push_back(cv::Point3d(point_parent.at<double>(0), point_parent.at<double>(1), point_parent.at<double>(2)));
		}
	}
}

bool CameraBaseCalibrationMarker::calculateTransformationChains(cv::Mat& T_gapfirst_to_marker, std::vector<cv::Mat> T_between_gaps,
		cv::Mat& T_gaplast_to_camera_optical, std::string marker_frame)
{
	bool result = true;
	result &= transform_utilities::getTransform(transform_listener_, transforms_to_calibrate_[0].parent_, marker_frame, T_gapfirst_to_marker);
	result &= transform_utilities::getTransform(transform_listener_, transforms_to_calibrate_[ transforms_to_calibrate_.size()-1 ].child_, camera_optical_frame_, T_gaplast_to_camera_optical);

	for ( int i=0; i<transforms_to_calibrate_.size()-1; ++i )
	{
		if ( transforms_to_calibrate_[i].parent_ == transforms_to_calibrate_[i].child_ ) // several gaps in a row, no certain trafos in between
			continue;

		cv::Mat temp;
		result &= transform_utilities::getTransform(transform_listener_, transforms_to_calibrate_[i].child_, transforms_to_calibrate_[i+1].parent_, temp);
		T_between_gaps.push_back(temp);
		transforms_to_calibrate_[i].trafo_until_next_gap_idx_ = T_between_gaps.size()-1;
	}

	return result;
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
