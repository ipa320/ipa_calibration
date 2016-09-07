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
			CameraBaseCalibrationMarker(nh)
{
	// load parameters
	std::cout << "\n========== CameraBaseCalibrationPiTag Parameters ==========\n";
	node_handle_.param<std::string>("marker_frame_base_name", marker_frame_base_name_, "marker");
	std::cout << "marker_frame_base_name: " << marker_frame_base_name_ << std::endl;

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

	// extrinsic calibration between base and torso_lower as well as torso_upper and camera
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
	displayAndSaveCalibrationResult(T_base_to_torso_lower_, T_torso_upper_to_camera_);

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

				ROS_INFO("=#=#=#=#=#=#=#=#= Found %s", marker_frame.c_str());
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
