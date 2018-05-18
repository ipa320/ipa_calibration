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

#include <sstream>
#include <cob_object_detection_msgs/DetectObjects.h>


CameraBaseCalibrationPiTag::CameraBaseCalibrationPiTag(ros::NodeHandle nh, CalibrationInterface* interface) :
	CameraBaseCalibrationMarker(nh, interface)
{
	// load parameters
	std::cout << "\n========== CameraBaseCalibrationPiTag Parameters ==========\n";
	node_handle_.param<std::string>("marker_frame_base_name", marker_frame_base_name_, "");
	std::cout << "marker_frame_base_name: " << marker_frame_base_name_ << std::endl;
	node_handle_.param<std::string>("get_fiducials_topic", get_fiducials_topic_, "");
	std::cout << "get_fiducials_topic: " << get_fiducials_topic_ << std::endl;

	pitag_client_ = node_handle_.serviceClient<cob_object_detection_msgs::DetectObjects>(get_fiducials_topic_);
	ROS_INFO("CameraBaseCalibrationPiTag initialized.");
}

CameraBaseCalibrationPiTag::~CameraBaseCalibrationPiTag()
{
}

bool CameraBaseCalibrationPiTag::calibrateCameraToBase(const bool load_data)
{
	// acquire images
	std::vector<cv::Mat> T_gapfirst_to_marker_vector;
	std::vector< std::vector<cv::Mat> > T_between_gaps_vector;
	std::vector<cv::Mat> T_gaplast_to_marker_vector;
	acquireCalibrationData(load_data, T_gapfirst_to_marker_vector, T_between_gaps_vector, T_gaplast_to_marker_vector);

	// prepare marker 3d points (actually only the point (0,0,0) in the marker coordinate system
	std::vector< std::vector<cv::Point3f> > pattern_points_3d(T_gapfirst_to_marker_vector.size(), std::vector<cv::Point3f>(1, cv::Point3f(0.f, 0.f, 0.f)));

	// extrinsic calibration optimization
	for (int i=0; i<optimization_iterations_; ++i)
	{
		for ( int j=0; j<transforms_to_calibrate_.size(); ++j )
		{
			extrinsicCalibration(pattern_points_3d, T_gapfirst_to_marker_vector, T_between_gaps_vector, T_gaplast_to_marker_vector, calibration_order_[j]);
		}
	}

	// display and save calibration parameters
	RobotCalibration::displayAndSaveCalibrationResult("camera_calibration_pitag_urdf.txt");

	calibrated_ = true;
	return true;
}

bool CameraBaseCalibrationPiTag::acquireCalibrationData(const bool load_data, std::vector<cv::Mat>& T_gapfirst_to_marker_vector,
														std::vector< std::vector<cv::Mat> >& T_between_gaps_vector, std::vector<cv::Mat>& T_gaplast_to_marker_vector)
{
	std::stringstream path;
	path << calibration_storage_path_ << "pitag_data.yml";

	// capture images from different perspectives
	if (load_data == false)
	{
		const int number_images_to_capture = (int)camera_configurations_.size();
		for (int image_counter = 0; image_counter < number_images_to_capture; ++image_counter)
		{
			if ( !ros::ok() )
				return false;

			std::cout << "Configuration " << (image_counter+1) << "/" << number_images_to_capture << std::endl;

			moveRobot(image_counter);

			// wait a moment here to mitigate shaking camera effects.
			ros::Duration(3).sleep();

			// extract marker points
			cob_object_detection_msgs::DetectObjects detect;
			// hack to fix camera providing outdated image!
			pitag_client_.call(detect);
			detect.response.object_list.detections.clear();
			pitag_client_.call(detect);
			detect.response.object_list.detections.clear();
			pitag_client_.call(detect);
			detect.response.object_list.detections.clear();
			pitag_client_.call(detect);
			detect.response.object_list.detections.clear();
			pitag_client_.call(detect);

			if (detect.response.object_list.detections.size() == 0)
				continue;

			for (size_t detection=0; detection<detect.response.object_list.detections.size(); ++detection)
			{
				cob_object_detection_msgs::Detection& det = detect.response.object_list.detections[detection];
				std::string marker_frame = marker_frame_base_name_ + det.label.substr(3);	// yields e.g. "tag_18"

				// retrieve transformations
				cv::Mat T_gapfirst_to_marker, T_gaplast_to_camera_optical, T_camera_optical_to_marker, T_gaplast_to_marker;
				std::vector<cv::Mat> T_between_gaps;
				bool result = calculateTransformationChains(T_gapfirst_to_marker, T_between_gaps, T_gaplast_to_camera_optical, marker_frame);
				if (result == false)
					continue;

				// extract trafo from camera optical to marker_frame
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

				T_camera_optical_to_marker = transform_utilities::makeTransform(rotcv, transcv);
//std::cout << marker_frame << ": " << T_camera_optical_to_marker.at<double>(0,3) << ", " << T_camera_optical_to_marker.at<double>(1,3) << ", " << T_camera_optical_to_marker.at<double>(2,3) << std::endl;
				T_gaplast_to_marker = T_gaplast_to_camera_optical*T_camera_optical_to_marker;
//std::cout << "gap_last: " << T_gaplast_to_marker.at<double>(0,3) << ", " << T_gaplast_to_marker.at<double>(1,3) << ", " << T_gaplast_to_marker.at<double>(2,3) << std::endl;
				// attach data to array
				T_gapfirst_to_marker_vector.push_back(T_gapfirst_to_marker);
//std::cout << "gap_first: " << T_gapfirst_to_marker.at<double>(0,3) << ", " << T_gapfirst_to_marker.at<double>(1,3) << ", " << T_gapfirst_to_marker.at<double>(2,3) << std::endl;
				if ( T_between_gaps.empty() )
					T_between_gaps = std::vector<cv::Mat>(1, cv::Mat::zeros(cv::Size(1,1), CV_64FC1));

				T_between_gaps_vector.push_back(T_between_gaps);
				T_gaplast_to_marker_vector.push_back(T_gaplast_to_marker);

				ROS_INFO("=#=#=#=#=#=#=#=#= Found %s", marker_frame.c_str());
			}
		}

		// save transforms to file
		cv::FileStorage fs(path.str().c_str(), cv::FileStorage::WRITE);
		if (fs.isOpened())
		{
			fs << "T_gapfirst_to_marker_vector" << T_gapfirst_to_marker_vector;
			fs << "T_gaplast_to_marker_vector" << T_gaplast_to_marker_vector;
			fs << "T_between_gaps_vector" << T_between_gaps_vector;
		}
		else
		{
			ROS_WARN("Could not write transformations to file '%s'.", path.str().c_str());
		}
		fs.release();
	}
	else
	{
		// load data from file
		cv::FileStorage fs(path.str().c_str(), cv::FileStorage::READ);
		if (fs.isOpened())
		{
			fs["T_gapfirst_to_marker_vector"] >> T_gapfirst_to_marker_vector;
			fs["T_gaplast_to_marker_vector"] >> T_gaplast_to_marker_vector;
			fs["T_between_gaps_vector"] >> T_between_gaps_vector;
		}
		else
		{
			ROS_WARN("Could not read transformations from file '%s'.", path.str().c_str());
		}
		fs.release();
	}

	std::cout << "Captured markers: " << T_gaplast_to_marker_vector.size() << std::endl;
	return true;
}
