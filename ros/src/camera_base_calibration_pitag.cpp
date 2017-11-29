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
	node_handle_.param<std::string>("get_fiducials_topic", get_fiducials_topic_, "/fiducials/get_fiducials");
	std::cout << "get_fiducials_topic: " << get_fiducials_topic_ << std::endl;


	pitag_client_ = node_handle_.serviceClient<cob_object_detection_msgs::DetectObjects>(get_fiducials_topic_);

	ROS_INFO("CameraBaseCalibrationPiTag initialized.");
}

CameraBaseCalibrationPiTag::~CameraBaseCalibrationPiTag()
{
}

bool CameraBaseCalibrationPiTag::calibrateCameraToBase(const bool load_data)
{
	// setup storage folder
	//int return_value = system("mkdir -p robotino_calibration/camera_calibration");

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
	std::vector<cv::Mat> calibrated_Transforms;
	calibrated_Transforms.push_back(T_base_to_torso_lower_);
	calibrated_Transforms.push_back(T_torso_upper_to_camera_);
	displayAndSaveCalibrationResult(calibrated_Transforms);

	// save calibration
	saveCalibration();
	calibrated_ = true;

	return true;
}

bool CameraBaseCalibrationPiTag::calibrateCameraToBaseNEW(const bool load_data)
{
	// acquire images
	std::vector<cv::Mat> T_base_to_marker_vector;
	std::vector< std::vector<cv::Mat> > T_between_gaps_vector;
	std::vector<cv::Mat> T_camera_to_marker_vector;
	acquireCalibrationDataNEW(robot_configurations_, load_data,
			T_base_to_marker_vector, T_between_gaps_vector, T_camera_to_marker_vector);

	// prepare marker 3d points (actually only the point (0,0,0) in the marker coordinate system
	std::vector< std::vector<cv::Point3f> > pattern_points_3d(T_base_to_marker_vector.size(), std::vector<cv::Point3f>(1, cv::Point3f(0.f, 0.f, 0.f)));

	// extrinsic calibration optimization
	for (int i=0; i<optimization_iterations_; ++i)
	{
		for ( int j=0; j<transforms_to_calibrate_.size(); ++j )
		{
			extrinsicCalibration(pattern_points_3d, T_base_to_marker_vector, T_between_gaps_vector, T_camera_to_marker_vector, calibration_order_[j]);
		}
	}

	// display calibration parameters
	displayAndSaveCalibrationResult();

	// save calibration
	saveCalibration();
	calibrated_ = true;

	return true;
}

bool CameraBaseCalibrationPiTag::acquireCalibrationDataNEW(const std::vector<calibration_utilities::RobotConfiguration>& robot_configurations,
		const bool load_data, std::vector<cv::Mat>& T_base_to_marker_vector,
		std::vector< std::vector<cv::Mat> >& T_between_gaps_vector, std::vector<cv::Mat>& T_camera_to_marker_vector)
{
	std::stringstream path;
	path << calibration_storage_path_ << "pitag_data.yml";

	// capture images from different perspectives
	if (load_data == false)
	{
		const int number_images_to_capture = (int)robot_configurations.size();
		for (int image_counter = 0; image_counter < number_images_to_capture; ++image_counter)
		{
			if ( !ros::ok() )
				return false;

			std::cout << "Configuration " << (image_counter+1) << "/" << number_images_to_capture << std::endl;

			moveRobot(robot_configurations[image_counter]);

			// wait a moment here to mitigate shaking camera effects.
			ros::Duration(3).sleep();

			// extract marker points
			cob_object_detection_msgs::DetectObjects detect;
			pitag_client_.call(detect);
			if (detect.response.object_list.detections.size() == 0)
				continue;

			for (size_t detection=0; detection<detect.response.object_list.detections.size(); ++detection)
			{
				cob_object_detection_msgs::Detection& det = detect.response.object_list.detections[detection];
				std::string marker_frame = marker_frame_base_name_ + det.label.substr(3);	// yields e.g. "tag_18"

				// retrieve transformations
				cv::Mat T_base_to_marker, T_camera_to_camera_optical, T_camera_optical_to_marker, T_camera_to_marker;
				std::vector<cv::Mat> T_between_gaps;
				bool result = true;
				result &= transform_utilities::getTransform(transform_listener_, base_frame_, marker_frame, T_base_to_marker);
				result &= transform_utilities::getTransform(transform_listener_, camera_frame_, camera_optical_frame_, T_camera_to_camera_optical);

				for ( int i=0; i<transforms_to_calibrate_.size()-1; ++i )
				{
					if ( transforms_to_calibrate_[i].parent_ == transforms_to_calibrate_[i].child_ ) // several gaps in a row, no certain trafos in between
						continue;

					cv::Mat temp;
					result &= transform_utilities::getTransform(transform_listener_, transforms_to_calibrate_[i].child_, transforms_to_calibrate_[i+1].parent_, temp);
					T_between_gaps.push_back(temp);
					transforms_to_calibrate_[i].trafo_until_next_gap_idx_ = T_between_gaps.size()-1;
				}

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
				T_camera_optical_to_marker = transform_utilities::makeTransform(rotcv, transcv);
				T_camera_to_marker = T_camera_to_camera_optical*T_camera_optical_to_marker;

				// attach data to array
				T_base_to_marker_vector.push_back(T_base_to_marker);
				T_between_gaps_vector.push_back(T_between_gaps);
				T_camera_to_marker_vector.push_back(T_camera_to_marker);

				ROS_INFO("=#=#=#=#=#=#=#=#= Found %s", marker_frame.c_str());
			}
		}

		// save transforms to file
		cv::FileStorage fs(path.str().c_str(), cv::FileStorage::WRITE);
		if (fs.isOpened())
		{
			fs << "T_base_to_marker_vector" << T_base_to_marker_vector;
			fs << "T_camera_to_marker_vector" << T_camera_to_marker_vector;
			fs << "T_torso_lower_to_torso_upper_vector" << T_between_gaps_vector;
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
			fs["T_base_to_marker_vector"] >> T_base_to_marker_vector;
			fs["T_camera_to_marker_vector"] >> T_camera_to_marker_vector;
			fs["T_torso_lower_to_torso_upper_vector"] >> T_between_gaps_vector;
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

bool CameraBaseCalibrationPiTag::acquireCalibrationData(const std::vector<calibration_utilities::RobotConfiguration>& robot_configurations,
		const bool load_data, std::vector<cv::Mat>& T_base_to_marker_vector,
		std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector, std::vector<cv::Mat>& T_camera_to_marker_vector)
{
	std::stringstream path;
	path << calibration_storage_path_ << "pitag_data.yml";

	// capture images from different perspectives
	if (load_data == false)
	{
		const int number_images_to_capture = (int)robot_configurations.size();
		for (int image_counter = 0; image_counter < number_images_to_capture; ++image_counter)
		{
			if ( !ros::ok() )
				return false;

			std::cout << "Configuration " << (image_counter+1) << "/" << number_images_to_capture << std::endl;

			moveRobot(robot_configurations[image_counter]);
			// wait a moment here to mitigate shaking camera effects?

			ros::Duration(3).sleep();

			// NOT necessary, apparently the results are good enough and simulated tests show that the influence of little shaking can be compensated by enough data

			// extract marker points
			cob_object_detection_msgs::DetectObjects detect;
			pitag_client_.call(detect);
			if (detect.response.object_list.detections.size() == 0)
				continue;

			for (size_t detection=0; detection<detect.response.object_list.detections.size(); ++detection)
			{
				cob_object_detection_msgs::Detection& det = detect.response.object_list.detections[detection];
				std::string marker_frame = marker_frame_base_name_ + det.label.substr(3);	// yields e.g. "tag_18"

				// retrieve transformations
				cv::Mat T_base_to_marker, T_torso_lower_to_torso_upper, T_camera_to_camera_optical, T_camera_optical_to_marker, T_camera_to_marker;
				bool result = true;
				result &= transform_utilities::getTransform(transform_listener_, base_frame_, marker_frame, T_base_to_marker);
				result &= transform_utilities::getTransform(transform_listener_, torso_lower_frame_, torso_upper_frame_, T_torso_lower_to_torso_upper);
				result &= transform_utilities::getTransform(transform_listener_, camera_frame_, camera_optical_frame_, T_camera_to_camera_optical);
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
				T_camera_optical_to_marker = transform_utilities::makeTransform(rotcv, transcv);
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
		// load data from file
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

		// some testing code for checking the influence of different error sources on the calibration results
		// set sim=true for using this verification method
		/*bool sim = false;
		if (sim==true)
		{
			std::cout << "=============================== using simulated data ==================================" << std::endl;
			T_base_to_marker_vector.clear();
			T_torso_lower_to_torso_upper_vector.clear();
			T_camera_to_marker_vector.clear();

			const double laser_scanner_yaw_err = 0.04;
			const double laser_scanner_pitch_err = 0.04;

			const double laser_scanner_height = 0.15;
			cv::Mat T_base_to_laser = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(0., 0., 0.), cv::Mat(cv::Vec3d(0.2, 0., laser_scanner_height)));
			cv::Mat T_base_to_laser_err = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(laser_scanner_yaw_err, laser_scanner_pitch_err, 0.), cv::Mat(cv::Vec3d(0.2, 0., laser_scanner_height)));
			cv::Mat points_3d_in_corner = (cv::Mat_<double>(9,3) << -0.34, 0., 0.758,
																	-0.481, 0., 0.65,
																	-0.34, 0., 0.533,
																	0., 0.612, 0.765,
																	0., 0.472, 0.658,
																	0., 0.612, 0.539,
																	-0.071, 0.293, 0.,
																	-0.21, 0.293, 0.,
																	-0.348, 0.293, 0.);
			const double max_marker_measurement_err = 0.002;
			cv::Mat points_3d_in_corner_err = (cv::Mat_<double>(9,3) << max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX), 0., max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX),
					max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX), 0., max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX),
					max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX), 0., max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX),
					0., max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX), max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX),
					0., max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX), max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX),
					0., max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX), max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX),
					max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX), max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX), 0.,
					max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX), max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX), 0.,
					max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX), max_marker_measurement_err*(-1.+2.*(double)rand()/(double)RAND_MAX), 0.);
			cv::Mat T_base_to_torsolower = T_base_to_torso_lower_; //transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(0., 0., 0.), cv::Mat(cv::Vec3d(0.3, 0., 0.69)));
			cv::Mat T_torsoupper_to_camera = T_torso_upper_to_camera_; //transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(0., 0., -1.57), cv::Mat(cv::Vec3d(0.015, 0.065, 0.)));


			const int number_images_to_capture = (int)robot_configurations.size();
			for (int image_counter = 0; image_counter < number_images_to_capture; ++image_counter)
			{
//				if (image_counter>0 && robot_configurations[image_counter-1].pose_x_==robot_configurations[image_counter].pose_x_ && robot_configurations[image_counter-1].pose_y_==robot_configurations[image_counter].pose_y_)
//					continue;

				cv::Mat T_laser_to_corner = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(0., 0., 0.), cv::Mat(cv::Vec3d(robot_configurations[image_counter].pose_x_, robot_configurations[image_counter].pose_y_, -laser_scanner_height)));
				cv::Mat T_laser_to_corner_err = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(0., 0., 0.), cv::Mat(cv::Vec3d(robot_configurations[image_counter].pose_x_+0.005*(-1.+2.*(double)rand()/(double)RAND_MAX), robot_configurations[image_counter].pose_y_+0.005*(-1.+2.*(double)rand()/(double)RAND_MAX), -laser_scanner_height)));
				for (int marker_index=0; marker_index<points_3d_in_corner.rows; ++marker_index)
				{
					// simulate ratio of found markers
					if (((double)rand()/(double)RAND_MAX) > 0.2)
						continue;

					// base to marker
					cv::Mat T_corner_to_marker = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(0., 0., 0.),
							cv::Mat(cv::Vec3d(points_3d_in_corner.at<double>(marker_index,0), points_3d_in_corner.at<double>(marker_index,1), points_3d_in_corner.at<double>(marker_index,2))));
					cv::Mat T_corner_to_marker_err = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(0., 0., 0.),
							cv::Mat(cv::Vec3d(points_3d_in_corner.at<double>(marker_index,0)+points_3d_in_corner_err.at<double>(marker_index,0), points_3d_in_corner.at<double>(marker_index,1)+points_3d_in_corner_err.at<double>(marker_index,1), points_3d_in_corner.at<double>(marker_index,2)+points_3d_in_corner_err.at<double>(marker_index,2))));
					cv::Mat T_base_to_marker = T_base_to_laser * T_laser_to_corner * T_corner_to_marker;
					cv::Mat T_base_to_marker_err = T_base_to_laser_err * T_laser_to_corner_err * T_corner_to_marker_err;

					// torso_lower to torso_upper
					cv::Mat T_torsolower_to_pan = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(robot_configurations[image_counter].pan_angle_, 0., 0.), cv::Mat(cv::Vec3d(0., 0., 0.)));
					cv::Mat T_pan_to_tilt1 = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(0., 0., 1.57), cv::Mat(cv::Vec3d(0., 0., 0.)));
					cv::Mat T_tilt1_to_tilt = transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(-robot_configurations[image_counter].tilt_angle_, 0., 0.), cv::Mat(cv::Vec3d(0., 0., 0.)));
					cv::Mat T_torso_lower_to_torso_upper = T_torsolower_to_pan * T_pan_to_tilt1 * T_tilt1_to_tilt;

					// camera to marker
					cv::Mat T_base_to_camera = T_base_to_torsolower * T_torso_lower_to_torso_upper * T_torsoupper_to_camera;
					cv::Mat T_camera_to_marker = T_base_to_camera.inv() * T_base_to_marker;
					cv::Mat T_base_to_camera_err = T_base_to_torsolower * T_torso_lower_to_torso_upper
							* transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(0., -0.07, 0.), cv::Mat(cv::Vec3d(0., 0., 0.)))
							* T_torsoupper_to_camera;
					const double marker_detection_max_offset = 0.02;
					cv::Mat T_camera_to_marker_err = T_base_to_camera_err.inv() * T_base_to_marker
							* transform_utilities::makeTransform(transform_utilities::rotationMatrixFromYPR(0., 0., 0.), cv::Mat(cv::Vec3d(marker_detection_max_offset*(-1.+2.*(double)rand()/(double)RAND_MAX), marker_detection_max_offset*(-1.+2.*(double)rand()/(double)RAND_MAX), marker_detection_max_offset*(-1.+2.*(double)rand()/(double)RAND_MAX))));

					// attach data to array
					T_base_to_marker_vector.push_back(T_base_to_marker_err);
					T_torso_lower_to_torso_upper_vector.push_back(T_torso_lower_to_torso_upper);
					T_camera_to_marker_vector.push_back(T_camera_to_marker_err);
				}
			}
		}*/
	}

	std::cout << "Captured markers: " << T_camera_to_marker_vector.size() << std::endl;
	return true;
}

bool CameraBaseCalibrationPiTag::saveCalibration()
{
	bool success = true;

	// save calibration
	std::string filename = calibration_storage_path_ + "camera_calibration.yml";
	cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
	if (fs.isOpened() == true)
	{
		fs << "T_base_to_torso_lower" << T_base_to_torso_lower_;
		fs << "T_torso_upper_to_camera" << T_torso_upper_to_camera_;
	}
	else
	{
		std::cout << "Error: CameraBaseCalibrationPiTag::saveCalibration: Could not write calibration to file.";
		success = false;
	}
	fs.release();

	return success;
}

bool CameraBaseCalibrationPiTag::loadCalibration()
{
	bool success = true;

	// load calibration
	std::string filename = calibration_storage_path_ + "camera_calibration.yml";
	cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
	if (fs.isOpened() == true)
	{
		fs["T_base_to_torso_lower"] >> T_base_to_torso_lower_;
		fs["T_torso_upper_to_camera"] >> T_torso_upper_to_camera_;
	}
	else
	{
		std::cout << "Error: CameraBaseCalibrationPiTag::loadCalibration: Could not read calibration from file.";
		success = false;
	}
	fs.release();

	calibrated_ = true;

	return success;
}

void CameraBaseCalibrationPiTag::getCalibration(cv::Mat& T_base_to_torso_lower, cv::Mat& T_torso_upper_to_camera)
{
	if (calibrated_ == false && loadCalibration() == false)
	{
		std::cout << "Error: CameraBaseCalibrationPiTag not calibrated and no calibration data available on disk." << std::endl;
		return;
	}

	T_base_to_torso_lower = T_base_to_torso_lower_.clone();
	T_torso_upper_to_camera = T_torso_upper_to_camera_.clone();
}
