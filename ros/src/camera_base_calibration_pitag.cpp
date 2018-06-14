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
	for ( int l=0; l<calibration_setups_.size(); ++l )
	{
		int iterations = 0;

		// if there is only one trafo to calibrate, we don't need to optimize over iterations
		if ( calibration_setups_[l].transforms_to_calibrate_.size() > 1 )
			iterations = optimization_iterations_;
		else
			iterations = 1;

		for (int i=0; i<iterations; ++i)
		{
			for ( int j=0; j<calibration_setups_[l].transforms_to_calibrate_.size(); ++j )
			{
				extrinsicCalibration(pattern_points_3d, T_gapfirst_to_marker_vector, T_between_gaps_vector, T_gaplast_to_marker_vector, calibration_setups_[l]);
			}
		}

		calibration_setups_[l].calibrated_ = true;  // setup has been calibrated
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
	path << calibration_storage_path_ << "pitag_data.yml";  // retrieve from interface instead!

	if ( load_data == false )
	{
		const int num_configs = calibration_interface_->getConfigurationCount();
		for ( int config_counter = 0; config_counter < num_configs; ++config_counter )
		{
			if ( !ros::ok() )
				return false;

			std::cout << "Configuration " << (config_counter+1) << "/" << num_configs << std::endl;

			calibration_interface_->moveRobot(config_counter);

			// wait a moment here to mitigate shaking camera effects.
			ros::Duration(3).sleep();

			//calibration_interface_->waitForTransforms();  // give user possibility to halt here until tf has been provided with updated transforms (camera detections)

			// grab transforms for each setup and store them
			for ( int i=0; i<calibration_setups_.size(); ++i )
			{
				// snapshot parent branch
				populateTFSnapshots(calibration_setups_[i]);

				// snapshot child branch
				//populateTFSnapshots(calibration_setups_[i].child_branch);






				/*std::vector<cv::Mat> last_trafo_to_parent_markers;
				std::vector<cv::Mat> last_trafo_to_child_markers;
				std::vector< std::vector<cv::Mat> > in_between_trafos_parent_branch;  // origin to parent_markers is parent branch
				std::vector< std::vector<cv::Mat> > in_between_trafos_child_branch;

				// get last frame in parent_marker vector (remember, the transforms in there are in order, so the last is actually the last in tf tree) and
				// retrieve the trafo from child of last frame to all parent_markers
				cv::Mat trafo;
				CalibrationInfo info = calibration_setups_[i].origin_to_parent_marker_uncertainties_[calibration_setups_[i].origin_to_parent_marker_uncertainties_.size()-1];  // take last entry
				for ( int j=0; j<info.parent_markers_.size(); ++j )
				{
					if ( transform_utilities::getTransform(transform_listener_, info.child_, info.parent_markers_[j], trafo, true) )
						last_trafo_to_parent_markers.push_back(trafo);
				}

				// same for child_marker vector
				info = calibration_setups_[i].origin_to_child_marker_uncertainties_[calibration_setups_[i].origin_to_parent_marker_uncertainties_.size()-1];  // take last entry
				for ( int j=0; j<info.child_markers_.size(); ++j )
				{
					if ( transform_utilities::getTransform(transform_listener_, info.child_, info.child_markers_[j], trafo, true) )
						last_trafo_to_child_markers.push_back(trafo);
				}

				// get in between trafos, that means in between the uncertain trafos there might be well known trafos. Retreive them from tf as well
				std::vector<cv::Mat> in_between_trafos;
				std::string first_parent_frame = calibration_setups_[i].origin_to_parent_marker_uncertainties_[0].parent_;  // remember, vector is sorted!
				if ( first_parent_frame.compare(calibration_setups_[i].origin_) != 0 )  // add origin to first parent if it is not equal to origin (zero Mat in that case)
				{
					if ( transform_utilities::getTransform(transform_listener_, calibration_setups_[i].origin_, calibration_setups_[i].origin_to_parent_marker_uncertainties_[0].parent_, trafo, true) )
						in_between_trafos.push_back(trafo);
				}

				for ( int j=0; j<calibration_setups_[i].origin_to_parent_marker_uncertainties_.size()-1; ++j )
				{
					CalibrationInfo current_info = calibration_setups_[i].origin_to_parent_marker_uncertainties_[j];
					CalibrationInfo next_info = calibration_setups_[i].origin_to_parent_marker_uncertainties_[j+1];
					if ( current_info.child_ == next_info.parent_ ) // transforms come one after the other, so there is no transform in between -> next
						continue;

					if ( transform_utilities::getTransform(transform_listener_, current_info.child_, next_info.parent_, trafo, true) )
						in_between_trafos.push_back(trafo);
				}

				if ( !in_between_trafos.empty() )


				// same for in between trafos of child branch
				std::string first_parent_frame = calibration_setups_[i].origin_to_parent_marker_uncertainties_[0].parent_;  // remember, vector is sorted!
				if ( first_parent_frame.compare(calibration_setups_[i].origin_) != 0 )  // add origin to first parent if it is not equal to origin (zero Mat in that case)
				{
					if ( transform_utilities::getTransform(transform_listener_, calibration_setups_[i].origin_, calibration_setups_[i].origin_to_parent_marker_uncertainties_[0].parent_, trafo, true) )
						in_between_trafos_parent_branch.push_back(trafo);
				}

				for ( int j=0; j<calibration_setups_[i].origin_to_parent_marker_uncertainties_.size()-1; ++j )
				{
					CalibrationInfo current_info = calibration_setups_[i].origin_to_parent_marker_uncertainties_[j];
					CalibrationInfo next_info = calibration_setups_[i].origin_to_parent_marker_uncertainties_[j+1];
					if ( current_info.child_ == next_info.parent_ ) // transforms come one after the other, so there is no transform in between -> next
						continue;

					if ( transform_utilities::getTransform(transform_listener_, current_info.child_, next_info.parent_, trafo, true) )
						in_between_trafos_parent_branch.push_back(trafo);
				}*/
			}

			// retrieve transformations
			std::vector<cv::Mat> T_camera_optical_to_markers = calibration_interface_->getCameraOpticalToMarkers();

			// How to make several calibration chains possible here? start and endpoint are needed

			if ( T_camera_optical_to_markers.empty() )
				continue;

			cv::Mat T_gapfirst_to_marker, T_gaplast_to_camera_optical, T_camera_optical_to_marker, T_gaplast_to_marker;
			std::vector<cv::Mat> T_between_gaps;
			bool result = calculateTransformationChains(T_gapfirst_to_marker, T_between_gaps, T_gaplast_to_camera_optical, marker_frame);

			if ( result )
			{
				for ( int marker_counter = 0; marker_counter < T_camera_optical_to_markers.size(); ++marker_counter )
				{
					T_gapfirst_to_marker_vector.push_back()
					T_between_gaps_vector.push_back(T_between_gaps);
					T_gaplast_to_marker_vector.push_back(T_gaplast_to_marker);
				}
			}
		}
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


/*bool CameraBaseCalibrationPiTag::acquireCalibrationData(const bool load_data, std::vector<cv::Mat>& T_gapfirst_to_marker_vector,
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
}*/
