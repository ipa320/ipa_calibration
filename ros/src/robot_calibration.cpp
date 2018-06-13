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
 * Date of creation: October 2016
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


#include <robotino_calibration/robot_calibration.h>
#include <robotino_calibration/timer.h>
#include <robotino_calibration/transformation_utilities.h>
#include <std_msgs/Float64MultiArray.h>

//Exception
#include <exception>
#include <tf/exceptions.h>

// File writing
#include <sstream>
#include <fstream>

// Boost
#include <boost/filesystem.hpp>


// ToDo: Implement calibration order mechanics!


RobotCalibration::RobotCalibration(ros::NodeHandle nh, CalibrationInterface* interface) :
	node_handle_(nh), transform_listener_(nh), calibrated_(false), calibration_interface_(interface)
{
	// load parameters
	std::cout << "\n========== RobotCalibration Parameters ==========\n";

	// hack to fix tf::waitForTransform throwing error that transforms do not exist when now() == 0 at startup
	ROS_INFO("Waiting for TF listener to initialize...");
	Timer timeout;
	while ( timeout.getElapsedTimeInSec() < 10.f )
	{
		if ( ros::Time::now().isValid() )
			break;

		ros::Duration(0.2f).sleep();
	}
	ROS_INFO("End waiting for TF to initialize.");

	node_handle_.param<std::string>("camera_optical_frame", camera_optical_frame_, "");
	std::cout << "camera_optical_frame: " << camera_optical_frame_ << std::endl;
	node_handle_.param<std::string>("calibration_storage_path", calibration_storage_path_, "/calibration");
	std::cout << "calibration_storage_path: " << calibration_storage_path_ << std::endl;



	// load gaps including its initial values
	std::vector<std::string> uncertainties_list;
	node_handle_.getParam("uncertainties_list", uncertainties_list);

	if ( uncertainties_list.size() % 5 != 0 )
		ROS_WARN("Size of uncertainsties_list is not a factor of 4: [parent, child, parent_marker, child_marker, weight]");

	// create calibration setups, check for errors
	for ( int i=0; i<uncertainties_list.size(); i+=5 )
	{
		if ( !transform_listener_.frameExists(uncertainties_list[i]) || !transform_listener_.frameExists(uncertainties_list[i+1]) )
		{
			ROS_ERROR("parent frame %s or child frame %s does not exist, skipping.", uncertainties_list[i].c_str(), uncertainties_list[i+1].c_str());
			continue;
		}

		if ( !transform_listener_.frameExists(uncertainties_list[i+2]) || !transform_listener_.frameExists(uncertainties_list[i+3]) )
		{
			ROS_ERROR("parent_marker frame %s or child_marker frame %s does not exist, skipping.", uncertainties_list[i+2].c_str(), uncertainties_list[i+3].c_str());
			continue;
		}

		std::string child = uncertainties_list[i+1];  // child
		std::string actual_parent = "";
		transform_listener_.getParent(child, ros::Time(0), actual_parent);

		if ( actual_parent.compare(uncertainties_list[i]) == 0 )  // compare actual parent of child with parent user has input
		{
			// get origin of trafo chain, chains with same origin will be stored into one calibration setup
			std::string parent_marker = uncertainties_list[i+2];
			std::string child_marker = uncertainties_list[i+3];
			std::string origin;
			if ( getOrigin(parent_marker, child_marker, origin) )
			{
				bool found = false;
				for ( int j=0; j<calibration_setups_.size(); ++j )  // merge entries that have the same parent and child_marker
				{
					if ( origin.compare(calibration_setups_[j].origin_) == 0 )
					{
						feedCalibrationSetup(calibration_setups_[j], actual_parent, child, parent_marker, child_marker);
						found = true;
						break;
					}
				}

				if ( !found )  // no match has been found -> create new calibration setup
				{
					CalibrationSetup setup;
					setup.origin_ = origin;
					feedCalibrationSetup(setup, actual_parent, child, parent_marker, child_marker);
					calibration_setups_.push_back(setup);
				}
			}
			else
				ROS_WARN("No mutual origin found between %s and %s, skipping.", parent_marker.c_str(), child_marker.c_str());
		}
		else
			ROS_WARN("Given parent frame does not match actual parent (%s) in tf tree. Given parent: %s, given child: %s", actual_parent, uncertainties_list[i].c_str(), child.c_str());
	}

	// remove all corrupted entries in calibration setups vector
	for ( int i=0; i<calibration_setups_.size(); ++i )
	{
		if ( calibration_setups_[i].transforms_to_calibrate_.size() == 0 )  // remove corrupted setups
		{
			ROS_WARN("No transform to calibrate found in calibration setup -> removing setup.");
			calibration_setups_.erase(calibration_setups_.begin()+i);
			--i;
			continue;
		}

		for ( int j=0; j<calibration_setups_[i].transforms_to_calibrate_.size(); ++j )  // check for corrupted transforms
		{
			if ( calibration_setups_[i].transforms_to_calibrate_[j].parent_markers_.size() == 0 ||
					calibration_setups_[i].transforms_to_calibrate_[j].child_markers_.size() == 0 )
			{
				ROS_WARN("Empty parent or child_markers for %s to %s -> removing transform", calibration_setups_[i].transforms_to_calibrate_[j].parent_, calibration_setups_[i].transforms_to_calibrate_[j].child_);
				calibration_setups_[i].transforms_to_calibrate_.erase(calibration_setups_[i].transforms_to_calibrate_.begin()+j);
			}
		}

		if ( calibration_setups_[i].transforms_to_calibrate_.size() == 0 )  // check again
		{
			ROS_WARN("No transform to calibrate found in calibration setup -> removing setup.");
			calibration_setups_.erase(calibration_setups_.begin()+i);
			--i;
		}
	}

	// organize/sort calibration data
	// the transforms_to_calibrate_ vector for each setup is not useful for a calibration, as it is unsorted. Sort it now to make the process feasible.
	for ( int i=0; i<calibration_setups_.size(); ++i )
	{
		// 1. Find the corresponding closed chain consisting of two branches for each calibration setup (two branches: from parent_marker and child_marker to mutual origin)
		std::vector<std::string> parent_marker_to_origin;
		std::vector<std::string> child_marker_to_origin;
		std::string a_parent_marker = calibration_setups_[i].transforms_to_calibrate_[0].parent_markers_[0];  // get a parent_marker, actually any is suffice
		std::string a_child_marker = calibration_setups_[i].transforms_to_calibrate_[0].child_markers_[0];  // get a child_marker, any is suffice

		// retrieve the chain, that connects parent_marker and child_marker. Store it in two different vectors: one from origin to parent_marker and the other from origin to child_marker
		getChain(a_parent_marker, a_child_marker, calibration_setups_[i].origin_, parent_marker_to_origin, child_marker_to_origin);

		// 2. Find natural order of the frames to be calibrated in the resulting closed chain
		// search trafos_to_calibrate inside parent_marker_to_origin backwards (from origin to parent_marker)
		for ( int j=parent_marker_to_origin.size()-1; j>=1; --j )
		{
			for ( int k=0; k<calibration_setups_[i].transforms_to_calibrate_.size(); ++k )
			{
				if ( parent_marker_to_origin[j].compare(calibration_setups_[i].transforms_to_calibrate_[k].parent_) == 0 &&
						parent_marker_to_origin[j+1].compare(calibration_setups_[i].transforms_to_calibrate_[k].child_) == 0 )
				{
					CalibrationInfo info = calibration_setups_[i].transforms_to_calibrate_[k];
					calibration_setups_[i].origin_to_parent_marker_uncertainties_.push_back(info);  // now put the transforms in order, so that they reflect the order in which they appear in tf
				}
			}
		}

		// search trafos_to_calibrate inside child_marker_to_origin backwards (from origin to child_marker)
		for ( int j=child_marker_to_origin.size()-1; j>=0; --j )
		{
			for ( int k=0; k<calibration_setups_[i].transforms_to_calibrate_.size(); ++k )
			{
				if ( child_marker_to_origin[j].compare(calibration_setups_[i].transforms_to_calibrate_[k].parent_) == 0 &&
						child_marker_to_origin[j+1].compare(calibration_setups_[i].transforms_to_calibrate_[k].child_) == 0 )
				{
					CalibrationInfo info = calibration_setups_[i].transforms_to_calibrate_[k];
					calibration_setups_[i].origin_to_child_marker_uncertainties_.push_back(info);  // now put the transforms in order, so that they reflect the order in which they appear in tf
				}
			}
		}
	}






	/*if ( uncertainties_list.size() % 2 != 0 )
		ROS_WARN("Size of uncertainsties_list is not a factor of two.");

	for ( int i=0; i<uncertainties_list.size(); i+=2 )
	{
		CalibrationInfo tmp;
		tmp.parent_ = uncertainties_list[i];
		tmp.child_ = uncertainties_list[i+1];
		tmp.trafo_until_next_gap_idx_ = -1;
		bool success = transform_utilities::getTransform(transform_listener_, tmp.parent_, tmp.child_, tmp.current_trafo_);

		if ( success == false )
		{
			ROS_FATAL("Could not retrieve transform from %s to %s from TF!", tmp.parent_.c_str(), tmp.child_.c_str());
			throw std::exception();
		}

		transforms_to_calibrate_.push_back(tmp);
	}*/

	if ( calibration_setups_.size() == 0 )
	{
		ROS_WARN("No calibration setup entry found: Exiting.");
		throw std::exception();
	}

	node_handle_.param("optimization_iterations", optimization_iterations_, 1000);

	if ( optimization_iterations_ <= 0 )
	{
		std::cout << "Invalid optimization_iterations value: " << optimization_iterations_ << " -> Setting value to 1000." << std::endl;
		optimization_iterations_ = 1000;
	}

	/*if ( transforms_to_calibrate_.size() == 1 )
	{
		std::cout << "Only one transform to calibrate: Setting optimization_iterations to 1." << std::endl;
		optimization_iterations_ = 1;
	}*/

	std::cout << "optimization_iterations: " << optimization_iterations_ << std::endl;

	node_handle_.getParam("calibration_order", calibration_order_);
	if ( calibration_order_.size() != calibration_setups_.size() )
	{
		ROS_FATAL("Size of calibration_order and size of calibration_setups do not match!");
		throw std::exception();
	}

	std::cout << "calibration order:" << std::endl;
	for ( int i=0; i<calibration_order_.size(); ++i )
	{
		if ( calibration_order_[i] < 1 || calibration_order_[i] > calibration_setups_.size() )
		{
			ROS_FATAL("Invalid index in calibration order %d", calibration_order_[i]);
			throw std::exception();
		}
		else
		{
			calibration_order_[i] = calibration_order_[i]-1; // zero-indexed values from now on
			//std::cout << (i+1) << ". From " << transforms_to_calibrate_[calibration_order_[i]].parent_ << " to " << transforms_to_calibrate_[calibration_order_[i]].child_ << std::endl;
			//std::cout << "Initial transform: " << transforms_to_calibrate_[calibration_order_[i]].current_trafo_ << std::endl;
		}
	}

	createStorageFolder();

	if (calibration_interface_ == 0) // Throw exception, as we need an calibration interface in order to function properly!
	{
		ROS_FATAL("Could not create a calibration interface.");
		throw std::exception();
	}
}

RobotCalibration::~RobotCalibration()
{
	if ( calibration_interface_ != 0 )
		delete calibration_interface_;
}

// create data storage path if it does not yet exist
void RobotCalibration::createStorageFolder()
{
	boost::filesystem::path storage_path(calibration_storage_path_);

	if (boost::filesystem::exists(storage_path) == false)
	{
		if (boost::filesystem::create_directories(storage_path) == false && boost::filesystem::exists(storage_path) == false)
		{
			std::cout << "Error: RobotCalibration: Could not create directory " << storage_path << std::endl;
			return;
		}
	}
}

bool RobotCalibration::getOrigin(const std::string parent_marker, const std::string child_marker, std::string &origin)
{
	std::vector<std::string> parent_marker_to_origin;
	std::string parent = parent_marker;
	while ( true )
	{
		if ( transform_listener_.getParent(parent, ros::Time(0), parent) )
			parent_marker_to_origin.push_back(parent);
		else
			break;
	}

	if ( parent_marker_to_origin.size() == 0 )
		return false;

	// check for first occurence of child_marker back-chain in parent_marker back-chain
	parent = child_marker;

	while ( true )
	{
		bool done = false;

		if ( transform_listener_.getParent(parent, ros::Time(0), parent) )
		{
			for ( int i=0; i<parent_marker_to_origin.size(); ++i )
			{
				if ( parent.compare(parent_marker_to_origin[i]) )  // find frame where parent_marker_to_origin and child_marker_to_origin meet
				{
					origin = parent;
					return true;
				}
			}
		}
		else
			break;
	}

	return false;
}

bool RobotCalibration::getChain(const std::string parent_marker, const std::string child_marker, const std::string origin,
				std::vector<std::string> &parent_marker_to_origin, std::vector<std::string> &child_marker_to_origin)
{
	// fill parent_marker_to_origin
	std::string parent = parent_marker;  // start with first parent marker, but don't add it to list as there might be several parent markers (for each tag)
	while ( parent.compare(origin) != 0 )
	{
		if ( transform_listener_.getParent(parent, ros::Time(0), parent) )  // first entry is frame before parent marker, as this frame is equal for all markers
			parent_marker_to_origin.push_back(parent);
		else
		{
			ROS_WARN("Could not create back chain for parent_marker %s, no parent for %s!", parent_marker.c_str(), parent.c_str());
			return false;
		}
	}

	// fill child_marker_to_origin
	parent = child_marker;
	while ( parent.compare(origin) != 0 )  // build back chain from it until both chains (parent_marker_to_origin and child_marker_to_origin) meet in their mutual base
	{
		if ( transform_listener_.getParent(parent, ros::Time(0), parent) )  // first entry is frame before parent marker, as this frame is equal for all markers
			child_marker_to_origin.push_back(parent);
		else
		{
			ROS_WARN("Could not create back chain for child_marker %s, no parent for %s!", child_marker.c_str(), parent.c_str());
			return false;
		}
	}

	return true;
}

void RobotCalibration::feedCalibrationSetup(CalibrationSetup &setup, const std::string parent, const std::string child,
		const std::string parent_marker, const std::string child_marker)
{
	// first check if transform already exists, if so extend it
	for ( int i=0; i<setup.transforms_to_calibrate_.size(); ++i )
	{
		if ( setup.transforms_to_calibrate_[i].parent_.compare(parent) == 0 &&
				setup.transforms_to_calibrate_[i].child_.compare(child) == 0 )  // transform already exists, check if we have to extend its marker frames
		{
			for ( int j=0; i<setup.transforms_to_calibrate_[i].parent_markers_.size(); ++j )  // parent_markers and child_markers always have same length
			{
				if ( setup.transforms_to_calibrate_[i].parent_markers_[j].compare(parent_marker) &&
						setup.transforms_to_calibrate_[i].child_markers_[j].compare(child_marker) )  // marker frames already exist -> nothing to do here anymore
				{
					return;
				}
			}

			// add missing marker frames
			setup.transforms_to_calibrate_[i].parent_markers_.push_back(parent_marker);
			setup.transforms_to_calibrate_[i].child_markers_.push_back(child_marker);
			return;
		}
	}

	// add new transform to be calibrated
	CalibrationInfo info;
	info.parent_ = parent;
	info.child_ = child;
	info.parent_markers_.push_back(parent_marker);
	info.child_markers_.push_back(child_marker);
	setup.transforms_to_calibrate_.push_back(info);
}

void RobotCalibration::displayAndSaveCalibrationResult(std::string output_file_name)
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
		std::string path_file = calibration_storage_path_ + output_file_name;
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

void RobotCalibration::getTransformByIndex(const int index)
{

}

void RobotCalibration::extrinsicCalibration(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
											std::vector<cv::Mat>& T_gapfirst_to_marker_vector, std::vector< std::vector<cv::Mat> >& T_between_gaps_vector,
											std::vector<cv::Mat>& T_gaplast_to_marker_vector, const CalibrationSetup &setup)
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
			{
				if ( T_child_to_marker.empty() )
					T_child_to_marker = T_between_gaps_vector[i][transforms_to_calibrate_[j].trafo_until_next_gap_idx_].clone();
				else
					T_child_to_marker *= T_between_gaps_vector[i][transforms_to_calibrate_[j].trafo_until_next_gap_idx_];
			}

//std::cout << "forward 1: " << T_child_to_marker.at<double>(0,3) << ", " << T_child_to_marker.at<double>(1,3) << ", " << T_child_to_marker.at<double>(2,3) << std::endl;

			if ( T_child_to_marker.empty() )
				T_child_to_marker = transforms_to_calibrate_[j+1].current_trafo_.clone();
			else
				T_child_to_marker *= transforms_to_calibrate_[j+1].current_trafo_;
//std::cout << "forward: " << transforms_to_calibrate_[j+1].current_trafo_ << std::endl;
//std::cout << "forward 2: " << T_child_to_marker.at<double>(0,3) << ", " << T_child_to_marker.at<double>(1,3) << ", " << T_child_to_marker.at<double>(2,3) << std::endl;
		}

		if ( T_child_to_marker.empty() )
			T_child_to_marker = T_gaplast_to_marker_vector[i].clone();
		else
			T_child_to_marker *= T_gaplast_to_marker_vector[i];

		cv::Mat T_parent_to_marker;
		// Backwards in chain from parent frame on
		for ( int j=trafo_to_calibrate-1; j>=0; --j )
		{
			if ( transforms_to_calibrate_[j].trafo_until_next_gap_idx_ > -1 )
			{
				if ( T_parent_to_marker.empty() )
					T_parent_to_marker = T_between_gaps_vector[i][transforms_to_calibrate_[j].trafo_until_next_gap_idx_].inv();
				else
					T_parent_to_marker *= T_between_gaps_vector[i][transforms_to_calibrate_[j].trafo_until_next_gap_idx_].inv();
			}

//std::cout << "backward 1: " << T_parent_to_marker.at<double>(0,3) << ", " << T_parent_to_marker.at<double>(1,3) << ", " << T_parent_to_marker.at<double>(2,3) << std::endl;

			if ( T_parent_to_marker.empty() )
				T_parent_to_marker = transforms_to_calibrate_[j].current_trafo_.inv();
			else
				T_parent_to_marker *= transforms_to_calibrate_[j].current_trafo_.inv();
//std::cout << "backward: " << transforms_to_calibrate_[j].current_trafo_.inv() << std::endl;
//std::cout << "backward 2: " << T_parent_to_marker.at<double>(0,3) << ", " << T_parent_to_marker.at<double>(1,3) << ", " << T_parent_to_marker.at<double>(2,3) << std::endl;
		}

		if ( T_parent_to_marker.empty() )
			T_parent_to_marker = T_gapfirst_to_marker_vector[i].clone();
		else
			T_parent_to_marker *= T_gapfirst_to_marker_vector[i];

		for (size_t j=0; j<pattern_points_3d[i].size(); ++j)
		{
			cv::Mat point = cv::Mat(cv::Vec4d(pattern_points_3d[i][j].x, pattern_points_3d[i][j].y, pattern_points_3d[i][j].z, 1.0));

			// to child coordinate system
			cv::Mat point_child = T_child_to_marker * point;
//std::cout << "Child to marker: " << transforms_to_calibrate_[trafo_to_calibrate].child_ << ": " << point_child.at<double>(0) << ", " << point_child.at<double>(1) << ", " << point_child.at<double>(2) << std::endl;
			points_3d_child.push_back(cv::Point3d(point_child.at<double>(0), point_child.at<double>(1), point_child.at<double>(2)));

			// to parent coordinate system
			cv::Mat point_parent = T_parent_to_marker * point;
//std::cout << "Parent to marker: " << transforms_to_calibrate_[trafo_to_calibrate].parent_ << ": " << point_parent.at<double>(0) << ", " << point_parent.at<double>(1) << ", " << point_parent.at<double>(2) << std::endl;
			points_3d_parent.push_back(cv::Point3d(point_parent.at<double>(0), point_parent.at<double>(1), point_parent.at<double>(2)));
		}
	}

	transforms_to_calibrate_[trafo_to_calibrate].current_trafo_ = transform_utilities::computeExtrinsicTransform(points_3d_parent, points_3d_child);
}

bool RobotCalibration::calculateTransformationChains(cv::Mat& T_gapfirst_to_marker, std::vector<cv::Mat>& T_between_gaps,
													 cv::Mat& T_gaplast_to_camera_optical, const std::string& marker_frame)
{
	bool result = true;
	result &= transform_utilities::getTransform(transform_listener_, transforms_to_calibrate_[0].parent_, marker_frame, T_gapfirst_to_marker); // from first parent to marker
	result &= transform_utilities::getTransform(transform_listener_, transforms_to_calibrate_[ transforms_to_calibrate_.size()-1 ].child_, camera_optical_frame_, T_gaplast_to_camera_optical); // from last child to camera optical

	/*std::cout << transforms_to_calibrate_[0].parent_ << " to " << marker_frame << ": " << T_gapfirst_to_marker.at<double>(0,3) << ", " << T_gapfirst_to_marker.at<double>(1,3) << ", " << T_gapfirst_to_marker.at<double>(2,3) << std::endl;
	std::cout << transforms_to_calibrate_[ transforms_to_calibrate_.size()-1 ].child_ << " to " << camera_optical_frame_ << ": " << T_gaplast_to_camera_optical.at<double>(0,3) << ", " << T_gaplast_to_camera_optical.at<double>(1,3) << ", " << T_gaplast_to_camera_optical.at<double>(2,3) << std::endl;*/

	for ( int i=0; i<transforms_to_calibrate_.size()-1; ++i )
	{
		if ( transforms_to_calibrate_[i].child_ == transforms_to_calibrate_[i+1].parent_ ) // several gaps in a row, no certain trafos in between
			continue;

		cv::Mat temp;
		bool result_iter = false;

		result_iter = transform_utilities::getTransform(transform_listener_, transforms_to_calibrate_[i].child_, transforms_to_calibrate_[i+1].parent_, temp); // from current child to next parent
		result &= result_iter;

		if ( result_iter )
		{
			T_between_gaps.push_back(temp);
			transforms_to_calibrate_[i].trafo_until_next_gap_idx_ = T_between_gaps.size()-1;
		}
		else
			ROS_WARN("RobotCalibration::calculateTransformationChains: Warning - Could not retrieve trafo from TF: %s to %s. Results might be wrong!", transforms_to_calibrate_[i].child_.c_str(), transforms_to_calibrate_[i+1].parent_.c_str());
	}

	return result;
}
