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

//Exception
#include <exception>
#include <tf/exceptions.h>

// File writing
#include <sstream>
#include <fstream>

// Boost
#include <boost/filesystem.hpp>


// ToDo: Implement calibration order mechanics! [Already possible]
// ToDo: Implement snapshot save/load system
// ToDo: Move calibration_utilities to calibration_interface package and merge with interface header
// ToDo: Create custom exception classes for exception handling
// ToDo: Each uncertainty can have its own parent and child branch too! Implement that!


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

	if (calibration_interface_ == 0) // Throw exception, as we need an calibration interface in order to function properly!
	{
		ROS_FATAL("Could not create a calibration interface.");
		throw std::exception();
	}

	node_handle_.param<std::string>("calibration_storage_path", calibration_storage_path_, "/calibration");
	std::cout << "calibration_storage_path: " << calibration_storage_path_ << std::endl;

	// load gaps including its initial values
	std::vector<std::string> uncertainties_list;
	calibration_interface_->getUncertainties(uncertainties_list);

	if ( uncertainties_list.empty() )
	{
		ROS_WARN("Uncertainties list is empty... nothing to do.");
		return;
	}

	if ( uncertainties_list.size() % 6 != 0 )
		ROS_WARN("Size of uncertainsties_list is not a factor of 6: [parent frame, child frame, last parent-branch frame, last child-branch frame, parent marker, child marker]");

	// create calibration setups, check for errors
	for ( int i=0; i<uncertainties_list.size(); i+=6 )
	{
		std::string parent = uncertainties_list[i];  // parent frame of uncertainty
		std::string child = uncertainties_list[i+1];  // child frame of uncertainty
		std::string last_parent_branch_frame = uncertainties_list[i+2];
		std::string last_child_branch_frame = uncertainties_list[i+3];

		bool success = transform_listener_.waitForTransform(parent, child, ros::Time(0), ros::Duration(0.5));
		if ( !success )
		{
			ROS_ERROR("Transform from parent frame %s to child frame %s does not exist, skipping.", parent.c_str(), child.c_str());
			continue;
		}

		success = transform_listener_.waitForTransform(parent, last_parent_branch_frame, ros::Time(0), ros::Duration(0.5));
		if ( !success )
		{
			ROS_ERROR("Transform from parent frame %s to last parent-branch frame %s does not exist, skipping.", parent.c_str(), last_parent_branch_frame.c_str());
			continue;
		}

		success = transform_listener_.waitForTransform(child, last_child_branch_frame, ros::Time(0), ros::Duration(0.5));
		if ( !success )
		{
			ROS_ERROR("Transform from child frame %s to last child-branch frame %s does not exist, skipping.", child.c_str(), last_child_branch_frame.c_str());
			continue;
		}

		std::string actual_parent = "";
		transform_listener_.getParent(child, ros::Time(0), actual_parent);
		if ( actual_parent.compare(parent) == 0 )  // compare actual parent of child with parent user has input
		{
			// get origin of parent and child branch chain, chains with same origin will be stored into one calibration setup
			std::string origin = "";
			if ( getOrigin(last_parent_branch_frame, last_child_branch_frame, origin) )
			{
				std::string parent_marker = uncertainties_list[i+4];
				std::string child_marker = uncertainties_list[i+5];

				bool found = false;
				for ( int j=0; j<calibration_setups_.size(); ++j )  // merge entries that have the origin
				{
					if ( origin.compare(calibration_setups_[j].origin_) == 0 )
					{
						feedCalibrationSetup(calibration_setups_[j], parent, child, parent_marker, child_marker);
						found = true;
						break;
					}
				}

				if ( !found )  // no match has been found -> create new calibration setup
				{
					CalibrationSetup setup;
					setup.origin_ = origin;
					feedCalibrationSetup(setup, parent, child, parent_marker, child_marker);
					calibration_setups_.push_back(setup);
				}
			}
			else
				ROS_WARN("No mutual origin found, skipping uncertainty from %s to %s.", parent.c_str(), child.c_str());
		}
		else
			ROS_WARN("Given parent frame does not match actual parent (%s) in tf tree. Given parent: %s, given child: %s", actual_parent.c_str(), uncertainties_list[i].c_str(), child.c_str());
	}

	// remove all corrupted entries in calibration setups vector
	for ( int i=0; i<calibration_setups_.size(); ++i )
	{
		if ( calibration_setups_[i].uncertainties_list_.size() == 0 )  // remove corrupted setups
		{
			ROS_WARN("No transform to calibrate found in calibration setup -> removing setup.");
			calibration_setups_.erase(calibration_setups_.begin()+i);
			--i;
			continue;
		}

		for ( int j=0; j<calibration_setups_[i].uncertainties_list_.size(); ++j )  // check for corrupted transforms
		{
			if ( calibration_setups_[i].uncertainties_list_[j].parent_markers_.size() == 0 ||
					calibration_setups_[i].uncertainties_list_[j].child_markers_.size() == 0 )
			{
				ROS_WARN("Empty parent or child_markers for %s to %s -> removing transform", calibration_setups_[i].uncertainties_list_[j].parent_.c_str(), calibration_setups_[i].uncertainties_list_[j].child_.c_str());
				calibration_setups_[i].uncertainties_list_.erase(calibration_setups_[i].uncertainties_list_.begin()+j);
			}
		}

		if ( calibration_setups_[i].uncertainties_list_.size() == 0 )  // check again
		{
			ROS_WARN("No transform to calibrate found in calibration setup -> removing setup.");
			calibration_setups_.erase(calibration_setups_.begin()+i);
			--i;
		}
	}

	// ToDo: Feed calibration setup with each last_parent_branch_frame and last_child_branch_frame as well.
	// This is because each calibration setup can have several parent and child branches. Snapshots have to consider this as well!!!

	// organize/sort calibration data
	// the uncertainties_list_ vector for each setup is not useful for a calibration, as it is unsorted. Sort it now to make the process feasible.
	for ( int i=0; i<calibration_setups_.size(); ++i )
	{
		// 1. Retrieve the reversed parent and child branch (from last parent/child branch frame to their mutual origin)
		std::vector<std::string> parent_branch_reversed;
		std::vector<std::string> child_branch_reversed;
		std::string last_parent_branch_frame = uncertainties_list[i+2];
		std::string last_child_branch_frame = uncertainties_list[i+3];

		// retrieve backwards chains, that connect last_parent_branch_frame/last_child_branch_frame with its origin.
		bool success = getBackChain(last_parent_branch_frame, calibration_setups_[i].origin_, parent_branch_reversed);
		success &= getBackChain(last_child_branch_frame, calibration_setups_[i].origin_, child_branch_reversed);

		if ( !success )
			continue;

		std::cout << "FINDING RIGHT ORDER" << std::endl;

		// 2. Find natural order of the frames to be calibrated within the parent branch and child branch
		// search trafos_to_calibrate inside parent_branch_reversed backwards to iterate in natural order (from origin to last_parent_branch_frame)
		for ( int j=parent_branch_reversed.size()-1; j>=0; --j )
		{
			calibration_setups_[i].parent_branch.push_back(parent_branch_reversed[j]);  // store parent branch in right order for later usage: from origin to last_parent_branch_frame

			if ( j > 0 )  // don't execute in last iteration
			{
				for ( int k=0; k<calibration_setups_[i].uncertainties_list_.size(); ++k )
				{
					std::cout << "P_PARENT: " << parent_branch_reversed[j] << " to " << calibration_setups_[i].uncertainties_list_[k].parent_ << std::endl;
					std::cout << "P_CHILD: " << parent_branch_reversed[j-1] << " to " << calibration_setups_[i].uncertainties_list_[k].child_ << std::endl;

					if ( parent_branch_reversed[j].compare(calibration_setups_[i].uncertainties_list_[k].parent_) == 0 &&
							parent_branch_reversed[j-1].compare(calibration_setups_[i].uncertainties_list_[k].child_) == 0 )
					{
						CalibrationInfo &info = calibration_setups_[i].uncertainties_list_[k];
						calibration_setups_[i].origin_to_parent_marker_uncertainties_.push_back(info);  // now put the transforms in order, so that they reflect the order in which they appear in tf
						std::cout << "P_PARENT: " << info.parent_ << "P_CHILD: " << info.child_ << std::endl;
					}
				}
			}
		}

		// search trafos_to_calibrate inside child_branch_reversed backwards to iterate in natural order (from origin to last_child_branch_frame)
		for ( int j=child_branch_reversed.size()-1; j>=0; --j )
		{
			calibration_setups_[i].child_branch.push_back(child_branch_reversed[j]);  // store child branch in right order for later usage: from origin to last_child_branch_frame

			if ( j > 0 )  // don't execute in last execution
			{
				for ( int k=0; k<calibration_setups_[i].uncertainties_list_.size(); ++k )
				{
					std::cout << "C_PARENT: " << child_branch_reversed[j] << " to " << calibration_setups_[i].uncertainties_list_[k].parent_ << std::endl;
					std::cout << "C_CHILD: " << child_branch_reversed[j-1] << " to " << calibration_setups_[i].uncertainties_list_[k].child_ << std::endl;

					if ( child_branch_reversed[j].compare(calibration_setups_[i].uncertainties_list_[k].parent_) == 0 &&
							child_branch_reversed[j-1].compare(calibration_setups_[i].uncertainties_list_[k].child_) == 0 )
					{
						CalibrationInfo &info = calibration_setups_[i].uncertainties_list_[k];
						calibration_setups_[i].origin_to_child_marker_uncertainties_.push_back(info);  // now put the transforms in order, so that they reflect the order in which they appear in tf
						std::cout << "C_PARENT: " << info.parent_ << " C_CHILD: " << info.child_ << std::endl;
					}
				}
			}
		}
	}

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

	std::cout << "optimization_iterations: " << optimization_iterations_ << std::endl;

	createStorageFolder();
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
			ROS_ERROR("RobotCalibration: Could not create directory %s ", storage_path.c_str());
			return;
		}
	}
}

bool RobotCalibration::getOrigin(const std::string last_parent_branch_frame, const std::string last_child_branch_frame, std::string &origin)
{
	if ( last_parent_branch_frame.empty() || last_child_branch_frame.empty() )
	{
		ROS_ERROR("Invalid last parent-branch frame or last child-branch frame!");
		return false;
	}

	std::vector<std::string> parent_frames;  // from last_parent_branch_frame to last available frame
	std::string frame = last_parent_branch_frame;

	do
	{
		parent_frames.push_back(frame);  // populate backwards chain
	}
	while ( transform_listener_.getParent(frame, ros::Time(0), frame) );

	// check for occurrence of first frame of child-branch in parent-branch
	frame = last_child_branch_frame;
	do
	{
		for ( int i=0; i<parent_frames.size(); ++i )
		{
			if ( frame.compare(parent_frames[i]) == 0 )  // find frame where parent branch and child branch meets
			{
				origin = frame;
				return true;
			}
		}
	}
	while ( transform_listener_.getParent(frame, ros::Time(0), frame) );  // go back from last_child_branch_frame on and see where it meets with parent_frames

	ROS_WARN("No mutual origin found between %s and %s.", last_parent_branch_frame.c_str(), last_child_branch_frame.c_str());
	return false;
}

void RobotCalibration::feedCalibrationSetup(CalibrationSetup &setup, const std::string parent, const std::string child,
		const std::string parent_marker, const std::string child_marker)
{
	// first check if transform already exists, if so extend it
	for ( int i=0; i<setup.uncertainties_list_.size(); ++i )
	{
		if ( setup.uncertainties_list_[i].parent_.compare(parent) == 0 &&
				setup.uncertainties_list_[i].child_.compare(child) == 0 )  // transform already exists, check if we have to extend its marker frames
		{
			for ( int j=0; i<setup.uncertainties_list_[i].parent_markers_.size(); ++j )  // parent_markers and child_markers always have same size
			{
				if ( setup.uncertainties_list_[i].parent_markers_[j].compare(parent_marker) &&
						setup.uncertainties_list_[i].child_markers_[j].compare(child_marker) )  // marker frames already exist -> nothing to do here anymore
				{
					return;
				}
			}

			// add missing marker frames
			setup.uncertainties_list_[i].parent_markers_.push_back(parent_marker);
			setup.uncertainties_list_[i].child_markers_.push_back(child_marker);
			return;
		}
	}

	// add new transform to be calibrated
	CalibrationInfo info;

	bool success = transform_utilities::getTransform(transform_listener_, parent, child, info.current_trafo_);  // init uncertain trafo with what's in tf

	if ( success )
	{
		info.parent_ = parent;
		info.child_ = child;
		info.parent_markers_.push_back(parent_marker);
		info.child_markers_.push_back(child_marker);
		setup.uncertainties_list_.push_back(info);
	}
}

bool RobotCalibration::getBackChain(const std::string frame_start, const std::string frame_end, std::vector<std::string> &backchain)
{
	if ( frame_end.compare(frame_start) != 0 )  // do not execute if end frame and start frame are equal
	{
		std::string frame = frame_start;

		do
		{
			backchain.push_back(frame);  // populate backwards chain

			if ( !transform_listener_.getParent(frame, ros::Time(0), frame) )
			{
				ROS_WARN("Could not create back chain for frame %s, no parent frame for %s!", frame_start.c_str(), frame.c_str());
				backchain.clear();
				return false;
			}
		}
		while ( frame.compare(frame_end) != 0 );
	}
	backchain.push_back(frame_end);  // add origin as last frame to parent branch

	return true;
}


bool RobotCalibration::startCalibration(const bool load_data_from_drive)
{
	if ( calibration_interface_ == 0 ) // Throw exception, as we need an calibration interface in order to function properly!
	{
		ROS_FATAL("Calibration interface has not been set up!");
		throw std::exception();
	}

	if ( calibration_setups_.size() == 0 )
		return false;

	if ( !acquireTFData(load_data_from_drive) )  // make snapshots of all relevant tf transforms for every robot configuration
		return false;

	// extrinsic calibration optimization
	for ( int l=0; l<calibration_setups_.size(); ++l )
	{
		int iterations = 0;

		// if there is only one trafo to calibrate, we don't need to optimize over iterations
		if ( calibration_setups_[l].uncertainties_list_.size() > 1 )
			iterations = optimization_iterations_;
		else
			iterations = 1;

		for (int i=0; i<iterations; ++i)
		{
			for ( int j=0; j<calibration_setups_[l].uncertainties_list_.size(); ++j )
			{
				// ToDo: HERE NEEDS TO BE AN ORDER IN PLACE SO USER CAN DEFINE IT!
				if ( !extrinsicCalibration(calibration_setups_[l], j) )
				{
					ROS_ERROR("Calibration failed!");
					return false;
				}
			}
		}

		calibration_setups_[l].calibrated_ = true;  // setup has been calibrated
	}

	// display and save calibration parameters
	std::string output = calibration_interface_->getResultFileName();

	// check for file extension
	if( output.rfind(".") == std::string::npos )
	{
		output += ".txt";
	}

	RobotCalibration::displayAndSaveCalibrationResult(output);

	calibrated_ = true;
	return true;
}

bool RobotCalibration::acquireTFData(const bool load_data)
{
	//std::stringstream path;
	//path << calibration_storage_path_ << "pitag_data.yml";  // retrieve from interface instead!

	if ( load_data == false )
	{
		const int num_configs = calibration_interface_->getConfigurationCount();
		for ( int config_counter = 0; config_counter < num_configs; ++config_counter )
		{
			if ( !ros::ok() )
				return false;

			std::cout << "Configuration " << (config_counter+1) << "/" << num_configs << std::endl;

			// try to move robot
			try
			{
				if ( !calibration_interface_->moveRobot(config_counter) )
					continue;
			}
			catch( std::exception &ex )
			{
				return false;
			}

			// wait a moment here to mitigate shaking camera effects and give tf time to update
			ros::Duration(1).sleep();

			calibration_interface_->preSnapshot(config_counter);  // give user possibility to execute code before snapshots take place

			// grab transforms for each setup and store them
			for ( int i=0; i<calibration_setups_.size(); ++i )
			{
				// snapshot necessary tf data: 1) all transforms in parent and child branch, 2) all parent and child marker transforms for each uncertain trafo
				populateTFSnapshot(calibration_setups_[i]);
			}
		}

		// ToDo: Save snapshots to drive for offline calibration
	}
	else
	{
		// ToDo: Ask whether everything has to be stored
		// load data from file
		/*cv::FileStorage fs(path.str().c_str(), cv::FileStorage::READ);
		// discuss this functionality with Rirchard, would be hard to implement due to complex structs that have to be stored and loaded
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

		fs.release();*/
	}

	return true;
}

void RobotCalibration::populateTFSnapshot(const CalibrationSetup &setup)
{
	// populate parent branch trafos
	TFSnapshot snapshot;
	cv::Mat trafo;
	const double timeout = 1.0;
	for ( int i=0; i<setup.parent_branch.size()-1; ++i )
	{
		if ( transform_utilities::getTransform(transform_listener_, setup.parent_branch[i], setup.parent_branch[i+1], trafo, timeout, true) )
		{
			TFInfo info;
			info.parent_ = setup.parent_branch[i];
			info.child_ = setup.parent_branch[i+1];
			info.transform_ = trafo.clone();
			snapshot.parent_branch_.push_back(info);
		}
		else
		{
			ROS_ERROR("Could not build parent branch, skipping to snapshot current configuration!");
			return;
		}
	}

	// populate child branch trafos
	for ( int i=0; i<setup.child_branch.size()-1; ++i )
	{
		if ( transform_utilities::getTransform(transform_listener_, setup.child_branch[i], setup.child_branch[i+1], trafo, timeout, true) )
		{
			TFInfo info;
			info.parent_ = setup.child_branch[i];
			info.child_ = setup.child_branch[i+1];
			info.transform_ = trafo.clone();
			snapshot.child_branch_.push_back(info);
		}
		else
		{
			ROS_ERROR("Could not build parent branch, skipping to snapshot current configuration!");
			return;
		}
	}

	// populate parent and child markers of parent branch
	std::string last_parent_branch_frame = setup.parent_branch[setup.parent_branch.size()-1];
	std::string last_child_branch_frame = setup.child_branch[setup.child_branch.size()-1];
	for ( int i=0; i<setup.origin_to_parent_marker_uncertainties_.size(); ++i )
	{
		std::vector<TFInfo> parent_markers;  // store parent markers of parent branch
		for ( std::string parent_marker : setup.origin_to_parent_marker_uncertainties_[i].parent_markers_ )
		{
			TFInfo info;
			if ( transform_utilities::getTransform(transform_listener_, last_parent_branch_frame, parent_marker, trafo, timeout) )
			{
				info.parent_ = last_parent_branch_frame;
				info.child_ = parent_marker;
				info.transform_ = trafo.clone();
				parent_markers.push_back(info);
			}

			// Even store info if it is empty (e.g. due to timeout), so parent_markers size is the same as setup.origin_to_parent_marker_uncertainties_[i].parent_markers_
			parent_markers.push_back(info);
		}

		std::vector<TFInfo> child_markers;  // store child markers of parent branch
		for ( std::string child_marker : setup.origin_to_parent_marker_uncertainties_[i].child_markers_ )
		{
			TFInfo info;
			if ( transform_utilities::getTransform(transform_listener_, last_child_branch_frame, child_marker, trafo, timeout) )
			{
				info.parent_ = last_child_branch_frame;
				info.child_ = child_marker;
				info.transform_ = trafo.clone();
			}

			// Also store empty info so that child_markers and parent_markers have same size, this way we can filter out bad entries easily
			child_markers.push_back(info);
		}

		// check for bad entries now
		for ( int j=0; j<parent_markers.size(); ++j )  // parent_markers and child_markers have same size
		{
			if ( parent_markers[j].transform_.empty() || child_markers[j].transform_.empty() )  // remove bad entries from both lists, so that both vectors always have same size
			{
				parent_markers.erase(parent_markers.begin()+j);
				child_markers.erase(child_markers.begin()+j);
			}
		}

		if ( parent_markers.size() > 0 && parent_markers.size() == child_markers.size() )
		{
			snapshot.parent_branch_parent_markers_.push_back(parent_markers);
			snapshot.parent_branch_child_markers_.push_back(child_markers);
		}
		else
		{
			ROS_ERROR("Parent branch uncertainties: Parent marker vector is empty has not same size as child marker vector, skipping to snapshot current configuration!");
			return;
		}
	}

	// populate parent and child markers of child branch
	for ( int i=0; i<setup.origin_to_child_marker_uncertainties_.size(); ++i )
	{
		std::vector<TFInfo> parent_markers;  // store parent markers of child branch
		for ( std::string parent_marker : setup.origin_to_child_marker_uncertainties_[i].parent_markers_ )
		{
			TFInfo info;
			if ( transform_utilities::getTransform(transform_listener_, last_parent_branch_frame, parent_marker, trafo, timeout) )
			{
				info.parent_ = last_parent_branch_frame;
				info.child_ = parent_marker;
				info.transform_ = trafo.clone();
			}

			parent_markers.push_back(info);
		}

		std::vector<TFInfo> child_markers;  // store child markers of child branch
		for ( std::string child_marker : setup.origin_to_child_marker_uncertainties_[i].child_markers_ )
		{
			TFInfo info;
			if ( transform_utilities::getTransform(transform_listener_, last_child_branch_frame, child_marker, trafo, timeout) )
			{
				info.parent_ = last_child_branch_frame;
				info.child_ = child_marker;
				info.transform_ = trafo.clone();
			}

			child_markers.push_back(info);
		}

		// check for bad entries now
		for ( int j=0; j<parent_markers.size(); ++j )  // parent_markers and child_markers have same size
		{
			if ( parent_markers[j].transform_.empty() || child_markers[j].transform_.empty() )  // remove bad entries from both lists, so that both vectors always have same size
			{
				parent_markers.erase(parent_markers.begin()+j);
				child_markers.erase(child_markers.begin()+j);
			}
		}

		if ( parent_markers.size() > 0 && parent_markers.size() == child_markers.size() )
		{
			snapshot.child_branch_parent_markers_.push_back(parent_markers);
			snapshot.child_branch_child_markers_.push_back(child_markers);
		}
		else
		{
			ROS_ERROR("Child branch uncertainties: Parent marker vector is empty or has not same size as child marker vector, skipping to snapshot current configuration!");
			return;
		}
	}

	tf_snapshots.push_back(snapshot);
}

void RobotCalibration::displayAndSaveCalibrationResult(std::string output_file_name)
{
	std::stringstream output;

	output << "\n\n\n----- Replace the follwing parameters within the urdf file of your robot ----- \n\n";
	for ( int i=0; i<calibration_setups_.size(); ++i )
	{
		for ( int j=0; j<calibration_setups_[i].uncertainties_list_.size(); ++j )
		{
			cv::Vec3d ypr = transform_utilities::YPRFromRotationMatrix(calibration_setups_[i].uncertainties_list_[j].current_trafo_);

			output << "<!-- " << calibration_setups_[i].uncertainties_list_[j].child_ << " mount positions | camera_base_calibration | relative to " << calibration_setups_[i].uncertainties_list_[j].parent_ << "-->\n"
				   << "  <property name=\"" << calibration_setups_[i].uncertainties_list_[j].child_ << "_x\" value=\"" << calibration_setups_[i].uncertainties_list_[j].current_trafo_.at<double>(0,3) << "\"/>\n"
				   << "  <property name=\"" << calibration_setups_[i].uncertainties_list_[j].child_ << "_y\" value=\"" << calibration_setups_[i].uncertainties_list_[j].current_trafo_.at<double>(1,3) << "\"/>\n"
				   << "  <property name=\"" << calibration_setups_[i].uncertainties_list_[j].child_ << "_z\" value=\"" << calibration_setups_[i].uncertainties_list_[j].current_trafo_.at<double>(2,3) << "\"/>\n"
				   << "  <property name=\"" << calibration_setups_[i].uncertainties_list_[j].child_ << "_roll\" value=\"" << ypr.val[2] << "\"/>\n"
				   << "  <property name=\"" << calibration_setups_[i].uncertainties_list_[j].child_ << "_pitch\" value=\"" << ypr.val[1] << "\"/>\n"
				   << "  <property name=\"" << calibration_setups_[i].uncertainties_list_[j].child_ << "_yaw\" value=\"" << ypr.val[0] << "\"/>\n\n";
		}
	}

	std::cout << output.str();

	if ( ros::ok() )
	{
		std::string path_file = calibration_storage_path_ + output_file_name;
		std::fstream file_output;
		file_output.open(path_file.c_str(), std::ios::out);
		if (file_output.is_open())
			file_output << output.str();
		else
			ROS_WARN("Failed to open %s, not saving calibration results!", path_file.c_str());
		file_output.close();
	}
	else
	{
		ROS_WARN("Not saving calibration results.");
	}
}

bool RobotCalibration::extrinsicCalibration(CalibrationSetup &setup, const int current_uncertainty_idx)
{
	// find in which branch the current uncertainty resides, build forward and backward chain based on that
	int branch_idx = -1;

	CalibrationInfo &current_uncertainty = setup.uncertainties_list_[current_uncertainty_idx];
	bool on_parent_branch = isParentBranchUncertainty(setup, current_uncertainty_idx, branch_idx);
	if ( branch_idx > -1 )
	{
		std::vector<cv::Point3d> points_3d_uncertainty_parent;  // parent marker points in uncertainty parent frame
		std::vector<cv::Point3d> points_3d_uncertainty_child;  // child marker points in uncertainty child frame

		for ( int i=0; i<tf_snapshots.size(); ++i )  // go through all snapshots for this setup and build
		{
			bool success = true;
			std::vector<TFInfo>* branch_tmp;  			// either parent or child_branch
			std::vector<TFInfo>* other_branch_tmp;		// if branch = child_branch then this is parent_branch and vice versa
			std::vector<TFInfo>* parent_markers_tmp;	// points to markers of parent of current uncertainty
			std::vector<TFInfo>* child_markers_tmp;		// points to markers of child of current uncertainty
			if ( on_parent_branch )
			{
				branch_tmp = &tf_snapshots[i].parent_branch_;
				other_branch_tmp = &tf_snapshots[i].child_branch_;

				// Why can we use branch_idx when it is actually the index for setup.origin_to_parent_marker_uncertainties_ here?
				// Because for each uncertainty there will be a snapshot created of its own markers, therefore setup.origin_to_parent_marker_uncertainties_
				// and tf_snapshots[i].parent_branch_parent_markers_ have the same size and have the same order (check populateTFSnapshot())
				parent_markers_tmp = &tf_snapshots[i].parent_branch_parent_markers_[branch_idx];
				child_markers_tmp = &tf_snapshots[i].parent_branch_child_markers_[branch_idx];
			}
			else
			{
				branch_tmp = &tf_snapshots[i].child_branch_;
				other_branch_tmp = &tf_snapshots[i].parent_branch_;
				parent_markers_tmp = &tf_snapshots[i].child_branch_parent_markers_[branch_idx];
				child_markers_tmp = &tf_snapshots[i].child_branch_child_markers_[branch_idx];
			}

			// now with references
			std::vector<TFInfo> &branch = *branch_tmp;
			std::vector<TFInfo> &other_branch = *other_branch_tmp;
			std::vector<TFInfo> &parent_markers = *parent_markers_tmp;
			std::vector<TFInfo> &child_markers = *child_markers_tmp;

			if ( parent_markers.size() != child_markers.size() )
			{
				ROS_WARN("Parent markers and child markers vector do not have the same size for snapshot %d, skipping.", i);
				continue;
			}

			cv::Mat uncertainty_child_to_premarker;  // uncertainty child to transform before child markers
			cv::Mat uncertainty_parent_to_origin;  // uncertainty parent to origin
			cv::Mat origin_to_premarker;  // origin to transform before parent markers

			// build transform chains
			if ( branch.size() > 0 )
			{
				TFInfo last_trafo = branch[branch.size()-1];
				success &= buildTransformChain(current_uncertainty.child_, last_trafo.child_, branch, uncertainty_child_to_premarker);
			}
			else
			{
				ROS_ERROR("%s vector is empty, skipping snapshot %d.", (on_parent_branch ? "child branch" : "parent branch"), i );
				continue;
			}

			success &= buildTransformChain(current_uncertainty.parent_, setup.origin_, branch, uncertainty_parent_to_origin);

			if ( other_branch.size() > 0 )
			{
				TFInfo last_trafo = other_branch[other_branch.size()-1];
				success &= buildTransformChain(setup.origin_, last_trafo.child_, other_branch, origin_to_premarker);
			}
			else
			{
				ROS_ERROR("%s vector is empty, skipping snapshot %d.", (on_parent_branch ? "child branch" : "parent branch"), i );
				continue;
			}

			if ( !success )
			{
				ROS_ERROR("Failed to build one or more necessary transforms, skipping snapshot %d.", i);
				continue;
			}

			// build marker points for extrinsic calibration of uncertainty parent
			for ( int j=0; j<parent_markers.size(); ++j )
			{
				std::string parent_marker_frame = parent_markers[j].child_;
				std::vector<cv::Point3f> pattern_points_3d;
				calibration_interface_->getPatternPoints3D(parent_marker_frame, pattern_points_3d);  // get pattern points of current marker

				cv::Mat uncertainty_parent_to_marker = uncertainty_parent_to_origin * origin_to_premarker * parent_markers[j].transform_;
				for ( int k=0; k<pattern_points_3d.size(); ++k )
				{
					cv::Mat marker_point = cv::Mat(cv::Vec4d(pattern_points_3d[k].x, pattern_points_3d[k].y, pattern_points_3d[k].z, 1.0));
					cv::Mat point_parent = uncertainty_parent_to_marker * marker_point;
					points_3d_uncertainty_parent.push_back( cv::Point3d(point_parent.at<double>(0), point_parent.at<double>(1), point_parent.at<double>(2)) );
				}
			}

			// build marker points for extrinsic calibration of uncertainty child
			for ( int j=0; j<child_markers.size(); ++j )
			{
				std::string child_marker_frame = child_markers[j].child_;
				std::vector<cv::Point3f> pattern_points_3d;
				calibration_interface_->getPatternPoints3D(child_marker_frame, pattern_points_3d);  // get pattern points of current marker

				cv::Mat uncertainty_child_to_marker = uncertainty_child_to_premarker * child_markers[j].transform_;
				for ( int k=0; k<pattern_points_3d.size(); ++k )
				{
					cv::Mat marker_point = cv::Mat(cv::Vec4d(pattern_points_3d[k].x, pattern_points_3d[k].y, pattern_points_3d[k].z, 1.0));
					cv::Mat point_child = uncertainty_child_to_marker * marker_point;
					points_3d_uncertainty_child.push_back( cv::Point3d(point_child.at<double>(0), point_child.at<double>(1), point_child.at<double>(2)) );
				}
			}
		}

		// compute extrinsic transform
		if ( points_3d_uncertainty_parent.size() == points_3d_uncertainty_child.size() )
		{
			current_uncertainty.current_trafo_ = transform_utilities::computeExtrinsicTransform(points_3d_uncertainty_parent, points_3d_uncertainty_child);
			return true;
		}
		else
		{
			ROS_ERROR("Uncertainty points vectors do not have same size, transform from %s to %s not calibrated, exiting!", current_uncertainty.parent_.c_str(), current_uncertainty.child_.c_str());
		}
	}
	else
	{
		ROS_ERROR("Cannot find current uncertainty (from %s to %s) in neither parent nor child branch, exiting!", current_uncertainty.parent_.c_str(), current_uncertainty.child_.c_str());
	}

	return false;
}

bool RobotCalibration::isParentBranchUncertainty(const CalibrationSetup &setup, const int uncertainty_idx, int &branch_idx)
{
	CalibrationInfo uncertainty = setup.uncertainties_list_[uncertainty_idx];
	branch_idx = -1;
	for ( int i=0; i<setup.origin_to_parent_marker_uncertainties_.size(); ++i )  // search in parent branch for uncertainty
	{
		CalibrationInfo current = setup.origin_to_parent_marker_uncertainties_[i];
		if( uncertainty.parent_.compare(current.parent_) == 0 &&
				uncertainty.child_.compare(current.child_) == 0 )
		{
			branch_idx = i;
			return true;
		}
	}

	for ( int i=0; i<setup.origin_to_child_marker_uncertainties_.size(); ++i )  // search in child branch for uncertainty
	{
		CalibrationInfo current = setup.origin_to_child_marker_uncertainties_[i];
		if( uncertainty.parent_.compare(current.parent_) == 0 &&
				uncertainty.child_.compare(current.child_) == 0 )
		{
			branch_idx = i;
			return false;
		}
	}

	return false;
}

bool RobotCalibration::buildTransformChain(const std::string start, const std::string end, const std::vector<TFInfo> &branch, cv::Mat &trafo)  // get transform chain
{
	// start and end are not necessarily next to one another
	int start_idx = -1;
	int end_idx = -1;
	for ( int i=0; i<branch.size(); ++i )  // find indexes of start and end point
	{
		if ( start.compare(branch[i].parent_) == 0 )  // only use parents to determine
			start_idx = i;

		if ( end.compare(branch[i].parent_) == 0 )
			end_idx = i;
	}

	if ( start_idx < 0 || end_idx < 0 )  // invalid index
	{
		ROS_WARN("Could not build transform chain from %s to %s in current snapshot", start.c_str(), end.c_str());
		return false;
	}
	else if ( start_idx < end_idx )  // normal forward transform
	{
		for ( int i=start_idx; i<end_idx; ++i )  // retrieve trafo from start_idx to end_idx one by one
		{
			cv:: Mat temp;

			if ( retrieveTransform(branch[i].parent_, branch[i].child_, branch, temp) )
			{
				if ( trafo.empty() )
					trafo = temp;
				else
					trafo *= temp;
			}
			else
				return false;
		}
	}
	else if ( start_idx > end_idx )  // return inverse
	{
		for ( int i=end_idx-1; i>=start_idx; --i )  // retrieve trafo from end_idx to start_idx one by one
		{
			cv:: Mat temp;

			if ( retrieveTransform(branch[i].child_, branch[i].parent_, branch, temp) )
			{
				if ( trafo.empty() )
					trafo = temp;
				else
					trafo *= temp;
			}
			else
				return false;
		}
	}
	else  // start_idx == end_idx -> return [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 0]
	{
		trafo = cv::Mat::eye(4,4,CV_64FC1);
		trafo.at<double>(4,4) = 0.0;
		return true;
	}

	return false;
}

bool RobotCalibration::retrieveTransform(const std::string parent, const std::string child, const std::vector<TFInfo> &branch, cv::Mat &trafo)  // get the next trafo
{
	// search whether it is a uncertainty and if it is calibrated. in that case return it
	for ( int i=0; i<calibration_setups_.size(); ++i )
	{
		for ( int j=0; j<calibration_setups_[i].uncertainties_list_.size(); ++j )
		{
			if ( calibration_setups_[i].calibrated_ )
			{
				if ( calibration_setups_[i].uncertainties_list_[j].child_.compare(child) == 0 &&
						calibration_setups_[i].uncertainties_list_[j].parent_.compare(parent) == 0 )
				{
					trafo = calibration_setups_[i].uncertainties_list_[j].current_trafo_.clone();
					return true;
				}
				else if ( calibration_setups_[i].uncertainties_list_[j].child_.compare(parent) == 0 &&
						calibration_setups_[i].uncertainties_list_[j].parent_.compare(child) == 0 )  // return inverse
				{
					trafo = calibration_setups_[i].uncertainties_list_[j].current_trafo_.inv();
					return true;
				}
			}
		}
	}

	// in a second step search through the snapshotted branch and return the corresponding transform
	for ( int i=0; i<branch.size(); ++i )
	{
		if ( branch[i].child_.compare(child) && branch[i].parent_.compare(parent) == 0 )  // in right order
		{
			trafo = branch[i].transform_.clone();
			return true;
		}
		else if ( branch[i].child_.compare(parent) && branch[i].parent_.compare(child) == 0 )  // order swapped -> inverse
		{
			trafo = branch[i].transform_.inv();
			return true;
		}
	}

	ROS_WARN("Could not retrieve transform from %s to %s in current snapshot", parent.c_str(), child.c_str());
	return false;
}

