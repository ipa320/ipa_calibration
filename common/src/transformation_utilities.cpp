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
 * ROS stack name: squirrel_robotino
 * ROS package name: robotino_calibration
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Richard Bormann, email:richard.bormann@ipa.fhg.de
 *
 * Date of creation: December 2015
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

#include <robotino_calibration/transformation_utilities.h>

// Eigen
//#include <Eigen/Core>

//Exception
#include <tf/exceptions.h>

#include <string>
#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>

namespace transform_utilities
{
	// compute rotation matrix from yaw, pitch, roll
	// (w, p, r) = (yaW, Pitch, Roll) with
	// 1. rotation = yaw around z
	// 2. rotation = pitch around y'
	// 3. rotation = roll around x''
	cv::Mat rotationMatrixFromYPR(double yaw, double pitch, double roll)
	{
		double sy = sin(yaw);
		double cy = cos(yaw);
		double sp = sin(pitch);
		double cp = cos(pitch);
		double sr = sin(roll);
		double cr = cos(roll);
		cv::Mat rotation = (cv::Mat_<double>(3,3) <<
				cp*cy,				-cp*sy,				sp,
				cy*sp*sr + cr*sy,		cr*cy - sp*sr*sy,		-cp*sr,
				sr*sy - cr*cy*sp,		cy*sr + cr*sp*sy,		cp*cr);

		return rotation;
	}

	// computes yaw, pitch, roll angles from rotation matrix rot (can also be a 4x4 transformation matrix with rotation matrix at upper left corner)
	cv::Vec3d YPRFromRotationMatrix(const cv::Mat& rot)
	{
		tf::Matrix3x3 r_mat(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
							rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
							rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

		double yaw, pitch, roll;
		r_mat.getEulerYPR(yaw, pitch, roll, 1);
		return cv::Vec3d(yaw, pitch, roll);

		/*Eigen::Matrix3f rot_eigen;
		for (int i=0; i<3; ++i)
			for (int j=0; j<3; ++j)
				rot_eigen(i,j) = rot.at<double>(i,j);
		Eigen::Vector3f euler_angles = rot_eigen.eulerAngles(2,1,0);
		return cv::Vec3d(euler_angles(0), euler_angles(1), euler_angles(2));*/
	}

	cv::Mat makeTransform(const cv::Mat& R, const cv::Mat& t)
	{
		cv::Mat T = (cv::Mat_<double>(4,4) <<
				R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
				R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
				R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2),
				0., 0., 0., 1);
		return T;
	}

	// Takes a string like "1,1,1,1,1,1" and creates a 4x4 transformation matrix out of it.
	/*bool stringToTransform(const std::string values, cv::Mat& trafo)
	{
		const std::string delimiter = ",";
		size_t npos = 0, opos = 0;
		std::vector<double> nums;

		trafo.release();

		while ( (npos = values.find(delimiter)) != std::string::npos )
		{
			double temp = std::stod(values.substr(opos, npos));
			nums.push_back(temp);
		}

		if ( nums.size() == 6 )
		{
			trafo = makeTransform( rotationMatrixFromYPR(nums[3], nums[4], nums[5]), cv::Mat(cv::Vec3d(nums[0], nums[1], nums[2])));
			return true;
		}
		else
		{
			ROS_WARN("String does not contain amount of values for a transformation (exactly 6 needed).");

			trafo = ( cv::Mat_<double>(4,4) <<
							0., 0., 0., 0.,
							0., 0., 0., 0.,
							0., 0., 0., 0.,
							0., 0., 0., 0.);

			return false;
		}
	}*/

	// computes the transform from source_frame to target_frame (i.e. transform arrow is pointing from source_frame to target_frame)
	bool getTransform(const tf::TransformListener& transform_listener, const std::string& target_frame, const std::string& source_frame, cv::Mat& T, bool check_time)
	{
		try
		{
			tf::StampedTransform Ts;
			transform_listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
			transform_listener.lookupTransform(target_frame, source_frame, ros::Time(0), Ts);

			const double current_time = ros::Time::now().toSec();

			if ( check_time && ( !Ts.stamp_.isValid() || current_time - Ts.stamp_.toSec() > 1.f ) )
				throw tf::TransformException("getTransform: Transform from "+target_frame+" to "+source_frame+" timed out.");

			const tf::Matrix3x3& rot = Ts.getBasis();
			const tf::Vector3& trans = Ts.getOrigin();
			cv::Mat rotcv(3,3,CV_64FC1);
			cv::Mat transcv(3,1,CV_64FC1);
			for (int v=0; v<3; ++v)
				for (int u=0; u<3; ++u)
					rotcv.at<double>(v,u) = rot[v].m_floats[u];
			for (int v=0; v<3; ++v)
				transcv.at<double>(v) = trans.m_floats[v];
			T = makeTransform(rotcv, transcv);
		}
		catch (tf::TransformException& ex)
		{
			ROS_WARN("%s",ex.what());
			return false;
		}

		return true;
	}

	// computes the rigid transform between two sets of corresponding 3d points measured in different coordinate systems
	// the resulting 4x4 transformation matrix converts point coordinates from the target system into the source coordinate system
	cv::Mat computeExtrinsicTransform(const std::vector<cv::Point3d>& points_3d_source, const std::vector<cv::Point3d>& points_3d_target)
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

		return makeTransform(R, t);
	}
}
