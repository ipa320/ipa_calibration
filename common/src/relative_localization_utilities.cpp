/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2015 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
* \note
* Repository name: squirrel_calibration
* \note
* ROS package name: relative_localization
 *
 * \author
 * Author: Richard Bormann
 * \author
 * Supervised by:
 *
 * \date Date of creation: 10.08.2016
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <relative_localization/relative_localization_utilities.h>

namespace RelativeLocalizationUtilities
{
	void fitLine(const std::vector<cv::Point2d>& points, cv::Vec4d& line, const double inlier_ratio, const double success_probability, const double max_inlier_distance, bool draw_from_both_halves_of_point_set)
	{
		const int iterations = (int)(log(1.-success_probability)/log(1.-inlier_ratio*inlier_ratio));
	#ifdef DEBUG_OUTPUT
		std::cout << "fitLine: iterations: " << iterations << std::endl;
	#endif
		const int samples = (int)points.size();

		// RANSAC iterations
		int max_inliers = 0;
		for (int k=0; k<iterations; ++k)
		{
			// draw two different points from samples
			int index1, index2;
			if (draw_from_both_halves_of_point_set == false)
			{
				index1 = rand()%samples;
				index2 = index1;
				while (index2==index1)
					index2 = rand()%samples;
			}
			else
			{
				index1 = rand()%(samples/2);
				index2 = std::min((samples/2)+rand()%(samples/2), samples-1);
			}

			// compute line equation from points: d = n0 * (x - x0)  (x0=point on line, n0=normalized normal on line, d=distance to line, d=0 -> line)
			cv::Point2d x0 = points[index1];	// point on line
			cv::Point2d n0(points[index2].y-points[index1].y, points[index1].x-points[index2].x);	// normal direction on line
			const double n0_length = sqrt(n0.x*n0.x + n0.y*n0.y);
			n0.x /= n0_length; n0.y /= n0_length;
			const double c = -points[index1].x*n0.x - points[index1].y*n0.y;		// distance to line: d = n0*(x-x0) = n0.x*x + n0.y*y + c

			// count inliers
			int inliers = 0;
			for (size_t i=0; i<points.size(); ++i)
				if (fabs(n0.x * points[i].x + n0.y * points[i].y + c) <= max_inlier_distance) // count points that are within a margin around the line
					++inliers;

			// update best model
			if (inliers > max_inliers)
			{
				max_inliers = inliers;
				line = cv::Vec4d(points[index1].x, points[index1].y, n0.x, n0.y);		// [x0, y0, n0.x, n0.y]
			}
		}

	#ifdef DEBUG_OUTPUT
		std::cout << "Ransac line: " << line << std::endl;
	#endif

		// final optimization with least squares fit
		const cv::Point2d n0(line[2], line[3]);
		const double c = -line[0]*n0.x - line[1]*n0.y;
		std::vector<cv::Point2f> inlier_set;
		for (size_t i=0; i<points.size(); ++i)
			if (fabs(n0.x * points[i].x + n0.y * points[i].y + c) <= max_inlier_distance)
				inlier_set.push_back(cv::Point2f(points[i].x, points[i].y));
		cv::Vec4f line_ls;
		cv::fitLine(inlier_set, line_ls, CV_DIST_L2, 0, 0.01, 0.01);	// (vx, vy, x0, y0), where (vx, vy) is a normalized vector collinear to the line and (x0, y0) is a point on the line
		const double length = sqrt(line_ls[0]*line_ls[0]+line_ls[1]*line_ls[1]);
		line = cv::Vec4d(line_ls[2], line_ls[3], line_ls[1]/length, -line_ls[0]/length); // store optimized line and its normal vector

	#ifdef DEBUG_OUTPUT
		std::cout << "Optimized line: " << line << std::endl;
	#endif
	}

	// (npx, npy) = a point on the line
	// (n0x, n0y) = the normalized normal direction
	// (pointx, pointy) = the point, whose distance to the line is sought
	double distanceToLine(const double npx, const double npy, const double n0x, const double n0y, const double pointx, const double pointy)
	{
		return fabs(n0x*(pointx-npx) + n0y*(pointy-npy));
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

	bool getTransform(const tf::TransformListener& transform_listener, const std::string& target_frame, const std::string& source_frame, cv::Mat& T)
	{
		try
		{
			tf::StampedTransform Ts;
			transform_listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
			transform_listener.lookupTransform(target_frame, source_frame, ros::Time(0), Ts);
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

}
