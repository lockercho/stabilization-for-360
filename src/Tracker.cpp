#include "Tracker.h"

#ifndef _WIN32
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <limits.h>
#include <Eigen/Eigen>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/relative_pose/NoncentralRelativeMultiAdapter.hpp>
#include <opengv/sac/MultiRansac.hpp>
#include <opengv/sac_problems/relative_pose/MultiNoncentralRelativePoseSacProblem.hpp>
#include <fstream>
#endif

using namespace Eigen;
using namespace opengv;

bool SubTracker::Track(cv::Mat& img)
{
	cv::Mat yuv;
	cv::cvtColor(img, yuv, cv::COLOR_BGR2YUV);

	cv::Mat chan[3];
	cv::split(yuv, chan);

	cv::Mat luma = chan[0];

	bool isKeyFrame;
	if (prevLuma.empty())
	{
		isKeyFrame = true;
		goodFeaturesToTrack(luma, prevCorners, maxCorners, qualityLevel, minDistance);
	}
	else
	{
		std::vector<cv::Point2f> corners;
		std::vector<uchar> status;
		std::vector<float> errors;

		calcOpticalFlowPyrLK(prevLuma, luma, prevCorners, corners, status, errors);

		prevCorners.clear();
		if (cv::countNonZero(status) < status.size() * 0.5)
		{
			isKeyFrame = true;
			goodFeaturesToTrack(luma, prevCorners, maxCorners, qualityLevel, minDistance);
		}
		else
		{
			isKeyFrame = false;
			for (auto i = 0; i < status.size(); ++i)
			{
				prevCorners.push_back(corners.at(i));
			}
		}
	}

	luma.copyTo(prevLuma);
	return isKeyFrame;
}

opengv::transformation_t Tracker::GetKeyframeRotation(std::vector<std::vector<cv::Point2f>> features, int begin, int& end, bool recursive)
{
	// set bearing vector of the features as the normalized [u v 1]
	bearingVectors_t bearingVectors1;
	bearingVectors_t bearingVectors2;

	auto previousFeatures = features.at(begin);
	auto presentFeatures = features.at(end);

	auto previousFeaturesCount = previousFeatures.size();
	auto presentFeaturesCount = presentFeatures.size();

	auto featuresCount = std::min(previousFeaturesCount, presentFeaturesCount);

	for (auto p = 0; p < featuresCount; ++p)
	{
		Eigen::Vector3d previous(previousFeatures.at(p).x, previousFeatures.at(p).y, 1);
		previous = previous / previous.norm();

		Eigen::Vector3d present(presentFeatures.at(p).x, presentFeatures.at(p).y, 1);
		present = present / present.norm();

		bearingVectors1.push_back(previous);
		bearingVectors2.push_back(present);
	}

	//create a central relative adapter
	relative_pose::CentralRelativeAdapter adapter(
		bearingVectors1,
		bearingVectors2);

	sac::Ransac<
		sac_problems::relative_pose::CentralRelativePoseSacProblem> ransac;

	std::shared_ptr<
		sac_problems::relative_pose::CentralRelativePoseSacProblem> relposeproblem_ptr(
			new sac_problems::relative_pose::CentralRelativePoseSacProblem(
				adapter,
				sac_problems::relative_pose::CentralRelativePoseSacProblem::STEWENIUS));

	ransac.sac_model_ = relposeproblem_ptr;
	ransac.threshold_ = 2.0 * (1.0 - cos(atan(sqrt(2.0) * 0.5 / 800.0)));
	ransac.max_iterations_ = 50;
	ransac.computeModel();

	/*** ransac.model_coefficients_ is the result ***/

	auto inlierRatio = static_cast<double>(ransac.inliers_.size()) / static_cast<double>(featuresCount);

	if (recursive && inlierRatio < 0.5 && end - begin != 1)
	{
		end = begin + ((end - begin) / 2);
		return GetKeyframeRotation(features, begin, end, recursive);
	}
	else
	{
		return ransac.model_coefficients_;
	}
}

void Tracker::Track(cv::Mat(&imgs)[6])
{
	bool isKeyFrames[6];

	frameCount++;

	for (auto i = 0; i < 6; ++i)
	{
		isKeyFrames[i] = subTrackers[i].Track(imgs[i]);
	}

	bool hadKeyFrames = isKeyFrames[0] || isKeyFrames[1] || isKeyFrames[2] || isKeyFrames[3] || isKeyFrames[4] || isKeyFrames[5];

	if (frameCount > 29.97 * 3) // fps 29.97, 3 seconds
	{
		hadKeyFrames = true;
	}

	if (hadKeyFrames)
	{
		frameCount = 0;
	}

	if (hadKeyFrames)
	{
		// if not the first key frame
		if (!KeyFrames[1].empty())
		{
			std::vector<std::vector<cv::Point2f>> features[6];

			for (int frameNum = 0; frameNum < 6; frameNum++)
			{
				std::vector<std::vector<cv::Point2f>> f(trackedFeatures[frameNum]);

				features[frameNum] = f;

				features[frameNum].push_back(subTrackers[frameNum].prevCorners);
			}

			int begin = 0;
			int end = features[0].size() - 1;

			do
			{
				auto keyloc = end;

				for (int frameNum = 0; frameNum < 6; frameNum++)
				{
					auto e = end;

					GetKeyframeRotation(features[frameNum], begin, e, true);

					if (e <= keyloc)
					{
						keyloc = e;
					}
				}

				for (int frameNum = 0; frameNum < 6; frameNum++)
				{
					auto e = keyloc;
					auto rotation = GetKeyframeRotation(features[frameNum], begin, e, false);

					printf("%d:\n", frameNum);

					std::cout << rotation << std::endl;
				}

				if (keyloc != end)
				{
					begin = keyloc;
				}
				else
				{
					break;
				}
			} while (true);
		}

		for (auto i = 0; i < 6; ++i)
		{
			KeyFrames[i] = imgs[i];
			trackedFeatures[i].clear();
			trackedFeatures[i].push_back(subTrackers[i].prevCorners);
		}
	}
	else
	{
		for (auto i = 0; i < 6; ++i)
		{
			trackedFeatures[i].push_back(subTrackers[i].prevCorners);
		}
	}
}
