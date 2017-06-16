#include "Tracker.h"

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
				if (status[i])
				{
					prevCorners.push_back(corners[i]);
				}
			}
		}
	}

	luma.copyTo(prevLuma);
	return isKeyFrame;
}

void Tracker::Track(cv::Mat(&imgs)[6])
{
	bool isKeyFrames[6];

	// TODO parallelize here
	for (auto i = 0; i < 6; ++i)
	{
		isKeyFrames[i] = subTrackers[i].Track(imgs[i]);
	}

	bool hadKeyFrames = isKeyFrames[0] || isKeyFrames[1] || isKeyFrames[2] || isKeyFrames[3] || isKeyFrames[4] || isKeyFrames[5];

	if (hadKeyFrames)
	{
		// TODO stabilize here

        // if not the first key frame
        if(!KeyFrames[1].empty())
        {
            for (int frameNum=0;frameNum<6;frameNum++)
            {
                // set bearing vector of the features as the normalized [u v 1]
                bearingVectors_t bearingVectors1;
                bearingVectors_t bearingVectors2;
                for (int f=0;f<subTrackers[frameNum].prevCorners.size();f++)
                {
                    Eigen::Vector3d present(subTrackers[frameNum].prevCorners.at(f).x,subTrackers[frameNum].prevCorners.at(f).y,1);
                    present= present/present.norm();
                    Eigen::Vector3d previous(trackedFeatures[frameNum].at(0).at(f).x,trackedFeatures[frameNum].at(0).at(f).y,1);
                    previous= previous/previous.norm();
                    bearingVectors1.push_back(previous);
                    bearingVectors2.push_back(present);
                }
                //create a central relative adapter
                relative_pose::CentralRelativeAdapter adapter(bearingVectors1,
                                                              bearingVectors2);
                sac::Ransac<
                sac_problems::relative_pose::CentralRelativePoseSacProblem> ransac;
                std::shared_ptr<
                sac_problems::relative_pose::CentralRelativePoseSacProblem> relposeproblem_ptr(
                                                                                               new sac_problems::relative_pose::CentralRelativePoseSacProblem(
                                                                                                                                                              adapter,
                                                                                                                                                              sac_problems::relative_pose::CentralRelativePoseSacProblem::STEWENIUS));
                ransac.sac_model_ = relposeproblem_ptr;
                ransac.threshold_ = 2.0*(1.0 - cos(atan(sqrt(2.0)*0.5/800.0)));
                ransac.max_iterations_ = 50;
                ransac.computeModel();
                printf("%d:\n",frameNum);
                
                /*** ransac.model_coefficients_ is the result ***/
                std::cout<<ransac.model_coefficients_<<std::endl;

            }
//            imshow("present",imgs[1]);
//            imshow("previous",KeyFrames[1]);
//            cv::waitKey(0);
        }


		// stabilize end

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
