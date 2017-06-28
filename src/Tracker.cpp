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
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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
            prevFrame = luma;
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
            currFrame = luma;
		}
	}

	luma.copyTo(prevLuma);
	return isKeyFrame;
}


void plotMatches(cv::Mat& img1, cv::Mat& img2, std::vector<cv::Point2f>& p1, std::vector<cv::Point2f>& p2) {
    
    // write to file to debug
    cv::imwrite("/Users/lockercho/workspace/GPU/final_project/test/prevImg.jpg", img1);
    cv::imwrite("/Users/lockercho/workspace/GPU/final_project/test/currImg.jpg", img2);
    
    //
    FILE * f1 = fopen("/Users/lockercho/workspace/GPU/final_project/test/prevKey.txt", "w");
    FILE * f2 = fopen("/Users/lockercho/workspace/GPU/final_project/test/currKey.txt", "w");
    
    for(int i=0 ; i<p1.size() ; i++) {
        fprintf(f1, "%f %f\n", p1[i].x, p1[i].y);
        fprintf(f2, "%f %f\n", p2[i].x, p2[i].y);
    }
    
    fclose(f1);
    fclose(f2);
    
//    cv::Mat out;
//    std::vector<cv::DMatch> matches;
//    std::vector<cv::KeyPoint> k1;
//    std::vector<cv::KeyPoint> k2;
//    for(int i=0 ; i<p1.size() ; i++) {
//        cv::DMatch v = cv::DMatch(i, i, 0);
//        matches.push_back(v);
//        
//        k1.push_back(cv::KeyPoint(p1[i], 1.f));
//        k2.push_back(cv::KeyPoint(p2[i], 1.f));
//        
//    }
    
//    cv::drawMatches(img1, k1, img2, k2, matches, out);
//    cv::namedWindow("matches", 1);
//    cv::imshow("matches", out);
//    cv::waitKey(0);
}

opengv::transformation_t Tracker::GetKeyframeRotation(std::vector<std::vector<cv::Point2f>> features, int begin, int& end, bool recursive, std::vector<cv::Mat>& prevframes, std::vector<cv::Mat>& currframes)
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
		return GetKeyframeRotation(features, begin, end, recursive, prevframes, currframes);
	}
	else
	{
//        plotMatches(prevframes[0], currframes[0], previousFeatures, presentFeatures);
//        
//        FILE * file = fopen("/Users/lockercho/workspace/GPU/final_project/test/Rs.log", "w");
//        
//        for(int i=0; i<3; i++) {
//            for(int j=0 ; j<3 ; j++) {
//                fprintf(file, "%f, ", ransac.model_coefficients_(i, j));
//            }
//            fprintf(file, "\n");
//        }
//        fclose(file);
//        
//        exit(0);
        
        
		return ransac.model_coefficients_;
	}
}

std::vector<cv::Mat> Tracker::Track(cv::Mat(&imgs)[6])
{
	bool isKeyFrames[6];
    
    std::vector<cv::Mat> rotations;

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
            std::vector<cv::Mat> prevframes[6];
            std::vector<cv::Mat> currframes[6];

			for (int frameNum = 0; frameNum < 6; frameNum++)
			{
				std::vector<std::vector<cv::Point2f>> f(trackedFeatures[frameNum]);

				features[frameNum] = f;

				features[frameNum].push_back(subTrackers[frameNum].prevCorners);
                
                prevframes[frameNum].push_back(subTrackers[frameNum].prevFrame);
                currframes[frameNum].push_back(subTrackers[frameNum].currFrame);
			}

			int begin = 0;
			int end = features[0].size() - 1;

			do
			{
				auto keyloc = end;

				for (int frameNum = 0; frameNum < 6; frameNum++)
				{
					auto e = end;

					GetKeyframeRotation(features[frameNum], begin, e, true, prevframes[frameNum], currframes[frameNum]);

					if (e <= keyloc)
					{
						keyloc = e;
					}
				}
                
				for (int frameNum = 0; frameNum < 6; frameNum++)
				{
					auto e = keyloc;
					auto rotation = GetKeyframeRotation(features[frameNum], begin, e, false, prevframes[frameNum], currframes[frameNum]);
                    
                    double tmp = rotation(0,0);
                    std::vector<double> rot;
                    double m[3][3];
                    for(int i=0 ;i<3 ; i++) {
                        for(int j=0; j<3 ; j++) {
                            m[i][j] = rotation(i, j);
                        }
                    }
                    cv::Mat M = cv::Mat(3, 3, CV_64F, m).inv();
                    
                    rotations.push_back(M);
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
    return rotations;
}
