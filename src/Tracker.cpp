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
#include <ceres/ceres.h>
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

template <typename T>
T ARobustLossFunction(const T s)
{
	const T a = T(0.01);
	return pow(a, 2) * log(T(1) + s / pow(a, 2));
}

struct E1CostFunctor
{
	Eigen::Vector3d p;

	Eigen::Vector3d p1;

	E1CostFunctor(Eigen::Vector3d p, Eigen::Vector3d p1)
	{
		this->p = p;
		this->p1 = p1;
	}

	template <typename T>
	bool operator()(const T* const Rj, const T* const Rj1, T* residual) const
	{
		Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T>>(Rj);
		Eigen::Quaternion<T> q1 = Eigen::Map<const Eigen::Quaternion<T>>(Rj1);

		auto s = ((q * p.cast<T>()) - (q1 * p1.cast<T>())).squaredNorm();
		residual[0] = ARobustLossFunction(s);
		residual[1] = ARobustLossFunction(s);
		return true;
	}
};

struct E2CostFunctor
{
	Eigen::Vector3d p;

	Eigen::Vector3d p1;

	Eigen::Vector3d p2;

	E2CostFunctor(Eigen::Vector3d p, Eigen::Vector3d p1, Eigen::Vector3d p2)
	{
		this->p = p;
		this->p1 = p1;
		this->p2 = p2;
	}

	template <typename T>
	bool operator()(const T* const Rj, const T* const Rj1, const T* const Rj2, T* residual) const
	{
		Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T>>(Rj);
		Eigen::Quaternion<T> q1 = Eigen::Map<const Eigen::Quaternion<T>>(Rj1);
		Eigen::Quaternion<T> q2 = Eigen::Map<const Eigen::Quaternion<T>>(Rj2);

		auto s = (-(q * p.cast<T>()) + T(2) * (q1 * p1.cast<T>()) - (q2 * p2.cast<T>())).squaredNorm();
		residual[0] = ARobustLossFunction(s);
		residual[1] = ARobustLossFunction(s);
		residual[2] = ARobustLossFunction(s);
		return true;
	}
};

opengv::transformation_t Tracker::GetKeyframeRotation(std::vector<std::vector<cv::Point2f>> features, int begin, int& end, bool recursive)
{
	// set bearing vector of the features as the normalized [u v 128]
	bearingVectors_t bearingVectors1;
	bearingVectors_t bearingVectors2;

	auto previousFeatures = features.at(begin);
	auto presentFeatures = features.at(end);

	auto previousFeaturesCount = previousFeatures.size();
	auto presentFeaturesCount = presentFeatures.size();

	auto featuresCount = std::min(previousFeaturesCount, presentFeaturesCount);

	for (auto p = 0; p < featuresCount; ++p)
	{
		Eigen::Vector3d previous(previousFeatures.at(p).x, previousFeatures.at(p).y, 128);
		previous = previous / previous.norm();

		Eigen::Vector3d present(presentFeatures.at(p).x, presentFeatures.at(p).y, 128);
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
// new one with cubic coordinate (old one is within per image coordinate)
/**    4
 * 0 1 2 3
 *     5
 * let 2 as front, 1,3 as left, right and so on
 */
opengv::transformation_t Tracker::GetKeyframeRotation(std::vector<std::vector<cv::Point2f>> features, int begin, int& end, bool recursive, int frameIndex)
{
    // set bearing vector of the features as the normalized [u v 128]
    bearingVectors_t bearingVectors1;
    bearingVectors_t bearingVectors2;
    
    auto previousFeatures = features.at(begin);
    auto presentFeatures = features.at(end);
    
    auto previousFeaturesCount = previousFeatures.size();
    auto presentFeaturesCount = presentFeatures.size();
    
    auto featuresCount = std::min(previousFeaturesCount, presentFeaturesCount);
    
    for (auto p = 0; p < featuresCount; ++p)
    {
        Eigen::Vector3d previous(previousFeatures.at(p).x-128, previousFeatures.at(p).y-128, 128);
        Eigen::Vector3d present(presentFeatures.at(p).x, presentFeatures.at(p).y, 128);
        Eigen::Vecotr3d temp;
        
        // do index swapping to fetch the rotation matrix from same coordinate
        switch (frameIndex)
        {
            case 0: // back
                previous(0) = previous(0)*(-1);
                previous(1) = previous(1)*(-1);
                present(0) = present(0)*(-1);
                present(1) = present(1)*(-1);
                break;
            case 1: // left
                temp = previous;
                previous(0) = temp(2)*(-1);
                previous(2) = temp(0);
                temp = present;
                present(0) = temp(2)*(-1);
                present(2) = temp(0);
                break;
            case 2: // front
                break;
            case 3: // right
                temp = previous;
                previous(0) = temp(2);
                previous(2) = temp(0);
                temp = present;
                present(0) = temp(2);
                present(2) = temp(0);
                break;
            case 4: // top
                temp = previous;
                previous(1) = temp(2)*(-1);
                previous(2) = temp(1);
                temp = present;
                present(1) = temp(2)*(-1);
                present(2) = temp(1);
                break;
            case 5: // bot
                temp = previous;
                previous(1) = temp(2);
                previous(2) = temp(1);
                temp = present;
                present(1) = temp(2);
                present(2) = temp(1);
                break;
            default:
                break;
        }
        
        
        previous = previous / previous.norm();
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

std::vector<std::vector<Eigen::Quaternion<double> > > Tracker::Track(cv::Mat(&imgs)[6])
{
	bool isKeyFrames[6];
    
    std::vector<cv::Mat> rotations;
    
    std::vector<std::vector<Eigen::Quaternion<double> > > quaternionsRet;

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

					GetKeyframeRotation(features[frameNum], begin, e, true);

					if (e <= keyloc)
					{
						keyloc = e;
					}
				}
 
				for (int frameNum = 0; frameNum < 6; frameNum++)
				{
					auto e = keyloc;
					auto transformation = GetKeyframeRotation(features[frameNum], begin, e, false);
					auto rotation = opengv::rotation_t(transformation.block<3, 3>(0, 0));

					// Rki
					auto inverseRotation = StoredRotations[frameNum].inverse();

					// Rki+1
					auto inverseRotationNext = opengv::rotation_t(StoredRotations[frameNum] * rotation).inverse();

					auto quaternion = Eigen::Quaternion<double>(inverseRotation);
					auto quaternionNext = Eigen::Quaternion<double>(inverseRotationNext);

					// Rj
					auto quaternions = std::vector<Eigen::Quaternion<double>>();
					for (auto i = begin; i < e; ++i)
					{
						auto t = begin / e;
						auto initQuaternion = quaternion.slerp(t, quaternionNext);

						quaternions.push_back(initQuaternion);
					}

					ceres::Problem problem;

					for (auto innerFrameIndex = 0; innerFrameIndex < quaternions.size(); ++innerFrameIndex)
					{
						if (innerFrameIndex + 1 < quaternions.size())
						{
							for (auto p_index = 0; p_index < features[frameNum].at(innerFrameIndex).size() - 1; p_index++)
							{
								auto p = features[frameNum].at(innerFrameIndex).at(p_index);
								auto p1 = features[frameNum].at(innerFrameIndex + 1).at(p_index + 1);

								auto v = Eigen::Vector3d(p.x, p.y, 1);
								auto v1 = Eigen::Vector3d(p1.x, p1.y, 1);

								auto cost_function = new ceres::AutoDiffCostFunction<E1CostFunctor, 2, 4, 4>(new E1CostFunctor(v, v1));

								problem.AddResidualBlock(
									cost_function,
									nullptr,
									quaternions.at(innerFrameIndex).coeffs().data(),
									quaternions.at(innerFrameIndex + 1).coeffs().data());
							}
						}

						if (innerFrameIndex + 2 < quaternions.size())
						{
							for (auto p_index = 0; p_index < features[frameNum].at(innerFrameIndex).size() - 2; p_index++)
							{
								auto p = features[frameNum].at(innerFrameIndex).at(p_index);
								auto p1 = features[frameNum].at(innerFrameIndex + 1).at(p_index + 1);
								auto p2 = features[frameNum].at(innerFrameIndex + 2).at(p_index + 2);

								auto v = Eigen::Vector3d(p.x, p.y, 1);
								auto v1 = Eigen::Vector3d(p1.x, p1.y, 1);
								auto v2 = Eigen::Vector3d(p2.x, p2.y, 1);

								auto cost_function = new ceres::AutoDiffCostFunction<E2CostFunctor, 3, 4, 4, 4>(new E2CostFunctor(v, v1, v2));

								problem.AddResidualBlock(
									cost_function,
									nullptr,
									quaternions.at(innerFrameIndex).coeffs().data(),
									quaternions.at(innerFrameIndex + 1).coeffs().data(),
									quaternions.at(innerFrameIndex + 2).coeffs().data());
							}
						}

						if (problem.NumResidualBlocks() != 0)
						{
							problem.SetParameterization(quaternions.at(innerFrameIndex).coeffs().data(), new ceres::EigenQuaternionParameterization());
						}
					}

					ceres::Solver::Options options;
					options.minimizer_progress_to_stdout = true;

					ceres::Solver::Summary summary;
					ceres::Solve(options, &problem, &summary);

					std::cout << summary.BriefReport() << "\n";

					// TODO: the quaternions has the optimized R for each innerframe
                    quaternionsRet.push_back(quaternions);
                    

					this->StoredRotations[frameNum] *= rotation;
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
		else
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

			for (auto i = 0; i < 6; i++)
			{
				auto transformation = GetKeyframeRotation(features[i], begin, end, false);
				auto rotation = rotation_t(transformation.block<3, 3>(0, 0));
				this->StoredRotations[i] = rotation;
			}
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

	if (hadKeyFrames)
	{
		frameCount = 0;
	}

	return quaternionsRet;
}
