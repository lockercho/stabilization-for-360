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
    
    int a = adapter.getNumberCorrespondences();

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

                //FILE * file = fopen("/tmp/Rs.log", "a");
                
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

                    
					//printf("%d:\n", frameNum);
                    //fprintf(file, "%d:\n", frameNum);

					//std::cout << rotation << std::endl;
                    
                    //for(int i=0; i<3; i++) {
                    //    for(int j=0 ; j<3 ; j++) {
                    //        fprintf(file, "%f, ", rotation(i, j));
                    //    }
                    //    fprintf(file, "\n");
                    //}
                    
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

					this->StoredRotations[frameNum] *= rotation;
				}
                //fclose(file);
                

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

	return rotations;
}
