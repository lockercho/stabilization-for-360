#include "Tracker.h"

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
