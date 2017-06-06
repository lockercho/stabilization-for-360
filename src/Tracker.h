#pragma once

/**
 * \brief tracker for an octant
 */
class SubTracker
{
private:

	/**
	 * \brief goodFeaturesToTrack's parameter maxCorners
	 */
	int maxCorners = 50;

	/**
	 * \brief goodFeaturesToTrack's parameter qualityLevel
	 */
	double qualityLevel = 0.01;

	/**
	 * \brief goodFeaturesToTrack's parameter minDistance
	 */
	double minDistance = 10;

public:

	/**
	 * \brief The luma plane of previous frame.
	 */
	cv::Mat prevLuma;

	/**
	 * \brief The feature points of previous frame.
	 */
	std::vector<cv::Point2f> prevCorners;

	/**
	 * \brief do track
	 * \param img a BGR cv::Mat frame
	 * \return This frame is keyframe or not.
	 */
	bool Track(cv::Mat& img);
};

/**
 * \brief tracker for six octant frames
 */
class Tracker
{
private:

	/**
	 * \brief an octant tracker
	 */
	SubTracker subTrackers[6];

	/**
	 * \brief previous keyframe
	 */
	cv::Mat KeyFrames[6];

	/**
	 * \brief feature points of each frames after keyframe
	 */
	std::vector<std::vector<cv::Point2f>> trackedFeatures[6];

public:

	/**
	 * \brief do track
	 * \param imgs six BGR cv::Mat frames
	 */
	void Track(cv::Mat(&imgs)[6]);
};
