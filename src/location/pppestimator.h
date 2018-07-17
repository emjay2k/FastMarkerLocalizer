/*
 * pppestimator.h
 *
 *  Created on: 04.02.2015
 *      Author: jung
 */

#ifndef SRC_LOCATION_PPPESTIMATOR_H_
#define SRC_LOCATION_PPPESTIMATOR_H_

#include <array>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "location/poseestimator.h"
#include "generic/helper.h"

using namespace std;

namespace fml {

class PPPEstimator : public PoseEstimator {

	array<float, 4> computeFXWeight(const cv::Point2f (&vertices)[4]) const;
	float computeRotation(const cv::Point2f (&vertices)[4], float (&accuracy)[4]) const;
	float computeRotation(const cv::Point2f (&vertices)[4], const float ref, float (&accuracy)[4]) const;
	cv::Point2f computePosition(const cv::Point2f &p, const float z) const;
	cv::Point2f computePosition(const cv::Point2f (&vertices)[4], const float z) const;

public:
	PPPEstimator();
	virtual ~PPPEstimator();

	void estimatePose(list<MarkerInfo> &markers);
	void estimatePose(list<MarkerInfo> &markers, const Pose2d &predict) const;
};

} /* namespace fml */

#endif /* SRC_LOCATION_PPPESTIMATOR_H_ */
