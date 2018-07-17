/*
 * poseestimator.h
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#ifndef SRC_POSEESTIMATOR_H_
#define SRC_POSEESTIMATOR_H_

#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include "configure/config.h"
#include "location/pose2d.h"
#include "marker/marker.h"

using namespace std;

namespace fml {

class PoseEstimator {

protected:
	bool extrinsic_guess_;
	cv::Mat camera_matrix_;

	static vector<cv::Point3f> norm_object_points;	// default object points for marker_normal_size
	static map<int, vector<cv::Point3f> > special_object_points;	// individual object points for separately configured marker sizes
	static map<int, float> z_value;	// constant z-value of the marker

	map<int, cv::Mat> previous_translation_;
	map<int, cv::Mat> previous_rotation_;

public:
	PoseEstimator(const bool extrinsic_guess=false);
	virtual ~PoseEstimator();

	virtual void estimatePose(list<MarkerInfo> &markers) = 0;
	virtual void setLast(const int id, const cv::Mat &translation, const cv::Mat &rotation);
	static void init_static();
	static float getZValue(int id);
};

} /* namespace fml */

#endif /* SRC_POSEESTIMATOR_H_ */
