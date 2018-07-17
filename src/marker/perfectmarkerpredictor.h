/*
 * PerfectMarkerPredictor.h
 *
 *  Created on: 19.01.2015
 *      Author: jung
 */

#ifndef PERFECTMARKERPREDICTOR_H_
#define PERFECTMARKERPREDICTOR_H_

#include <algorithm>
#include <map>
#include <list>
#include <vector>
#include <opencv2/opencv.hpp>
#include "configure/config.h"
#include "generic/definitions.h"
#include "marker/roipredictor.h"
#include "marker/marker.h"
#include "marker/undistorter.h"

using namespace std;

namespace fml {

class PerfectMarkerPredictor : RoiPredictor {
	// maximum error of the projected pose in each direction
	const float max_error_mm = 100;
	float offset_xy_;				// offset to consider in x and y direction to determine marker candidates
	map<float, MarkerProjectedCorners*> projected_corners_by_size_;	// projected corners by size of the markers
	vector<MarkerInfo*> sorted_by_x_;	// world markers sorted by x coordinate
	MarkerInfo *comparison_value_;
	int num_rois_;
	int num_rois_completely_inside_;

	cv::Point3f pointCoordinate(const Pose2d &lhs, const Pose2d &rhs) const;	// multiply lhs and rhs but only return the translation part

public:
	PerfectMarkerPredictor();
	virtual ~PerfectMarkerPredictor();

	bool predict(const Pose2d &w_T_c, vector<MarkerROI> &rois);
};

}

#endif /* PERFECTMARKERPREDICTOR_H_ */
