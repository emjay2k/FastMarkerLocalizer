/*
 * markerdetector.h
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#ifndef SRC_MARKERDETECTOR_H_
#define SRC_MARKERDETECTOR_H_

#include <algorithm>
#include <list>
#include <map>
#include <vector>
#include <opencv2/opencv.hpp>
#include "generic/helper.h"
#include "configure/config.h"
#include "marker.h"

using namespace std;

namespace fml {

class MarkerDetector {

protected:
	bool pyr_down_;

	float max_deviation_factor_p2_;
	float min_deviation_factor_p2_;

	float opposing_length_ratio_;

	size_t min_perimeter_;
	size_t max_perimeter_;
	size_t min_length_p2_;
	size_t max_length_p2_;
	float min_max_ratio_p2_;
	float opposing_ratio_p2_;

	void approxQuad(const CvSeq* const contour, vector<cv::Point> &square) const;
	void filterSquares(list<vector<cv::Point> > &candidates) const;
	float fastContourLength(const vector<cv::Point> &curve, const bool closed) const;
	float fastContourLength(const CvSeq* const cv_contour, const bool closed) const;
	float contourLength(const CvSeq* const cv_contour, const bool closed) const;
	virtual void findSquares(const cv::Mat &image, list<vector<cv::Point> > &candidates) = 0;

public:
	MarkerDetector();
	virtual ~MarkerDetector();

	int detect(const cv::Mat &image, list<vector<cv::Point> > &candidates);
	void setPyrDown(const bool value);

	size_t getMaxPerimeter() const;
};

} /* namespace fml */

#endif /* SRC_MARKERDETECTOR_H_ */
