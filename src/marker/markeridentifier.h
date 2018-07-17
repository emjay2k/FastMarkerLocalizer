/*
 * markeridentifier.h
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#ifndef SRC_MARKERIDENTIFIER_H_
#define SRC_MARKERIDENTIFIER_H_

#include <opencv2/opencv.hpp>
#include "imgproc.h"
#include "generic/definitions.h"
#include "configure/config.h"
#include "marker.h"

using namespace std;

namespace fml {

class MarkerIdentifier {

protected:
	const size_t marker_warp_size_ = 28; // TODO: make configurable

	cv::Mat getPerspectiveTransform(const cv::Point2f (&src)[4], const cv::Point2f (&dst)[4]) const;
	void warpPerspective(const cv::Mat &src, cv::Mat &dst, cv::Mat transform, const uint8_t border_value=0) const;
	void warp_helper(const cv::Mat &src, cv::Mat &dst, const vector<cv::Point2f> &points) const;

	virtual void extractMarkerInfo(const cv::Mat &image, vector<cv::Point2f> &candidate, MarkerInfo &marker) const = 0;
	virtual void extractMarkerInfo(const cv::Mat &image, list<vector<cv::Point> > &candidates, list<MarkerInfo> &markers) const = 0;

public:
	MarkerIdentifier();
	virtual ~MarkerIdentifier();

	virtual bool identify(const cv::Mat &image, vector<cv::Point2f> &candidate, MarkerInfo &marker) const;
	virtual int identify(const cv::Mat &image, list<vector<cv::Point> > &candidates, list<MarkerInfo> &markers) const;
};

} /* namespace fml */

#endif /* SRC_MARKERIDENTIFIER_H_ */
