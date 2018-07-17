/*
 * markerextractor.h
 *
 *  Created on: 01.08.2015
 *      Author: Jung
 */

#ifndef SRC_NEWROIMARKEREXTRACTOR_H_
#define SRC_NEWROIMARKEREXTRACTOR_H_

#include <array>
#include <list>
#include <forward_list>
#include <vector>
#include "marker/markerextractor.h"

using namespace std;

namespace fml {

static const int search_grid = 6;
static const int search_pattern_offsets[8][2] = {
		{ 0 , search_grid } , { 0, -search_grid } , { search_grid, 0 } , { -search_grid, 0 } ,
		{ search_grid , search_grid } , { search_grid, -search_grid } , { -search_grid, -search_grid } , { -search_grid, search_grid }
};

class NewRoiMarkerExtractor : public MarkerExtractor {
	struct PointWithDist {
		int x;
		int y;
		float dist;
		PointWithDist(const int x, const int y, const float dist) {
			this->x = x;
			this->y = y;
			this->dist = dist;
		}

	    friend bool operator<(const PointWithDist &pwd1, const PointWithDist &pwd2) {
	        return(pwd1.dist < pwd2.dist);
	    }
	};

	uint8_t addWeighted_LUT[256][256];
	ArucoMarkerDetector detector_;
	ArucoMarkerIdentifier identifier_;
	float lb_factor_;
	float ub_factor_;

	int no_candidate_;
	int stage1_;
	int stage2_;
	int stage3_;

	//cv::Ptr<cv::FilterEngine> f_gb_;	// filter engine for gaussian blur
	void fastGaussianBlur5(const cv::Mat &src, cv::Mat &dst);
	void fastUnsharpMask(cv::Mat &image);
	bool getLine(cv::Mat &image, forward_list<PointWithDist> &points, const cv::Point2f &roi_tl, const float max_difference, cv::Vec4f &line);
	void getLinePoints(const cv::Mat &image, const cv::Point2f &start, const cv::Point2f &axis, const cv::Point2f &axis_grid, const cv::Vec4f &projected_line, const int window_size, forward_list<PointWithDist> &points);
	void findCandidate(cv::Mat &image, const MarkerROI &roi, forward_list<vector<cv::Point> > &candidates);
	cv::Vec4f refineEdge(cv::Mat &image, cv::Point2f &start, cv::Point2f &end);
	void refineEdges(cv::Mat &image, vector<cv::Point2f> &candidate, const int iterations=2);

public:
	NewRoiMarkerExtractor();
	virtual ~NewRoiMarkerExtractor();

	MarkerInfo findBestCandidate(const cv::Mat &image, const MarkerROI &roi, list<vector<cv::Point> > &candidates) const;
	list<MarkerInfo> getMarkers(const cv::Mat &image, const vector<MarkerROI> &rois, double &after_preprocessing, double &after_detect, double &after_identify);
};

} /* namespace fml */

#endif /* SRC_NEWROIMARKEREXTRACTOR_H_ */
