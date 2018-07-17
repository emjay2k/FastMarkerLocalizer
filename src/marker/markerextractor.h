/*
 * markerextractor.h
 *
 *  Created on: 01.08.2015
 *      Author: Jung
 */

#ifndef SRC_MARKEREXTRACTOR_H_
#define SRC_MARKEREXTRACTOR_H_

#include <array>
#include <list>
#include <vector>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "configure/config.h"
#include "marker/marker.h"
#include "marker/arucomarkerdetector.h"
#include "marker/arucomarkeridentifier.h"
#include "marker/undistorter.h"

namespace fml {

static const int map_corner_buffer[24][4] = {
		{ 0, 1, 2, 3 },
		{ 0, 1, 3, 2 },
		{ 0, 2, 1, 3 },
		{ 0, 2, 3, 1 },
		{ 0, 3, 1, 2 },
		{ 0, 3, 2, 1 },
		{ 1, 0, 2, 3 },
		{ 1, 0, 3, 2 },
		{ 1, 2, 0, 3 },
		{ 1, 2, 3, 0 },
		{ 1, 3, 0, 2 },
		{ 1, 3, 2, 0 },
		{ 2, 0, 1, 3 },
		{ 2, 0, 3, 1 },
		{ 2, 1, 0, 3 },
		{ 2, 1, 3, 0 },
		{ 2, 3, 0, 1 },
		{ 2, 3, 1, 0 },
		{ 3, 0, 1, 2 },
		{ 3, 0, 2, 1 },
		{ 3, 1, 0, 2 },
		{ 3, 1, 2, 0 },
		{ 3, 2, 0, 1 },
		{ 3, 2, 1, 0 },
};

static const float max_residual = 0.2f;

class MarkerExtractor {

protected:
	const float opp_length_ratio = 1.05f * 1.05f;
	const float seventh_lookup[7] = { 1.0f/7.0f, 2.0f/7.0f, 3.0f/7.0f, 4.0f/7.0f, 5.0f/7.0f, 6.0f/7.0f, 1.0f };

	size_t max_marker_;

	cv::Vec4f refineLine(const cv::Mat &image, const cv::Point2f &roi_tl, const cv::Point2f &start, const cv::Point2f &end, const int iteration) const;
	cv::Vec4f refineLineUltra(const cv::Mat &image, const cv::Point2f &roi_tl, const cv::Point2f &start, const cv::Point2f &end, float &residual, const int iteration) const;
	void refineMarkerLines(const cv::Mat &image, const cv::Point2f &roi_tl, cv::Point2f (&vertices)[4], const float projected_length, const int iterations=2) const;
	void refineMarkerLinesUltra(const cv::Mat &image, const cv::Point2f &roi_tl, cv::Point2f (&vertices)[4], float (&residuals)[4], const int iterations=2) const;
	void reconstructShape(cv::Point2f (&vertices)[4], const bool (&inside_image)[4], const float projected_length) const;
	bool keepMarker(const MarkerInfo &marker_info) const;
	bool isAccurateEnough(const float (&line_accuracy)[4]) const;
	cv::Vec4f fitLine(vector<cv::Point2f> &points) const;

public:
	int num_frames;
	void setMaxMarker(const size_t value);

	MarkerExtractor();
	virtual ~MarkerExtractor();
};

} /* namespace fml */

#endif /* SRC_MARKEREXTRACTOR_H_ */
