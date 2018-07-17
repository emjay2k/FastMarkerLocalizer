/*
 * arucomarkerdetector.h
 *
 *  Created on: 03.02.2015
 *      Author: jung
 */

#ifndef SRC_MARKER_ARUCOMARKERDETECTOR_H_
#define SRC_MARKER_ARUCOMARKERDETECTOR_H_

#include <vector>
#include "marker/markerdetector.h"
#include "imgproc.h"

using namespace std;

namespace fml {

class ArucoMarkerDetector : public MarkerDetector {

	static const int ADAPTIVE_THRESHOLD_BLOCKSIZE = 7;
	static const int ADAPTIVE_THRESHOLD_DELTA = 7;
	static int size_template_[65536];
	static uint16_t label_template_[65536];
	uint16_t L_[65536];	// equivalent list
	int size_[65536];	// size of the segments
	cv::Point *tl_;	// segment top left corner
	cv::Point *br_;	// segment bottom right corner

	// Crossproduct of 0A and 0B vectors (taken from wikipedia)
	// modified because we have the points sorted by y, instead of x
	inline int crossProduct(const CvPoint* const O, const CvPoint* const A, const CvPoint* const B) const {
		return((A->x - O->x) * (B->y - O->y) - (A->y - O->y) * (B->x - O->x));
	}

	// runs in O(1) used the code from wikipedia: https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain
	void convexHullMC(const CvSeq* const P, vector<CvPoint*> &H) const;
	void search_part(const cv::Mat &image, list<vector<cv::Point> > &candidates, const int factor_pyr);
	void integratedAdaptiveThreshold(const cv::Mat &src, cv::Mat &dst) const;
	void filterBinary(const cv::Mat &image, const int min_length_estimate, const int max_length_estimate, const int max_perimeter, list<vector<cv::Point> > &candidates);

public:
	ArucoMarkerDetector();
	virtual ~ArucoMarkerDetector();

	void findSquares(const cv::Mat &image, list<vector<cv::Point> > &candidates);
	static void init_static();
};

} /* namespace fml */

#endif /* SRC_MARKER_ARUCOMARKERDETECTOR_H_ */
