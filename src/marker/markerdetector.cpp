/*
 * markerdetector.cpp
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#include "marker/markerdetector.h"

namespace fml {

MarkerDetector::MarkerDetector() {
	pyr_down_ = max(Config::camera_parameters.camera_size.height, Config::camera_parameters.camera_size.width) > 1000;

	// this remains the only configurable option, put it somewhere else
	float min_deviation_factor = 1 - Config::tracker_parameters.marker_tolerance;
	float max_deviation_factor = 1 + Config::tracker_parameters.marker_tolerance;

	float f_min = min(Config::calibration_parameters.f.x, Config::calibration_parameters.f.y);
	float f_max = max(Config::calibration_parameters.f.x, Config::calibration_parameters.f.y);

	// we cannot find any rectangle inside the image, that is bigger than the actual image and smaller than 0
	// min and max projected length for all markers
	cv::Point marker_size_range(max(Config::camera_parameters.camera_size.width, Config::camera_parameters.camera_size.height), 0);
	// determine maximal ratio
	float f1 = min_deviation_factor * f_min;
	float f2 = max_deviation_factor * f_max;
	float min_max_length_ratio = FLOOR(f1)/CEIL(f2);
	opposing_length_ratio_ = min_deviation_factor * f_min/f_max;

	// find projected size for all known markers. Given some uncertainty modeled by the deviation factor these are the only sizes we care about
	for(map<int, Pose2d>::const_iterator it = Config::marker_parameters.base_T_m.begin(); it != Config::marker_parameters.base_T_m.end(); ++it) { // for all markers
		// the markers physical length in mm
		float physical_length_mm = Config::tracker_parameters.special_marker_sizes_mm.count(it->first) ? Config::tracker_parameters.special_marker_sizes_mm.at(it->first) : Config::tracker_parameters.marker_size_mm;
		// helper variable
		float length_by_z = physical_length_mm/(it->second.translation3d[2] - Config::camera_parameters.z_offset);
		// determine range of this specific marker and store it into point
		cv::Point min_max_range(FLOOR(f1 * length_by_z), CEIL(f2 * length_by_z));
		// update min max settings
		if(min_max_range.x < marker_size_range.x) {
			marker_size_range.x = min_max_range.x;
		}
		if(min_max_range.y > marker_size_range.y) {
			marker_size_range.y = min_max_range.y;
		}
	}

	size_t min_allowed_length = marker_size_range.x;
	size_t max_allowed_length = marker_size_range.y;

	min_perimeter_ = 4 * min_allowed_length;
	max_perimeter_ = 4 * max_allowed_length;
	min_length_p2_ = min_allowed_length * min_allowed_length;
	max_length_p2_ = max_allowed_length * max_allowed_length;
	min_max_ratio_p2_ = min_max_length_ratio * min_max_length_ratio;
	opposing_ratio_p2_ = opposing_length_ratio_ * opposing_length_ratio_;
	min_deviation_factor_p2_ = min_deviation_factor * min_deviation_factor;
	max_deviation_factor_p2_ = max_deviation_factor * max_deviation_factor;
}

MarkerDetector::~MarkerDetector() {
}

int MarkerDetector::detect(const cv::Mat &image, list<vector<cv::Point> > &candidates) {
	candidates.clear();
	findSquares(image, candidates);
	filterSquares(candidates);
	return(candidates.size());
}

// approximate the curve by a rectangle-like quadrilateral defined by the most prominent 4 corners
// it features a runtime of O(n) vs. the DP algorithm, which is O(n^2), but the since this is not a bottleneck, the reason
// for its implementation is another: this algorithm will find rectangles even when they are distorted in the original image.
// hence we do not need to undistort it.
void MarkerDetector::approxQuad(const CvSeq* const contour, vector<cv::Point> &square) const {
	int num_points = contour->total;

	CvPoint* start_point = (CvPoint*)cvGetSeqElem(contour, 0);
	cv::Point corner[2];
	for(size_t i = 0; i < 2; ++i) {
		int max_dist = 0;
		int max_index = 0;
		for(int j = 0; j < num_points; ++j) {
			CvPoint* pixel = (CvPoint*)cvGetSeqElem(contour, j);
			int dx = pixel->x - start_point->x;
			int dy = pixel->y - start_point->y;
			int dist = dx*dx + dy*dy;
			if(unlikely(dist > max_dist)) {
				max_dist = dist;
				max_index = j;
			}
		}
		start_point = (CvPoint*)cvGetSeqElem(contour, max_index);
		corner[MOD2(i)] = (*(CvPoint*)start_point);
	}

	square[0] = corner[0];
	square[2] = corner[1];

	int min_diff = INT_MAX;
	int min_index = 0;
	for(int i = 0; i < num_points; ++i) {
		CvPoint* pixel = (CvPoint*)cvGetSeqElem(contour, i);
		int dx = pixel->x - corner[0].x;
		int dy = pixel->y - corner[0].y;
		int dist1 = dx*dx + dy*dy;
		dx = pixel->x - corner[1].x;
		dy = pixel->y - corner[1].y;
		int dist2 = dx*dx + dy*dy;
		int dist = abs(dist1 - dist2);
		if(unlikely(dist < min_diff)) {
			min_diff = dist;
			min_index = i;
		}
	}

	start_point = (CvPoint*)cvGetSeqElem(contour, min_index);

	for(size_t i = 0; i < 2; ++i) {
		int max_dist = 0;
		int max_index = 0;
		for(int j = 0; j < num_points; ++j) {
			CvPoint* pixel = (CvPoint*)cvGetSeqElem(contour, j);
			int dx = pixel->x - start_point->x;
			int dy = pixel->y - start_point->y;
			int dist = dx*dx + dy*dy;
			if(unlikely(dist > max_dist)) {
				max_dist = dist;
				max_index = j;
			}
		}
		start_point = (CvPoint*)cvGetSeqElem(contour, max_index);
		corner[MOD2(i)] = (*(CvPoint*)start_point);
	}

	square[1] = corner[0];
	square[3] = corner[1];
}

void MarkerDetector::filterSquares(list<vector<cv::Point> > &candidates) const {
	for(list<vector<cv::Point> >::iterator candidate_it = candidates.begin(); candidate_it != candidates.end();) {
		// we assume having a convex polygon:
		// diagonal checks below effectively guarantee the filtering of all concave quatrilateral).
		// proof by contradiction (sketch):
		// 1) the only type of concave quadrilateral exists features exactly one reflex angle (>180�)
		// 2) assuming a concave quatrilateral would pass the test, an angle > 180 would pass the test
		// 3) the opposing and min_max lengths constraints makes sure all sides are roughly equally long
		// given 3)
		// 4) the diagonal check ensures that all angles are approx. 90� and hence either obtuse or acute) => contradiction
		size_t length_p2[6];
		length_p2[0] = squared_distance((*candidate_it)[0], (*candidate_it)[1]);
		size_t short_length_p2 = length_p2[0], long_length_p2 = length_p2[0];
		for(size_t i = 1; i < 4; ++i) {
			length_p2[i] = squared_distance((*candidate_it)[i], (*candidate_it)[MOD4((i+1))]);
			if(length_p2[i] < short_length_p2) {
				short_length_p2 = length_p2[i];
			} else if(length_p2[i] > long_length_p2) {
				long_length_p2 = length_p2[i];
			}
		}
		// the diagonals must also not be smaller than minimal allowed length (since we deal with the squared lengths, the diagonal is twice as big instead of sqrt(2))
		// this will destroy very flat polygons efficiently
		length_p2[4] = squared_distance((*candidate_it)[0], (*candidate_it)[2]) >> 1;
		length_p2[5] = squared_distance((*candidate_it)[1], (*candidate_it)[3]) >> 1;
		if(unlikely(length_p2[4] < short_length_p2)) {
			short_length_p2 = length_p2[4];
		}
		if(unlikely(length_p2[5] < short_length_p2)) {
			short_length_p2 = length_p2[5];
		}

		// delete all candidates which cannot be markers due to constraints
		size_t min_short_allowed_p2 = max(min_length_p2_, (size_t)(min_max_ratio_p2_ * long_length_p2));	// shortest line must be greater than this value
		bool erase_it = false;
		if(short_length_p2 < min_short_allowed_p2) { // constraint regarding the smaller sidelength (first since many false positives are small)
			erase_it = true;
		} else if(unlikely(long_length_p2 > max_length_p2_)) { // max length constraint (severely reduces worst case computational load of the succeeding code)
			erase_it = true;
		} else {	// some more complex filters
			size_t a2_b2 = (length_p2[0] + length_p2[1]) >> 1;
			size_t b2_c2 = (length_p2[1] + length_p2[2]) >> 1;
			// pythagoras (length of the diagonal). We use the min/max_deviation_factor_p2_ here because the angle might not be a perfect 90�
			// we have to take half of this value, because we did determine 1/2 value of the diagonal above
			if((length_p2[4] < min_deviation_factor_p2_ * a2_b2) || (length_p2[4] > max_deviation_factor_p2_ * a2_b2) || (length_p2[5] < min_deviation_factor_p2_ * b2_c2) || (length_p2[5] > max_deviation_factor_p2_ * b2_c2)) {
				erase_it = true;
			} else {
				// opposing line ratio constraint
				size_t min1, max1, min2, max2;
				if(length_p2[0] > length_p2[2]) {
					min1 = length_p2[2];
					max1 = length_p2[0];
				} else {
					min1 = length_p2[0];
					max1 = length_p2[2];
				}
				if(length_p2[1] > length_p2[3]) {
					min2 = length_p2[3];
					max2 = length_p2[1];
				} else {
					min2 = length_p2[1];
					max2 = length_p2[3];
				}
				if(opposing_ratio_p2_ * max1 > min1 || opposing_ratio_p2_ * max2 > min2) {
					erase_it = true;
				}
			}
		}
		if(erase_it) {
			candidate_it = candidates.erase(candidate_it);
		} else {
			++candidate_it;
		}
	}
}

float MarkerDetector::fastContourLength(const vector<cv::Point> &curve, const bool closed) const {
	const float pixel_distance[3] = { 0, 1, sqrt_2 };
	float length = 0;
	int last_index = curve.size()-1;

	// determine the distance between a points in the contour and its successor (either SQRT(1) or SQRT(2))
	for(int i = 0; i < last_index; ++i) {
		// how big is the distance between i and i+1
		int d_x = curve[i].x - curve[i+1].x;
		int d_y = curve[i].y - curve[i+1].y;
		int d = d_x*d_x + d_y*d_y;
		length += pixel_distance[d];
	}
	if(closed) {
		int d_x = curve[last_index].x - curve[0].x;
		int d_y = curve[last_index].y - curve[0].y;
		int d = d_x*d_x + d_y*d_y;
		length += pixel_distance[d];
	}

	return(length);
}

/**
 * Computes the contour length efficiently and copies the contour to a more manageable datastructure at the same time
 *
 * @param cv_contour
 * @param curve
 * @param closed
 * @return
 */
float MarkerDetector::fastContourLength(const CvSeq* const cv_contour, const bool closed) const {
	const int pixel_distance[5] = { 1, 0, 1, 0, 1 };
	int pixel_value_count[2] = { 0, 0 };
	int num_points = cv_contour->total;

	CvSeqReader reader;
	cvStartReadSeq(cv_contour, &reader);
	// determine the distance between a points in the contour and its successor (either SQRT(1) or SQRT(2))
	CvPoint p;
	CV_READ_SEQ_ELEM(p, reader);

	CvPoint p_first = p;	// buffer first element for closed
	for(int i = 1; i < num_points; ++i) {
		int sum = 2 + p.x + p.y;	// use p
		CV_READ_SEQ_ELEM(p, reader); // turn p into p_next
		sum -= p.x + p.y;	// use p_next
		++pixel_value_count[pixel_distance[sum]];
	}

	if(closed) {
		int sum = 2 + p.x + p.y - p_first.x - p_first.y;
		++pixel_value_count[pixel_distance[sum]];
	}

	return(pixel_value_count[0] + (float)sqrt_2 * pixel_value_count[1]);
}

float MarkerDetector::contourLength(const CvSeq* const cv_contour, const bool closed) const {
	int num_points = cv_contour->total;

	CvSeqReader reader;
	cvStartReadSeq(cv_contour, &reader);
	// determine the distance between a points in the contour and its successor (either SQRT(1) or SQRT(2))
	CvPoint p, p_next;
	CV_READ_SEQ_ELEM(p, reader);

	CvPoint p_first = p;	// buffer first element for closed
	float length = 0;
	for(int i = 1; i < num_points; ++i) {
		CV_READ_SEQ_ELEM(p_next, reader); // turn p into p_next
		int dx = p_next.x - p.x;
		int dy = p_next.y - p.y;
		length += SQRT(dx*dx + dy*dy);
		p = p_next;
	}

	if(closed) {
		int dx = p_first.x - p.x;
		int dy = p_first.y - p.y;
		length += SQRT(dx*dx + dy*dy);
	}

	return(length);
}

size_t MarkerDetector::getMaxPerimeter() const {
	return(max_perimeter_);
}

void MarkerDetector::setPyrDown(const bool value) {
	pyr_down_ = value;
}

} /* namespace fml */
