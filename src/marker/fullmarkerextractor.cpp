/*
 * markerextractor.cpp
 *
 *  Created on: 01.08.2015
 *      Author: Jung
 */

#include "marker/fullmarkerextractor.h"

namespace fml {

FullMarkerExtractor::FullMarkerExtractor() {
}

FullMarkerExtractor::~FullMarkerExtractor() {
}

list<MarkerInfo> FullMarkerExtractor::getMarkers(cv::Mat &image, double &after_preprocessing, double &after_detect, double &after_identify) {
	list<MarkerInfo> markers;

	// preprocessing: use only equalizeHist here to gain better detection at low performance cost
	cv::Mat equalized;
	cv::equalizeHist(image, equalized);
	//cv::Mat image2;
	//fastNormalizedAwb(image, image);
	after_preprocessing = (double)cv::getTickCount() * Config::time_ms_factor;

	// now extract the markers from the full image
	list<vector<cv::Point> > candidates; //, candidates2;
	detector_.detect(equalized, candidates);
	//detector_.detect(image2, candidates2);
	//candidates.splice(candidates.end(), candidates2);
	// filter those candidates which do not warrant further examination (this is necessary as we have a relaxed detection for those cases where we do not need identification)
	for(list<vector<cv::Point> >::iterator it = candidates.begin(); it != candidates.end(); ) {
		if(keepCandidate(*it)) {
			++it;
		} else {
			it = candidates.erase(it);
		}
	}
	after_detect = (double)cv::getTickCount() * Config::time_ms_factor;
	identifier_.identify(equalized, candidates, markers);

	// do the advanced lines refinement
	cv::Point2f roi_tl(0,0);
	for(list<MarkerInfo>::iterator it = markers.begin(); it != markers.end();) {
		float projected_length;
		if(Config::marker_parameters.base_T_m.count(it->id)) {
			float fx_weight = computeFXWeight(it->vertices, Config::calibration_parameters.c);
			float f_wmean = fx_weight * Config::calibration_parameters.f.x + (1-fx_weight) * Config::calibration_parameters.f.y;
			projected_length = f_wmean * it->length_mm/(Config::marker_parameters.base_T_m.at(it->id).translation3d[2] - Config::camera_parameters.z_offset);
		} else {
			projected_length = -1;
		}
		refineMarkerLines(equalized, roi_tl, it->vertices, projected_length);

		if(projected_length > 0) {
			// reconstruct shape
			bool corner_inside[4] = { true, true, true, true };
			reconstructShape(it->vertices, corner_inside, projected_length);

			// distort points so we can work with them again
			vector<cv::Point2f> distorted_points(it->vertices, it->vertices+4);
			Undistorter::distortPoints(distorted_points);
			it->vertices[0] = distorted_points[0] - roi_tl;
			it->vertices[1] = distorted_points[1] - roi_tl;
			it->vertices[2] = distorted_points[2] - roi_tl;
			it->vertices[3] = distorted_points[3] - roi_tl;

			// refine the lines of the reconstructed shape
			refineMarkerLines(equalized, roi_tl, it->vertices, projected_length);
		}

		if(keepMarker(*it)) {
			// final refinement
			vector<cv::Point2f> distorted_points(it->vertices, it->vertices+4);
			Undistorter::distortPoints(distorted_points);
			it->vertices[0] = distorted_points[0] - roi_tl;
			it->vertices[1] = distorted_points[1] - roi_tl;
			it->vertices[2] = distorted_points[2] - roi_tl;
			it->vertices[3] = distorted_points[3] - roi_tl;
			refineMarkerLinesUltra(image, roi_tl, it->vertices, it->line_accuracy);

			++it;
		} else {
			it = markers.erase(it);
		}
	}
	after_identify = (double)cv::getTickCount() * Config::time_ms_factor;

	return(markers);
}

bool FullMarkerExtractor::keepCandidate(const vector<cv::Point> &candidate) const {
	float length1 = squared_distance(candidate[0], candidate[1]);
	float length2 = squared_distance(candidate[2], candidate[3]);
	float length3 = squared_distance(candidate[1], candidate[2]);
	float length4 = squared_distance(candidate[3], candidate[0]);
	float length_ratio1 = (length1 > length2) ? length1/length2 : length2/length1;
	float length_ratio2 = (length3 > length4) ? length3/length4 : length4/length3;

	return(length_ratio1 > length_ratio2 ? length_ratio1 < detection_opp_length_threshold : length_ratio2 < detection_opp_length_threshold);
}

} /* namespace fml */
