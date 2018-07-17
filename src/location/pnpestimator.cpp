/*
 * pnpestimator.cpp
 *
 *  Created on: 01.06.2015
 *      Author: jung
 */

#include "location/pnpestimator.h"

namespace fml {

PnPEstimator::PnPEstimator(const bool extrinsic_guess, const int flags) : PoseEstimator(extrinsic_guess) {
	flags_ = flags;
}

PnPEstimator::~PnPEstimator() {
}

void PnPEstimator::estimatePose(list<MarkerInfo> &markers) {
	for(list<MarkerInfo>::iterator it = markers.begin(); it != markers.end(); ++it) {
		int id = it->id;
		// get model points
		vector<cv::Point3f> model_points = special_object_points.count(id) ? special_object_points.at(id) : norm_object_points;
		// copy image points
		vector<cv::Point2f> image_points(it->vertices, it->vertices+4);
		// result vectors for solvePnP
		cv::Mat translation;
		cv::Mat rotation;

		bool use_last = previous_translation_.count(id);
		if(use_last) {
			translation = previous_translation_.at(id);
			rotation = previous_rotation_.at(id);
		} else {
			translation = cv::Mat();
			rotation = cv::Mat();
		}

		bool success = solvePnP(model_points, image_points, camera_matrix_, cv::Mat(), rotation, translation, use_last, flags_);
		if(success) {
			// update rotation to real values
			rotation.at<double>(0,0) = 0;
			rotation.at<double>(1,0) = 0;
			if(z_value.count(id)) {
				translation.at<double>(2,0) = z_value.at(id);	// set to correct value
			}
			it->w_T_m = Pose2d(id, 0, 0, 0, translation.at<double>(0,0), translation.at<double>(1,0), translation.at<double>(2,0), rotation.at<double>(2,0));
			// update last vector where necessary
			if(extrinsic_guess_) {
				map<int, cv::Mat>::iterator it = previous_translation_.find(id);
				if(it != previous_translation_.end()) {
				    it->second = translation.clone();
				} else {
					previous_translation_.insert(pair<int, cv::Mat>(id, translation.clone()));
				}
				it = previous_rotation_.find(id);
				if(it != previous_rotation_.end()) {
					it->second = rotation.clone();
				} else {
					previous_rotation_.insert(pair<int, cv::Mat>(id, rotation.clone()));
				}
			}
		}
	}
}

} /* namespace fml */
