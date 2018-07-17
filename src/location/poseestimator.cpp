/*
 * poseestimator.cpp
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#include "location/poseestimator.h"

namespace fml {

vector<cv::Point3f> PoseEstimator::norm_object_points;	// default object points for marker_normal_size
map<int, vector<cv::Point3f> > PoseEstimator::special_object_points;	// individual object points for separately configured marker sizes
map<int, float> PoseEstimator::z_value;	// constant z-value of the marker

PoseEstimator::PoseEstimator(const bool extrinsic_guess) {
	camera_matrix_ = Config::calibration_parameters.camera_matrix;
	extrinsic_guess_ = extrinsic_guess;
}

PoseEstimator::~PoseEstimator() {
}

void PoseEstimator::setLast(const int id, const cv::Mat &translation, const cv::Mat &rotation) {
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

void PoseEstimator::init_static() {
	// buffer object points
	// size is split in half in each direction from the center of the marker
	float size = 0.5f * Config::tracker_parameters.marker_size_mm;
	norm_object_points.reserve(4);
	norm_object_points.push_back(cv::Point3f(-size, -size, 0));
	norm_object_points.push_back(cv::Point3f( size, -size, 0));
	norm_object_points.push_back(cv::Point3f( size,  size, 0));
	norm_object_points.push_back(cv::Point3f(-size,  size, 0));

	for(map<int, Pose2d>::const_iterator it = Config::marker_parameters.base_T_m.begin(); it != Config::marker_parameters.base_T_m.end(); ++it) {
		int id = it->first;
		z_value.insert(pair<int,float>(id, it->second.translation3d[2] - Config::camera_parameters.z_offset));	// store constant height for each marker
		if(Config::tracker_parameters.special_marker_sizes_mm.count(id)) {
			float size = 0.5f * Config::tracker_parameters.special_marker_sizes_mm.at(id);
			vector<cv::Point3f> marker_points;
			marker_points.reserve(4);
			marker_points.push_back(cv::Point3f(-size, -size, 0));
			marker_points.push_back(cv::Point3f( size, -size, 0));
			marker_points.push_back(cv::Point3f( size,  size, 0));
			marker_points.push_back(cv::Point3f(-size,  size, 0));

			special_object_points.insert(pair<int,vector<cv::Point3f> >(id, marker_points));
		}
	}
}

float PoseEstimator::getZValue(int id) {
	return(z_value.at(id));
}

} /* namespace fml */
