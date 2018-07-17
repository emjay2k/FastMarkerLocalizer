/*
 * trackerparameters.cpp
 *
 *  Created on: 12.09.2014
 *      Author: jung
 */

#include "configure/trackerparameters.h"

namespace fml {

TrackerParameters::TrackerParameters() {
	demo = false;
	evaluation = false;

	network = false;
	network_id = 1;
	network_ip = "localhost";
	network_port = 24191;

	mapping = false;
	mapping_startid = 639;
	mapping_minmeasurements = 10;

	marker_mode = MarkerMode::ARUCO_FID;
	correction = true;
	split_detection = 0;
	marker_limit = 100;
	marker_size_mm = 100;
	marker_tolerance = 0.2f;
	max_error = 0.25f;
	min_line_quality = 0.1f;

	prediction = true;
	tracker_algorithm = TrackerAlgorithm::TRACK_VECTOR;
	kalman_Rk_x = 1e-04;
	kalman_Rk_theta = 1e-08;
	kalman_Qk_x = 1e-04;
	kalman_Qk_theta = 1e-08;

	pose_estimator = PoseEstimationAlgorithm::POSE_PPP;
}

TrackerParameters::TrackerParameters(const string &filename) {
	demo = false;
	evaluation = false;

	network = false;
	network_id = 1;
	network_ip = "localhost";
	network_port = 24191;

	mapping = false;
	mapping_startid = 639;
	mapping_minmeasurements = 10;

	marker_mode = MarkerMode::ARUCO_FID;
	correction = true;
	split_detection = 0;
	marker_limit = 100;
	marker_size_mm = 100;
	marker_tolerance = 0.2f;
	max_error = 0.25f;
	min_line_quality = 0.1f;

	prediction = true;
	tracker_algorithm = TrackerAlgorithm::TRACK_VECTOR;
	kalman_Rk_x = 1e-04;
	kalman_Rk_theta = 1e-08;
	kalman_Qk_x = 1e-04;
	kalman_Qk_theta = 1e-08;

	pose_estimator = PoseEstimationAlgorithm::POSE_PPP;

	cv::FileStorage fs(filename, cv::FileStorage::READ);
	int temp_int = 0;
	fs["Marker_Predict"] >> temp_int;
	prediction = temp_int > 0;
	temp_int = 0;
	fs["Marker_Correct"] >> temp_int;
	correction = temp_int > 0;
	temp_int = 0;
	fs["Split_Detection"] >> split_detection;
	temp_int = 0;
	fs["Marker_Limit"] >> marker_limit;
	temp_int = 0;
	fs["Mapping"] >> temp_int;
	mapping = temp_int > 0;
	temp_int = 0;
	fs["Evaluation"] >> temp_int;
	evaluation = temp_int > 0;
	temp_int = 0;
	fs["Demo"] >> temp_int;
	demo = temp_int > 0;
	temp_int = 0;

	// network settings
	fs["Network"] >> temp_int;
	network = temp_int > 0;
	fs["Network_Id"] >> network_id;
	fs["Network_Ip"] >> network_ip;
	fs["Network_Port"] >> network_port;

	// mapping settings
	fs["Mapping_Startid"] >> mapping_startid;
	fs["Mapping_Minmeasurements"] >> mapping_minmeasurements;

	const string str_aruco_fid("ARUCO_FID");
	const string str_aruco_hrm("ARUCO_HRM");
	const string str_ppp("POSE_ESTIMATOR_PPP");
	const string str_rpp("POSE_ESTIMATOR_RPP");
	const string str_iterative("POSE_ESTIMATOR_ITERATIVE");
	const string str_epnp("POSE_ESTIMATOR_EPNP");
	const string str_p3p("POSE_ESTIMATOR_P3P");
	const string str_posit("POSE_ESTIMATOR_POSIT");
	const string str_planarposit("POSE_ESTIMATOR_PLANARPOSIT");

	string temp_str;
	fs["Marker_Mode"] >> temp_str;
	if(temp_str.compare(str_aruco_fid) == 0) {
		marker_mode = MarkerMode::ARUCO_FID;
	} else if(temp_str.compare(str_aruco_hrm) == 0) {
		marker_mode = MarkerMode::ARUCO_HRM;
	}

	// pose estimation algorithm
	fs["Pose_Estimator"] >> temp_str;
	if(temp_str.compare(str_ppp) == 0) {
		pose_estimator = PoseEstimationAlgorithm::POSE_PPP;
	} else if(temp_str.compare(str_rpp) == 0) {
		pose_estimator = PoseEstimationAlgorithm::POSE_RPP;
	} else if(temp_str.compare(str_iterative) == 0) {
		pose_estimator = PoseEstimationAlgorithm::POSE_ITERATIVE;
	} else if(temp_str.compare(str_epnp) == 0) {
		pose_estimator = PoseEstimationAlgorithm::POSE_EPNP;
	} else if(temp_str.compare(str_p3p) == 0) {
		pose_estimator = PoseEstimationAlgorithm::POSE_P3P;
	} else if(temp_str.compare(str_posit) == 0) {
		pose_estimator = PoseEstimationAlgorithm::POSE_POSIT;
	} else if(temp_str.compare(str_planarposit) == 0) {
		pose_estimator = PoseEstimationAlgorithm::POSE_PLANARPOSIT;
	}

	fs["Marker_Normal_Size"] >> marker_size_mm;
	fs["Marker_Tolerance"] >> marker_tolerance;
	fs["Max_Error"] >> max_error;
	fs["Min_Line_Quality"] >> min_line_quality;

	const string str_track_vector("TRACK_VECTOR");
	const string str_track_kalman("TRACK_KALMAN");
	const string str_track_extkalman("TRACK_EXTKALMAN");
	// Tracking Algorithm
	fs["Tracker_Algorithm"] >> temp_str;
	if(temp_str.compare(str_track_vector) == 0) {
		tracker_algorithm = TrackerAlgorithm::TRACK_VECTOR;
	} else if(temp_str.compare(str_track_kalman) == 0) {
		tracker_algorithm = TrackerAlgorithm::TRACK_KALMAN;
		fs["Tracker_Kalman_Rk_x"] >> kalman_Rk_x;
		fs["Tracker_Kalman_Rk_theta"] >> kalman_Rk_theta;
		fs["Tracker_Kalman_Qk_x"] >> kalman_Qk_x;
		fs["Tracker_Kalman_Qk_theta"] >> kalman_Qk_theta;
	} /*else if(temp_str.compare(str_track_extkalman) == 0) {
		tracker_algorithm = TrackerAlgorithm::TRACK_EXTKALMAN; 	// not implemented because our problem is pretty linear
	}*/

	cv::Mat special_marker_sizes;
	fs["Marker_Sizes"] >> special_marker_sizes;
	// read all non-standard marker sizes (for normal sizes see the tracker setting)
	for(int i = 0; i < special_marker_sizes.rows; ++i) {
		// each row holds id, size_x, size_y as float
		const float* sizes_row = special_marker_sizes.ptr<float>(i);
		special_marker_sizes_mm.insert(pair<int, float>((int)sizes_row[0], sizes_row[1]));
	}

	// effectively used markers for positioning (rest is only used for evaluation
	cv::Mat markers_for_positioning;	// relevant in evaluation
	fs["Marker_forPos"] >> markers_for_positioning;
	for(int i = 0; i < markers_for_positioning.rows; ++i) {
		pos_uids.insert(markers_for_positioning.at<int>(i,0));
	}

	fs.release();
}

TrackerParameters::~TrackerParameters() {
}

} /* namespace fml */
