/*
 * trackerparameters.h
 *
 *  Created on: 12.09.2014
 *      Author: jung
 */

#ifndef TRACKERPARAMETERS_H_
#define TRACKERPARAMETERS_H_

#include <map>
#include <set>
#include <string>
#include <opencv2/opencv.hpp>
#include "generic/definitions.h"
#include "generic/types.h"

using namespace std;

namespace fml {

class TrackerParameters {

public:
	bool demo;
	bool evaluation;

	bool network;
	int network_id;
	string network_ip;
	int network_port;

	bool mapping;
	int mapping_startid;
	int mapping_minmeasurements;

	bool prediction;
	TrackerAlgorithm tracker_algorithm;
	float kalman_Rk_x;
	float kalman_Rk_theta;
	float kalman_Qk_x;
	float kalman_Qk_theta;

	MarkerMode marker_mode;
	bool correction;
	int split_detection;
	int marker_limit;
	float marker_size_mm;
	float marker_tolerance;
	float max_error;
	float min_line_quality;
	map<int, float> special_marker_sizes_mm;	// size of markers (x,y) in mm mapped to id
	set<int> pos_uids;

	PoseEstimationAlgorithm pose_estimator;

	TrackerParameters();
	TrackerParameters(const string &filename);
	virtual ~TrackerParameters();
};

} /* namespace fml */

#endif /* TRACKERPARAMETERS_H_ */
