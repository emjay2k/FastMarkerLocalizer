/*
 * config.cpp
 *
 *  Created on: 12.09.2014
 *      Author: jung
 */

#include "configure/config.h"

namespace fml {

double Config::time_ms_factor;
CalibrationParameters Config::calibration_parameters;
CameraParameters Config::camera_parameters;
MarkerParameters Config::marker_parameters;
MarkerParameters Config::marker_parameters_gt;
TrackerParameters Config::tracker_parameters;

void Config::setConfigs(const CalibrationParameters &calibration_params, const CameraParameters &camera_params, const MarkerParameters &marker_params, const MarkerParameters &marker_params_gt, const TrackerParameters &tracker_params) {
	calibration_parameters = calibration_params;
	camera_parameters = camera_params;
	marker_parameters = marker_params;
	marker_parameters_gt = marker_params_gt;
	tracker_parameters = tracker_params;

	time_ms_factor = 1000000.0/cv::getTickFrequency();
}

} /* namespace fml */
