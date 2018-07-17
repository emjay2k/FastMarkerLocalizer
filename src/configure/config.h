/*
 * config.h
 *
 *  Created on: 12.09.2014
 *      Author: jung
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "calibrationparameters.h"
#include "cameraparameters.h"
#include "markerparameters.h"
#include "trackerparameters.h"

using namespace std;

namespace fml {

class Config {

public:

	static double time_ms_factor;
	static CalibrationParameters calibration_parameters;
	static CameraParameters camera_parameters;
	static MarkerParameters marker_parameters;
	static MarkerParameters marker_parameters_gt;
	static TrackerParameters tracker_parameters;

	static void setConfigs(const CalibrationParameters &calibration_params, const CameraParameters &camera_params, const MarkerParameters &marker_params, const MarkerParameters &marker_params_gt, const TrackerParameters &tracker_params);
};

} /* namespace fml */
#endif /* CONFIG_H_ */
