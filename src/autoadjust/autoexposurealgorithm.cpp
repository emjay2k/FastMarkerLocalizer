/**
 * AutoExposureAlgorithm.cpp
 *
 *	Computes new exposure time for AE
 *
 *  Created on: 07.12.2012
 *      Author: jung
 */

#include "autoexposurealgorithm.h"

namespace fml {

AutoExposureAlgorithm::AutoExposureAlgorithm(const float min_val, const float max_val) : AutoSensorAlgorithm(min_val, max_val) {
	exposure_threshold_ = 1.05f;
}

AutoExposureAlgorithm::~AutoExposureAlgorithm() {
}

bool AutoExposureAlgorithm::update(const cv::Mat &image, list<cv::Rect> &rois) {
	// calculate new exposureTime from previous one
	float next_exposure = EXP(2.0f * calcError(image, rois)) * cur_value_;

	bool overflow = false;
	if(next_exposure > max_value_) {
		next_exposure = max_value_;
		overflow = true;
	} else if(next_exposure < min_value_) {
		next_exposure = min_value_;
		overflow = true;
	}

	float factor = next_exposure > cur_value_ ? next_exposure/cur_value_ : cur_value_/next_exposure;
	if(factor >= exposure_threshold_ || (overflow && next_exposure != cur_value_)) {
		cur_value_ = next_exposure;
		return(true);
	} else {
		return(false);
	}
}

float AutoExposureAlgorithm::getThreshold() const {
	return(exposure_threshold_);
}

void AutoExposureAlgorithm::setThreshold(const float value) {
	exposure_threshold_ = value;
}

}
