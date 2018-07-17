/**
 * AutoSensorAlgorithm.cpp
 *
 *  Created on: 07.12.2012
 *      Author: jung
 */

#include "autosensoralgorithm.h"

namespace fml {

AutoSensorAlgorithm::AutoSensorAlgorithm(const float min_val, const float max_val) {
	min_limit_ = min_val;
	max_limit_ = max_val;
	min_value_ = min_limit_;
	max_value_ = max_limit_;
	cur_value_ = min_limit_;
}

AutoSensorAlgorithm::~AutoSensorAlgorithm() {
}

void AutoSensorAlgorithm::reset() {
	min_value_ = min_limit_;
	max_value_ = max_limit_;
}

float AutoSensorAlgorithm::getMin() const {
	return(min_value_);
}

float AutoSensorAlgorithm::getMax() const {
	return(max_value_);
}

float AutoSensorAlgorithm::getValue() const {
	return(cur_value_);
}

bool AutoSensorAlgorithm::setMin(const float new_min) {
	if(new_min > min_limit_) {
		min_value_ = min(new_min, max_limit_);
		return(true);
	} else {
		return(false);
	}
}

bool AutoSensorAlgorithm::setMax(const float new_max) {
	if(new_max < max_limit_) {
		max_value_ = max(new_max, min_limit_);
		return(true);
	} else {
		return(false);
	}
}

float AutoSensorAlgorithm::setValue(const float value) {
	if(value > max_limit_) cur_value_ = max_limit_;
	else if(value < min_limit_)	cur_value_ = min_limit_;
	else cur_value_ = value;
	return(cur_value_);
}

bool AutoSensorAlgorithm::run(const cv::Mat &image, list<cv::Rect> &rois) {
	return(update(image, rois));
}

}
