/**
 * MeanAutoExposureAlgorithm.cpp
 *
 *	Determines new exposure time based on the mean pixel value
 *
 *  Created on: 07.12.2012
 *      Author: jung
 */

#include "meanautoexposurealgorithm.h"

namespace fml {

MeanAutoExposureAlgorithm::MeanAutoExposureAlgorithm(const float min_val, const float max_val) : AutoExposureAlgorithm(min_val, max_val) {
	target_average_ = 128.0f;
}

MeanAutoExposureAlgorithm::~MeanAutoExposureAlgorithm() {
}

uint32_t MeanAutoExposureAlgorithm::computeSum(const cv::Mat &image) const {
	cv::Size size = image.size();
	int num_pixel = size.height * size.width;
	if(image.isContinuous()) {
		size.width = num_pixel;
		size.height = 1;
	}

	uint32_t sum = 0;
	for(int i = 0; i < size.height; ++i) {
		const uint8_t* p = image.data + image.step*i;
		int j = 0;
		for( ; j < size.width-3; j += 4) {
			uint32_t v0 = p[0], v1 = p[1], v2 = p[2], v3 = p[3];
			uint32_t tmp = v0 + v1 + v2 + v3;
			sum += tmp;

			p += 4;
		}
		for( ; j < size.width; ++j) {
			sum += p[0];

			++p;
		}
	}
	return(sum);
}

float MeanAutoExposureAlgorithm::calcError(const cv::Mat &image, list<cv::Rect> &rois) const {
	if(!rois.empty()) {
		int num_pixels = 0;
		uint32_t sum = 0;
		for(list<cv::Rect>::const_iterator it = rois.begin(); it != rois.end(); ++it) {
			cv::Mat tmp(image, *it);
			sum += computeSum(tmp);
			num_pixels += (it->width * it->height);
		}
		return(1.0f - sum / (num_pixels * target_average_));
	} else {
		return(1.0f - computeSum(image) / ((image.cols * image.rows) * target_average_));
	}
}

float MeanAutoExposureAlgorithm::getTargetValue() const {
	return(target_average_);
}

void MeanAutoExposureAlgorithm::setTargetValue(const float value) {
	target_average_ = value;
}

}
