/**
 * MeanAutoExposureAlgorithm.cpp
 *
 *	Determines new exposure time based on the mean pixel value
 *
 *  Created on: 07.12.2012
 *      Author: jung
 */

#include "matrixautoexposurealgorithm.h"

namespace fml {

MatrixAutoExposureAlgorithm::MatrixAutoExposureAlgorithm(const float min_val, const float max_val) : AutoExposureAlgorithm(min_val, max_val) {
	//target_average_ = 96.0f;
	target_average_ = 128.0f;
}

MatrixAutoExposureAlgorithm::~MatrixAutoExposureAlgorithm() {
}

uint32_t MatrixAutoExposureAlgorithm::computeSum(const cv::Mat &image) const {
	uint32_t sum = 0;
	for(int i = 0; i < image.rows; ++i) {
		const uint8_t *row = image.ptr<uint8_t>(i);
		int j = 0;
		for( ; j < image.cols-3; j += 4) {
			int s0 = row[0], s1 = row[1], s2 = row[2], s3 = row[3];
			int x0 = s0 + s1 + s2 + s3;
			sum += x0;

			row += 4;
		}
		for( ; j < image.cols-3; ++j) {
			sum += row[0];

			++row;
		}
	}
	return(sum);
}

float MatrixAutoExposureAlgorithm::calcError(const cv::Mat &image, list<cv::Rect> &rois) const {
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
		int row_center_dist = image.rows/num_row_samples_;
		int col_center_dist = image.cols/num_col_samples_;
		// size of the square sampled at each position
		int size = min(row_center_dist, col_center_dist) >> 1;
		int half_size = size >> 1;
		// distance from top left corner
		int row_start = (row_center_dist >> 1) - half_size;
		int col_start = (col_center_dist >> 1) - half_size;

		uint32_t sum = 0;
		for(int i = 0; i < num_row_samples_; ++i) {
			int y = row_start + i * row_center_dist;
			for(int j = 0; j < num_col_samples_; ++j) {
				int x = col_start + j * col_center_dist;
				cv::Mat tmp(image, cv::Rect(x, y, size, size));
				sum += computeSum(tmp);
			}
		}
		// devide by the number of pixels number of squares * square_size
		return(1.0f - (float)sum / ((num_row_samples_ * num_col_samples_ * size * size) * target_average_));
	}
}

float MatrixAutoExposureAlgorithm::getTargetValue() const {
	return(target_average_);
}

void MatrixAutoExposureAlgorithm::setTargetValue(const float value) {
	target_average_ = value;
}

}
