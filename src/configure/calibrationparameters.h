/*
 * calibrationparameters.h
 *
 *  Created on: 12.09.2014
 *      Author: jung
 */

#ifndef CALIBRATIONPARAMETERS_H_
#define CALIBRATIONPARAMETERS_H_

#include <string>
#include <opencv2/opencv.hpp>
#include "generic/definitions.h"

using namespace std;

namespace fml {

class CalibrationParameters {

public:
	float rfx_;
	float rfy_;
	float k_[5];
	cv::Point2f f;
	cv::Point2f c;
	cv::Size camera_size;
	cv::Mat camera_matrix;
	cv::Mat distortion_coefficients;

	CalibrationParameters();
	CalibrationParameters(const string &filename);
	virtual ~CalibrationParameters();

	void resize(cv::Size size);
};

} /* namespace fml */
#endif /* CALIBRATIONPARAMETERS_H_ */
