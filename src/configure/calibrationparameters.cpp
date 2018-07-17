/*
 * calibrationparameters.cpp
 *
 *  Created on: 12.09.2014
 *      Author: jung
 */

#include "configure/calibrationparameters.h"

namespace fml {

CalibrationParameters::CalibrationParameters() {
	camera_size = cv::Size();
	camera_matrix = cv::Mat::eye(3, 3, MAT_TYPE);
	distortion_coefficients = cv::Mat::zeros(5, 1, MAT_TYPE);
	rfx_ = 0;
	rfy_ = 0;
}

CalibrationParameters::CalibrationParameters(const string &filename) {
	camera_size = cv::Size();
	camera_matrix = cv::Mat::eye(3, 3, MAT_TYPE);
	distortion_coefficients = cv::Mat::zeros(5, 1, MAT_TYPE);

	cv::FileStorage fs(filename, cv::FileStorage::READ);
	fs["Camera_Matrix"] >> camera_matrix;
	fs["Distortion_Coefficients"] >> distortion_coefficients;
	fs["Input_Width"] >> camera_size.width;
	fs["Input_Height"] >> camera_size.height;
	fs.release();

	f.x = camera_matrix.at<float>(0,0);
	c.x = camera_matrix.at<float>(0,2);
	f.y = camera_matrix.at<float>(1,1);
	c.y = camera_matrix.at<float>(1,2);
	rfx_ = 1.0f/f.x;
	rfy_ = 1.0f/f.y;
	for(size_t i = 0; i < 4; ++i) {
		k_[i] = distortion_coefficients.at<float>(i,0);
	}
}

CalibrationParameters::~CalibrationParameters() {
}

// recompute camera matrix for specific size
void CalibrationParameters::resize(cv::Size size) {
	float scaleX = (float)(size.width)/(float)(camera_size.width);
	float scaleY = (float)(size.height)/(float)(camera_size.height);
	camera_matrix.at<float>(0,0) *= scaleX;
	camera_matrix.at<float>(0,2) *= scaleX;
	camera_matrix.at<float>(1,1) *= scaleY;
	camera_matrix.at<float>(1,2) *= scaleY;

	f.x *= scaleX;
	c.x *= scaleX;
	f.y *= scaleY;
	c.y *= scaleY;
	rfx_ = 1.0f/f.x;
	rfy_ = 1.0f/f.y;
}

} /* namespace fml */
