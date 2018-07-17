/*
 * cameraparameters.h
 *
 *  Created on: 12.09.2014
 *      Author: jung
 */

#ifndef CAMERAPARAMETERS_H_
#define CAMERAPARAMETERS_H_

#include <string>
#include <opencv2/opencv.hpp>
#include "generic/definitions.h"

using namespace std;

namespace fml {

class CameraParameters {

public:
	int input;
	list<string> input_str;
	float fps;
	cv::Size camera_size;
	bool show_image;
	int undistort;
	float pixel_speedlimit; // max speed in pixels/frame
	float x_offset;	// transformation to vehicle coordinates x component
	float y_offset;	// transformation to vehicle coordinates y component
	float z_offset;	// how far is the camera mounted above ground
	float angle_offset;	// transformation to vehicle coordinates yaw angle component

	CameraParameters();
	CameraParameters(const string &filename);
	virtual ~CameraParameters();
};

} /* namespace fml */

#endif /* CAMERAPARAMETERS_H_ */
