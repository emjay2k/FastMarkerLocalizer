/*
 * cameraparameters.cpp
 *
 *  Created on: 12.09.2014
 *      Author: jung
 */

#include "configure/cameraparameters.h"

namespace fml {

CameraParameters::CameraParameters() {
	input = 0;
	camera_size = cv::Size();
	fps = 15;
	pixel_speedlimit = 5;
	show_image = false;
	undistort = 0;
	x_offset = 0;
	y_offset = 0;
	z_offset = 0;
	angle_offset = 0;
}

CameraParameters::CameraParameters(const string &filename) {
	input = 0;
	camera_size = cv::Size();
	fps = 15;
	pixel_speedlimit = 5;
	show_image = false;
	undistort = 0;
	x_offset = 0;
	y_offset = 0;
	z_offset = 0;
	angle_offset = 0;

	cv::FileStorage fs(filename, cv::FileStorage::READ);
	fs["Input"] >> input;
	cv::FileNode n = fs["Input_Str"];
	if(n.type() == cv::FileNode::SEQ) {
		for(cv::FileNodeIterator it = n.begin(); it != n.end(); ++it) {
			input_str.push_back((string)*it);
		}
	} else if(n.type() == cv::FileNode::STRING){
		string temp;
		fs["Input_Str"] >> temp;
		input_str.push_back(temp);
	}

	fs["Input_FramesPerSecond"] >> fps;
	fs["Input_Width"] >> camera_size.width;
	fs["Input_Height"] >> camera_size.height;
	fs["Undistort"] >> undistort;
	fs["Pixel_Speedlimit"] >> pixel_speedlimit;
	fs["Show_Image"] >> show_image;
	fs["X_Offset"] >> x_offset;
	fs["Y_Offset"] >> y_offset;
	fs["Z_Offset"] >> z_offset;
	fs["Angle_Offset"] >> angle_offset;
	fs.release();
}

CameraParameters::~CameraParameters() {
}

} /* namespace fml */
