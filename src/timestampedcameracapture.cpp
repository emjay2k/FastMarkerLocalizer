/*
 * TimestampedCameraCapture.cpp
 *
 *  Created on: 26.06.2015
 *      Author: jung
 */

#ifndef RPI
#include "timestampedcameracapture.h"

namespace fml {

TimestampedCameraCapture::TimestampedCameraCapture(const int index) : VideoCapture(index) {
	last_timestamp_ = 0;
	ms_factor_ = 1000.0/cv::getTickFrequency();
}

TimestampedCameraCapture::~TimestampedCameraCapture() {
}

bool TimestampedCameraCapture::grab() {
	bool retval = VideoCapture::grab();
	int now_soft = (double)cv::getTickCount() * ms_factor_;
	int now_hard = VideoCapture::get(CV_CAP_PROP_POS_MSEC);
	last_timestamp_ = (now_hard < 0) ? now_soft : now_hard;

	return(retval);
}

double TimestampedCameraCapture::get(const int propId) {
	if(propId == CV_CAP_PROP_POS_MSEC) {
		return(last_timestamp_);
	} else return(VideoCapture::get(propId));
}

} /* namespace fml */
#endif
