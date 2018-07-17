/*
 * imageprocessor.cpp
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#include "imageprocessor.h"

namespace fml {

ImageProcessor::ImageProcessor() {
	if(!Config::camera_parameters.input_str.empty()) {
		string video_file(Config::camera_parameters.input_str.front());
		Config::camera_parameters.input_str.pop_front();
		capture_ = new TimestampedVideoCapture(video_file);
		is_live_ = false;
	} else {
#ifdef RPI
		capture_ = new raspicam::RaspiCam_Cv();
#else
		capture_ = new TimestampedCameraCapture(Config::camera_parameters.input);
		capture_->set(CV_CAP_PROP_FRAME_WIDTH, Config::camera_parameters.camera_size.width);
		capture_->set(CV_CAP_PROP_FRAME_HEIGHT, Config::camera_parameters.camera_size.height);
		capture_->set(CV_CAP_PROP_FPS, Config::camera_parameters.fps);
#endif
		is_live_ = true;
	}

	raw_image_timestamp_ = -1;
}

ImageProcessor::~ImageProcessor() {
	delete capture_;
}

int ImageProcessor::getRawImage(cv::Mat &image) {
	bool success = grab_helper(raw_image_) && raw_image_.cols == Config::camera_parameters.camera_size.width && raw_image_.rows == Config::camera_parameters.camera_size.height;
	if(success) {
		image = raw_image_.clone();
	} else {
		raw_image_timestamp_ = -1;
	}
	return(raw_image_timestamp_);
}

void ImageProcessor::setExposure(const double micro_seconds) {
	if(is_live_) {
		capture_->set(CV_CAP_PROP_EXPOSURE, micro_seconds);
	}
}

bool ImageProcessor::isLive() const {
	return(is_live_);
}

/**
 * Grab an image from the camera and convert it to grayscale if necessary
 *
 * @param image
 */
bool ImageProcessor::grab_helper(cv::Mat &image) {
	cv::Mat tmp;
	*capture_ >> tmp;
	raw_image_timestamp_ = capture_->get(CV_CAP_PROP_POS_MSEC);
	bool got_image = ! tmp.empty();
	if(got_image) {
		image = tmp;
	}
	return(got_image);
}

} /* namespace fml */
