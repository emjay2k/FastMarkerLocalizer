/*
 * imageprocessor.h
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#ifndef SRC_IMAGEPROCESSOR_H_
#define SRC_IMAGEPROCESSOR_H_

#include <opencv2/opencv.hpp>
#include "timestampedcameracapture.h"
#include "timestampedvideocapture.h"
#include "configure/config.h"
#ifdef RPI
#include <raspicam/raspicam_cv.h>
#endif

using namespace std;

namespace fml {

class ImageProcessor {
	bool is_live_;
	int raw_image_timestamp_;
	cv::Mat raw_image_;

	cv::VideoCapture *capture_;

	bool grab_helper(cv::Mat &image);

public:
	ImageProcessor();
	virtual ~ImageProcessor();

	void setExposure(const double micro_seconds);
	virtual bool isLive() const;
	virtual int getRawImage(cv::Mat &image);
};

} /* namespace fml */

#endif /* SRC_IMAGEPROCESSOR_H_ */
