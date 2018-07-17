/*
 * TimestampedCameraCapture.h
 *
 *  Created on: 26.06.2015
 *      Author: jung
 */

#ifndef SRC_TIMESTAMPEDCAMERACAPTURE_H_
#define SRC_TIMESTAMPEDCAMERACAPTURE_H_

#ifndef RPI
#include <opencv2/opencv.hpp>

using namespace std;

namespace fml {

class TimestampedCameraCapture : public cv::VideoCapture {

	int last_timestamp_;
	double ms_factor_;

public:
	TimestampedCameraCapture(const int index);
	virtual ~TimestampedCameraCapture();

	bool grab();
	double get(const int propId);
};

} /* namespace fml */
#endif
#endif /* SRC_TIMESTAMPEDCAMERACAPTURE_H_ */
