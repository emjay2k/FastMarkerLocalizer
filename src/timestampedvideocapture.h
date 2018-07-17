/*
 * TimestampedVideoCapture.h
 *
 *  Created on: 26.06.2015
 *      Author: jung
 */

#ifndef SRC_TIMESTAMPEDVIDEOCAPTURE_H_
#define SRC_TIMESTAMPEDVIDEOCAPTURE_H_

#include <opencv2/opencv.hpp>

using namespace std;

namespace fml {

class TimestampedVideoCapture : public cv::VideoCapture {

	cv::FileStorage fs_;
	cv::Mat timestamps_;
	int index_;

public:
	TimestampedVideoCapture(const string &video_file);
	TimestampedVideoCapture(const string &video_file, const string &timestamp_file);
	virtual ~TimestampedVideoCapture();

	bool grab();
	double get(const int propId);
};

} /* namespace fml */

#endif /* SRC_TIMESTAMPEDVIDEOCAPTURE_H_ */
