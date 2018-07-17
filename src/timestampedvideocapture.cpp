/*
 * TimestampedVideoCapture.cpp
 *
 *  Created on: 26.06.2015
 *      Author: jung
 */

#include "timestampedvideocapture.h"

namespace fml {

TimestampedVideoCapture::TimestampedVideoCapture(const string &video_file) : VideoCapture(video_file) {
	index_ = 0;
	size_t pos = video_file.rfind('/');
	string timestamp_file;
	if(pos != string::npos) {
		timestamp_file.append(video_file.substr(0, pos+1));
	}
	timestamp_file.append("timestamps.yml");
	fs_.open(timestamp_file, cv::FileStorage::READ);
	fs_["timestamps"] >> timestamps_;
	fs_.release();
}

TimestampedVideoCapture::TimestampedVideoCapture(const string &video_file, const string &timestamp_file) : VideoCapture(video_file) {
	index_ = 0;
	fs_.open(timestamp_file, cv::FileStorage::READ);
	fs_["timestamps"] >> timestamps_;
	fs_.release();
}

TimestampedVideoCapture::~TimestampedVideoCapture() {
	release();
}

bool TimestampedVideoCapture::grab() {
	bool retval = VideoCapture::grab();
	++index_;
	return(retval);
}

double TimestampedVideoCapture::get(const int propId) {
	if(propId == CV_CAP_PROP_POS_MSEC) {
		return((timestamps_.rows > index_) ? timestamps_.at<int32_t>(index_, 0) : INT_MAX);
	} else return(VideoCapture::get(propId));
}

} /* namespace fml */
