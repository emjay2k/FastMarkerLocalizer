/*
 * undistorter.h
 *
 *  Created on: 04.12.2015
 *      Author: jung
 */

#ifndef SRC_MARKER_UNDISTORTER_H_
#define SRC_MARKER_UNDISTORTER_H_

#include <opencv2/opencv.hpp>
#include <configure/config.h>

using namespace std;

namespace fml {

class Undistorter {

public:
	Undistorter();
	virtual ~Undistorter();

	static void undistortPoints(vector<cv::Point2f> &points);
	static void distortPoints(vector<cv::Point2f> &points);
	static void projectPoints(const vector<cv::Point3f> &object_points, vector<cv::Point2f> &image_points);
};

} /* namespace fml */

#endif /* SRC_MARKER_UNDISTORTER_H_ */
