/*
 * undistorter.cpp
 *
 *  Created on: 04.12.2015
 *      Author: jung
 */

#include "marker/undistorter.h"

namespace fml {

Undistorter::Undistorter() {
}

Undistorter::~Undistorter() {
}

/**
 * Code extracted from opencv source, just the stuff we need, to make it much faster
 *
 * @param input
 * @param output
 */
void Undistorter::undistortPoints(vector<cv::Point2f> &points) {
	size_t num_points = points.size();
	for(size_t i = 0; i < num_points; ++i) {
		float x = (points[i].x - Config::calibration_parameters.c.x) * Config::calibration_parameters.rfx_;
		float y = (points[i].y - Config::calibration_parameters.c.y) * Config::calibration_parameters.rfy_;
		float x0 = x, y0 = y;

		// compensate distortion iteratively
		for(size_t j = 0; j < 5; ++j) {
			float xx = x*x;
			float yy = y*y;
			float xy2 = x*y*2;
			float r2 = xx + yy;

			float icdist = 1.0f/(1.0f + r2*(Config::calibration_parameters.k_[0] + r2*Config::calibration_parameters.k_[1]));
			float dx = Config::calibration_parameters.k_[2]*xy2 + Config::calibration_parameters.k_[3]*(yy + 3*xx);
			float dy = Config::calibration_parameters.k_[3]*xy2 + Config::calibration_parameters.k_[2]*(xx + 3*yy);
			x = (x0 - dx) * icdist;
			y = (y0 - dy) * icdist;
		}

		points[i].x = Config::calibration_parameters.f.x * x + Config::calibration_parameters.c.x;
		points[i].y = Config::calibration_parameters.f.y * y + Config::calibration_parameters.c.y;
	}
}

void Undistorter::distortPoints(vector<cv::Point2f> &points) {
	size_t num_points = points.size();
    for(size_t i = 0; i < num_points; ++i) {
    	float x = (points[i].x - Config::calibration_parameters.c.x) * Config::calibration_parameters.rfx_;
    	float y = (points[i].y - Config::calibration_parameters.c.y) * Config::calibration_parameters.rfy_;

    	float xx = x*x;
    	float yy = y*y;
    	float xy2 = x*y*2;
    	float r2 = xx + yy;

    	float cdist = 1.0f + r2*(Config::calibration_parameters.k_[0] + r2*Config::calibration_parameters.k_[1]);
    	float xd = x*cdist + Config::calibration_parameters.k_[2]*xy2 + Config::calibration_parameters.k_[3]*(yy + 3*xx);
    	float yd = y*cdist + Config::calibration_parameters.k_[3]*xy2 + Config::calibration_parameters.k_[2]*(xx + 3*yy);

        points[i].x = Config::calibration_parameters.f.x * xd + Config::calibration_parameters.c.x;
        points[i].y = Config::calibration_parameters.f.y * yd + Config::calibration_parameters.c.y;
    }
}


/**
 * Code extracted from opencv source, just the stuff we need, to make it much faster
 *
 * @param object_points
 * @param image_points
 */
void Undistorter::projectPoints(const vector<cv::Point3f> &object_points, vector<cv::Point2f> &image_points) {
	size_t num_points = object_points.size();
	image_points.resize(num_points);
    for(size_t i = 0; i < num_points; ++i) {
        float z = 1.0f/object_points[i].z;
        float x = object_points[i].x * z;
        float y = object_points[i].y * z;

        float xx = x*x;
        float yy = y*y;
        float xy2 = x*y*2;
        float r2 = xx + yy;

        float cdist = 1.0f + r2*(Config::calibration_parameters.k_[0] + r2*Config::calibration_parameters.k_[1]);
        float xd = x*cdist + Config::calibration_parameters.k_[2]*xy2 + Config::calibration_parameters.k_[3]*(yy + 3*xx);
        float yd = y*cdist + Config::calibration_parameters.k_[3]*xy2 + Config::calibration_parameters.k_[2]*(xx + 3*yy);

        image_points[i].x = Config::calibration_parameters.f.x * xd + Config::calibration_parameters.c.x;
        image_points[i].y = Config::calibration_parameters.f.y * yd + Config::calibration_parameters.c.y;
    }
}

} /* namespace fml */
