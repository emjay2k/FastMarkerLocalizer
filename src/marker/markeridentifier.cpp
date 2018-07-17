/*
 * markeridentifier.cpp
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#include "marker/markeridentifier.h"

namespace fml {

MarkerIdentifier::MarkerIdentifier() {
}

MarkerIdentifier::~MarkerIdentifier() {
}

bool MarkerIdentifier::identify(const cv::Mat &image, vector<cv::Point2f> &candidate, MarkerInfo &marker) const {
	extractMarkerInfo(image, candidate, marker);
	if(marker.id != -1) {
		// assign length value
		marker.length_mm = Config::tracker_parameters.special_marker_sizes_mm.count(marker.id) ? Config::tracker_parameters.special_marker_sizes_mm.at(marker.id) : Config::tracker_parameters.marker_size_mm;
		return(true);
	} else {
		return(false);
	}
}

int MarkerIdentifier::identify(const cv::Mat &image, list<vector<cv::Point> > &candidates, list<MarkerInfo> &markers) const {
	markers.clear();
	if(!candidates.empty()) {
		extractMarkerInfo(image, candidates, markers);

		// assign length value
		for(list<MarkerInfo>::iterator it = markers.begin(); it != markers.end(); ++it) {
			it->length_mm = Config::tracker_parameters.special_marker_sizes_mm.count(it->id) ? Config::tracker_parameters.special_marker_sizes_mm.at(it->id) : Config::tracker_parameters.marker_size_mm;

			// TODO: check if this is really necessary
			// use this to improve refine corners
			/*cv::Rect bounding_box = getBoundingBox(it->vertices, cv::Size(image.cols, image.rows), 0);
				float desired_length = (float)(1.1 * sqrt_2 * 0.25) * computePerimeter(it->vertices);
				float actual_length = min(bounding_box.width, bounding_box.height);
				bounding_box = getBoundingBox(it->vertices, cv::Size(image.cols, image.rows), 0.5f*(desired_length-actual_length));
				cv::Mat normalize_marker = cv::Mat(image, bounding_box);
				fastNormalize(normalize_marker, normalize_marker);*/
		}
	}
	return(markers.size());
}

// copied from opencv
/* Calculates coefficients of perspective transformation
 * which maps (xi,yi) to (ui,vi), (i=1,2,3,4):
 *
 *      c00*xi + c01*yi + c02
 * ui = ---------------------
 *      c20*xi + c21*yi + c22
 *
 *      c10*xi + c11*yi + c12
 * vi = ---------------------
 *      c20*xi + c21*yi + c22
 *
 * Coefficients are calculated by solving linear system:
 * / x0 y0  1  0  0  0 -x0*u0 -y0*u0 \ /c00\ /u0\
 * | x1 y1  1  0  0  0 -x1*u1 -y1*u1 | |c01| |u1|
 * | x2 y2  1  0  0  0 -x2*u2 -y2*u2 | |c02| |u2|
 * | x3 y3  1  0  0  0 -x3*u3 -y3*u3 |.|c10|=|u3|,
 * |  0  0  0 x0 y0  1 -x0*v0 -y0*v0 | |c11| |v0|
 * |  0  0  0 x1 y1  1 -x1*v1 -y1*v1 | |c12| |v1|
 * |  0  0  0 x2 y2  1 -x2*v2 -y2*v2 | |c20| |v2|
 * \  0  0  0 x3 y3  1 -x3*v3 -y3*v3 / \c21/ \v3/
 *
 * where:
 *   cij - matrix coefficients, c22 = 1
 */
cv::Mat MarkerIdentifier::getPerspectiveTransform(const cv::Point2f (&src)[4], const cv::Point2f (&dst)[4]) const {
	cv::Mat M(3, 3, CV_32F), X(8, 1, CV_32F, M.data);
	float a[8][8], b[8];
	cv::Mat A(8, 8, CV_32F, a), B(8, 1, CV_32F, b);

	for(int i = 0; i < 4; ++i) {
		a[i][0] = a[i+4][3] = src[i].x;
		a[i][1] = a[i+4][4] = src[i].y;
		a[i][2] = a[i+4][5] = 1;
		a[i][3] = a[i][4] = a[i][5] =
				a[i+4][0] = a[i+4][1] = a[i+4][2] = 0;
		a[i][6] = -src[i].x*dst[i].x;
		a[i][7] = -src[i].y*dst[i].x;
		a[i+4][6] = -src[i].x*dst[i].y;
		a[i+4][7] = -src[i].y*dst[i].y;
		b[i] = dst[i].x;
		b[i+4] = dst[i].y;
	}

	solve(A, B, X);	// default = LU decomposition works fine in our case and is much faster
	((float*)M.data)[8] = 1.;

	return M;
}

/**
 * This was implemented in the hopes that it would be faster than OpenCV's version. It is faster, but only a little bit.
 *
 * @param src
 * @param dst
 * @param transform
 */
void MarkerIdentifier::warpPerspective(const cv::Mat &src, cv::Mat &dst, cv::Mat transform, const uint8_t border_value) const {
	invert(transform, transform, cv::DECOMP_CHOLESKY);
	dst = cv::Mat(marker_warp_size_, marker_warp_size_, CV_8UC1);
	const float* M = transform.ptr<float>(0);
	for(int i = 0; i < dst.rows; ++i) {
		uint8_t* p = dst.data + dst.step*i;
		float temp1 = M[1] * i + M[2];
		float temp2 = M[4] * i + M[5];
		float temp3 = M[7] * i + M[8];
		for(int j = 0; j < dst.cols; ++j) {
			float q = 1.0f/temp3;
			int x = q * temp1 + 0.5f;	// row
			int y = q * temp2 + 0.5f;	// col
			p[j] = (likely(x >= 0 && x < src.cols && y >= 0 && y < src.rows)) ? src.at<uint8_t>(y,x) : border_value;

			temp1 += M[0];
			temp2 += M[3];
			temp3 += M[6];
		}
	}
}

//obtain and apply the perspective transform
void MarkerIdentifier::warp_helper(const cv::Mat &src, cv::Mat &dst, const vector<cv::Point2f> &points) const {
	cv::Point2f src_points[4] = { points[0], points[1], points[2], points[3] };
	cv::Point2f dst_points[4] = { cv::Point2f(0, 0), cv::Point2f(marker_warp_size_-1, 0), cv::Point2f(marker_warp_size_-1, marker_warp_size_-1), cv::Point2f(0, marker_warp_size_-1) };
	warpPerspective(src, dst, getPerspectiveTransform(src_points, dst_points), 128);
	//cv::warpPerspective(src, dst, getPerspectiveTransform(src_points, dst_points), marker_warp_size_, cv::INTER_NEAREST);
}

} /* namespace fml */
