/*
 * markerextractor.cpp
 *
 *  Created on: 01.08.2015
 *      Author: Jung
 */

#include "marker/newroimarkerextractor.h"

namespace fml {

NewRoiMarkerExtractor::NewRoiMarkerExtractor() {
	lb_factor_ = 1 - Config::tracker_parameters.marker_tolerance;
	ub_factor_ = 1 + Config::tracker_parameters.marker_tolerance;

	//const int ksize = 5;
	//f_gb_ = cv::createGaussianFilter(CV_8UC1, cv::Size(ksize,ksize), 0, 0, cv::BORDER_DEFAULT);
	// fill lookup table for static addWeighted in unsharp mask
	const float unsharp_weight = 0.5f;
	float LUT1[256], LUT2[256];
	float f1 = 1.0f + unsharp_weight;
	float f2 = -unsharp_weight;
	// compute lookup table helper
	for(int i = 0; i < 256; ++i) {
		LUT1[i] = f1 * i;
		LUT2[i] = f2 * i;
	}
	for(int i = 0; i < 256; ++i) {
		for(int j = 0; j < 256; ++j) {
			addWeighted_LUT[i][j] = cv::saturate_cast<uint8_t>(LUT1[i] + LUT2[j]);
		}
	}
	num_frames = 0;
	no_candidate_ = 0;

	stage1_ = 0;
	stage2_ = 0;
	stage3_ = 0;
}

NewRoiMarkerExtractor::~NewRoiMarkerExtractor() {
	//f_gb_.release();
	cerr << "Stage: " << stage1_ << ", " << stage2_ << ", " << stage3_ << endl;
	cerr << "There were no candidates: " << no_candidate_ << endl;
}

void NewRoiMarkerExtractor::fastGaussianBlur5(const cv::Mat &src, cv::Mat &dst) {
	const int k0 = (256.0f * 0.375f);
	const int k1 = (256.0f * 0.25f);
	const int k2 = (256.0f * 0.0625f);
	dst.create(src.rows, src.cols, src.type());
	cv::Mat tmp(src.rows, src.cols, CV_32S);
	int step_size = tmp.step/sizeof(int);

	int num_middle_rows = src.rows - 4;
	int num_middle_cols = src.cols - 4;

	// first the columns
	const uint8_t *sdata = src.data;
	int *tmp_data = (int*)tmp.data;
	for(int i = 0; i < src.rows; ++i) {
		int e0, e1, e2, e3, e4;
		// first element
		e0 = *sdata; ++sdata;
		e1 = *sdata; ++sdata;
		e2 = *sdata; ++sdata;
		*tmp_data = k0*e0 + 2*(k1*e1 + k2*e2); ++tmp_data;
		// second element
		e3 = *sdata; ++sdata;
		*tmp_data = k0*e1 + k1*(e0+e2) + 2*k2*e3; ++tmp_data;
		// middle elements
		int j = 0;
		for(; j < num_middle_cols-3; j += 4) {
			// read next 4 elements
			e4 = *sdata;
			int e5 = *(sdata+1);
			int e6 = *(sdata+2);
			int e7 = *(sdata+3);

			// determine next 4 temp values
			*tmp_data = k0*e2 + k1*(e1+e3) + k2*(e0+e4);
			*(tmp_data+1) = k0*e3 + k1*(e2+e4) + k2*(e1+e5);
			*(tmp_data+2) = k0*e4 + k1*(e3+e5) + k2*(e2+e6);
			*(tmp_data+3) = k0*e5 + k1*(e4+e6) + k2*(e3+e7);

			e0 = e4, e1 = e5, e2 = e6, e3 = e7;
			sdata += 4; tmp_data += 4;
		}
		for(; j < num_middle_cols; ++j) {
			e4 = *sdata;
			*tmp_data = k0*e2 + k1*(e1+e3) + k2*(e0+e4);
			e0 = e1, e1 = e2, e2 = e3, e3 = e4;
			++sdata; ++tmp_data;
		}
		// last-1 element
		*tmp_data = k0*e3 + k1*(e2+e4) + 2*k2*e1;
		++tmp_data;
		// last element
		*tmp_data = k0*e4 + 2*(k1*e3 + k2*e2);
		++tmp_data;	// first in next line
	}

	uint8_t *ddata = dst.data;
	int *r0 = (int*)tmp.data;
	int *r1 = r0 + step_size;
	int *r2 = r1 + step_size;
	int *r3, *r4;
	// first row
	for(int j = 0; j < src.cols; ++j) {
		int e0 = *r0, e1 = *r1, e2 = *r2;
		*ddata = (k0*e0 + 2*(k1*e1 + k2*e2)) >> 16;
		++r0, ++r1, ++r2, ++ddata;
	}
	// second row
	r3 = r2; r2 = r1; r1 = r0, r0 = (int*)tmp.data;
	for(int j = 0; j < src.cols; ++j) {
		int e0 = *r0, e1 = *r1, e2 = *r2, e3 = *r3;
		*ddata = (k0*e1 + k1*(e0+e2) + 2*k2*e3) >> 16;
		++r0, ++r1, ++r2, ++r3, ++ddata;
	}

	// middle rows
	r4 = r3, r3 = r2; r2 = r1; r1 = r0, r0 = (int*)tmp.data;
	int num_elements = num_middle_rows * src.cols;
	int i = 0;
	for(; i < num_elements-1; i += 2) {
		int e0 = *r0, e1 = *r1, e2 = *r2, e3 = *r3, e4 = *r4;
		int f0 = *(r0+1), f1 = *(r1+1), f2 = *(r2+1), f3 = *(r3+1), f4 = *(r4+1);
		*ddata = (k0*e2 + k1*(e1+e3) + k2*(e0+e4)) >> 16;
		*(ddata+1) = (k0*f2 + k1*(f1+f3) + k2*(f0+f4)) >> 16;

		r0 += 2; r1 += 2; r2 += 2; r3 += 2; r4 += 2; ddata += 2;
	}
	if(i < num_elements) {
		int e0 = *r0, e1 = *r1, e2 = *r2, e3 = *r3, e4 = *r4;
		*ddata = (k0*e2 + k1*(e1+e3) + k2*(e0+e4)) >> 16;
		++r0, ++r1, ++r2, ++r3, ++r4, ++ddata;
	}

	// last-1 row
	for(int j = 0; j < src.cols; ++j) {
		int e0 = *r0, e1 = *r1, e2 = *r2, e3 = *r3;
		*ddata = (k0*e2 + k1*(e1+e3) + 2*k2*e0) >> 16;
		++r0, ++r1, ++r2, ++r3, ++ddata;
	}

	// last row
	for(int j = 0; j < src.cols; ++j) {
		int e0 = *r0, e1 = *r1, e2 = *r2;
		*ddata = (k0*e2 + 2*(k1*e1 + k2*e0)) >> 16;
		++r0, ++r1, ++r2, ++ddata;
	}
}

/**
 * This implementation is twice as fast as the opencv version, however it does not contribute much to speedup, due to the low impact of this function
 *
 * @param m1
 * @param m2
 */
void NewRoiMarkerExtractor::fastUnsharpMask(cv::Mat &image) {
	// gaussian blur
	//cv::Mat blurred(image.size(), image.type());
	//f_gb_->apply(image, blurred);
	cv::Mat blurred;
	fastGaussianBlur5(image,  blurred);

	// apply lookup table
	int num_pixel = image.rows * image.cols;
	uint8_t *src1 = image.data;
	uint8_t *src2 = blurred.data;
	int i = 0;
	for(; i < num_pixel-3; i += 4) {
		int v0 = src1[0], v1 = src1[1];
		int b0 = src2[0], b1 = src2[1];
		uint8_t x0 = addWeighted_LUT[v0][b0], x1 = addWeighted_LUT[v1][b1];
		src1[0] = x0, src1[1] = x1;

		v0 = src1[2], v1 = src1[3];
		b0 = src2[2], b1 = src2[3];
		x0 = addWeighted_LUT[v0][b0], x1 = addWeighted_LUT[v1][b1];
		src1[2] = x0, src1[3] = x1;

		src1 += 4;
		src2 += 4;
	}
	for( ; i < num_pixel; ++i) {
		int v = src1[0];
		int b = src2[0];
		uint8_t x = addWeighted_LUT[v][b];
		src1[0] = x;

		++src1;
		++src2;
	}
}

bool NewRoiMarkerExtractor::getLine(cv::Mat &image, forward_list<PointWithDist> &points, const cv::Point2f &roi_tl, const float max_difference, cv::Vec4f &line) {
	if(points.empty()) {
		return(false);
	}
	// sort the points by their distance to the projected line
	points.sort();
	// find the biggest set of points which differ in their distance to the projected line by no more than max_difference
	size_t current_size = 1;
	size_t biggest_size = 1;
	// biggest set found so far
	forward_list<PointWithDist>::iterator begin_biggest = points.begin();
	// currently considered set
	forward_list<PointWithDist>::iterator begin_current = points.begin();
	// working iterator
	forward_list<PointWithDist>::iterator it = points.begin();
	++it;
	float start_distance = begin_current->dist;
	for(; it != points.end();) {
		float difference = it->dist - start_distance;
		if(difference <= max_difference) {
			++current_size;
			++it;
			if(current_size > biggest_size) {
				begin_biggest = begin_current;
				biggest_size = current_size;
			}
		} else {
			++begin_current;
			start_distance = begin_current->dist;
			if(begin_current == it) {
				++it;
			} else {
				--current_size;
			}
		}
	}

	// fit a line through the biggest set, this is our initial line candidate
	if(biggest_size >= 5) {	// we chose the grid size so we'd get a minimum of 10 measurements, at least half of which we should have here to be sure we have a line
		std::vector<cv::Point2f> points(biggest_size, roi_tl);
		for(size_t i = 0; i < biggest_size; ++i, ++begin_biggest) {
			points[i].x += begin_biggest->x;
			points[i].y += begin_biggest->y;
			/*cv::Scalar color(0,255,0);
			cv::Point center(it->x, it->y);
			circle(image, center, 3, color);*/
		}
		line = fitLine(points);
		return(true);
	} else {
		return(false);
	}
}

void NewRoiMarkerExtractor::getLinePoints(const cv::Mat &image, const cv::Point2f &start, const cv::Point2f &axis, const cv::Point2f &axis_grid, const cv::Vec4f &projected_line, const int window_size, forward_list<PointWithDist> &points) {
	// some settings
	const int threshold = 128;
	const int half_size = window_size >> 1;

	// return if our start_pixel is outside the image
	float start_pixel_x = start.x + 0.5f;
	float start_pixel_y = start.y + 0.5f;
	if(start_pixel_x < 0 || start_pixel_x >= image.cols || start_pixel_y < 0 || start_pixel_y >= image.rows) return;

	cv::Point2f line_start(start);
	bool axis_grid_leave_above = axis_grid.y < 0;
	bool axis_grid_leave_left = axis_grid.x < 0;
	float raxis_grid_x = FABS(1.0f/axis_grid.x);
	float raxis_grid_y = FABS(1.0f/axis_grid.y);

	float dx[2], dy[2];
	dx[0] = image.cols - start.x;	// leave right
	dx[1] = start.x;	// leave left
	dy[0] = image.rows - start.y; // leave below
	dy[1] = start.y;	// leave above
	// if we need a smaller amount of steps to get outside, then we leave in that direction faster
	float grid_ratio_x = dx[axis_grid_leave_left] * raxis_grid_x;
	float grid_ratio_y = dy[axis_grid_leave_above] * raxis_grid_y;
	int num_lines = (int)min(grid_ratio_x, grid_ratio_y);

	/*float raxis_x = FABS(1.0f/axis.x);
	float raxis_y = FABS(1.0f/axis.y);
	// these declare what will eventually happen given our grid heading
	bool leave_above = axis_grid.y < 0;
	bool leave_left = axis_grid.x < 0;
	// now we can find out which happens first
	float dx[2], dy[2];
	dx[0] = image.cols - start.x;	// leave right
	dx[1] = start.x;	// leave left
	dy[0] = image.rows - start.y; // leave below
	dy[1] = start.y;	// leave above
	// if we need a smaller amount of steps to get outside, then we leave in that direction faster
	float ratio_x = FABS(dx[leave_left]*raxis_y);
	float ratio_y = FABS(dy[leave_above]*raxis_x);
	bool sideways_first = ratio_x < ratio_y;

	// either both are negative or both are positive, in which case the product is > 0
	// also we cannot come back into the image if one of the axis vector components is 0 (doesnt matter which one we take here)
	bool comeback_possible = ((sideways_first) ? axis_grid.y*axis.y > 0 : axis_grid.x*axis.x > 0) && axis_grid.x != 0 && axis_grid.y != 0;*/

	// these declare what will eventually happen given our grid heading
	bool axis_leave_above = axis.y < 0;
	bool axis_leave_left = axis.x < 0;
	float raxis_x = FABS(1.0f/axis.x);
	float raxis_y = FABS(1.0f/axis.y);
	for(int i = 0; i < num_lines; ++i) {
		//bool inside_image = line_start.x > 0 && line_start.x < image.cols && line_start.y > 0 && line_start.y < image.rows;
		/*if(unlikely(!inside_image && comeback_possible)) {
			// jump to first point inside image which is possible
			float factor;
			if(sideways_first) {
				factor = raxis_x * (leave_left ? -line_start.x : line_start.x - image.cols);
			} else {
				factor = raxis_y * (leave_above ? -line_start.y : line_start.y - image.rows);
			}
			line_start += (factor+1) * axis;
			// if it doesnt work, line_start is still outside the image => abort
			inside_image = line_start.x >= 0 && line_start.x < image.cols && line_start.y >= 0 && line_start.y < image.rows;
		}*/

		// determine how many measurements we can potentially make
		float dx[2], dy[2];
		dx[0] = image.cols - line_start.x;	// leave right
		dx[1] = line_start.x;	// leave left
		dy[0] = image.rows - line_start.y; // leave below
		dy[1] = line_start.y;	// leave above
		// if we need a smaller amount of steps to get outside, then we leave in that direction faster
		float ratio_x = (dx[axis_leave_left]-0.5f) * raxis_x;
		float ratio_y = (dy[axis_leave_above]-0.5f) * raxis_y;
		int num_meas = (int)min(ratio_x, ratio_y);

		if(num_meas < window_size) break;

		// initialize intensity and gradient buffer
		int intensity[num_meas];
		int gradient[num_meas];
		cv::Point2f next(line_start);
		// determine points
		for(int j = 0; j < num_meas; ++j) {
			// nearest neighbor interpolation, because linear interpolation is too time consuming for this step
			int x = (int)(next.x+0.5f);
			int y = (int)(next.y+0.5f);
			//assert(x >= 0 && y >= 0 && x < image.cols && y < image.rows);
			int value = image.at<uint8_t>(y,x);
			intensity[j] = value;
			next += axis;
		}
		// determine intensity difference
		gradient[0] = intensity[1] - intensity[0];
		int max_index = num_meas - 1;
		for(int k = 1; k < max_index; ++k) {
			gradient[k] = intensity[k+1] - intensity[k-1];
		}
		gradient[max_index] = intensity[max_index] - intensity[max_index-1];
		// determine sum over window (reuse intensity buffer to store the sum)
		int sum = 0;
		// initialize sum
		for(int k = 0; k < window_size; ++k) {
			sum += gradient[k];
		}

		bool gradient_found = false;
		int best_gradient = -1;
		int best_gradient_value = 0;
		// check first point
		if(sum >= threshold) {
			best_gradient = half_size;
			best_gradient_value = sum;
			gradient_found = true;
		}
		max_index = num_meas - window_size;
		for(int k = 0; k < max_index; ++k) {
			sum += gradient[k+window_size] - gradient[k];
			if(sum >= threshold) {
				if(!gradient_found || sum > best_gradient_value) {
					best_gradient = k + half_size + 1;
					best_gradient_value = sum;
					gradient_found = true;
				}
			} else if(gradient_found) {
				int x = (int)(line_start.x + 0.5f + best_gradient*axis.x);
				int y = (int)(line_start.y + 0.5f + best_gradient*axis.y);
				points.push_front(PointWithDist(x,y,signedDistToLine(x,y,projected_line)));
				best_gradient_value = 0;
				gradient_found = false;
			}
		}

		line_start += axis_grid;
	}
}

void NewRoiMarkerExtractor::findCandidate(cv::Mat &image, const MarkerROI &roi, forward_list<vector<cv::Point> > &candidates) {
	// have approx 10 measurements per line (2 might be on the edge or even outside the line, therefore we probe with 12)
	const float grid_size = 1.0f/12.0f * roi.projected_line_length[0];
	// window size should be roughly half 1/7 of the line length (span half of the individual square on each side of the line)
	const int half_size = (int)(1.0f/14.0f * roi.projected_line_length[0] - 0.5f);	// -0.5 because we could have both on the pixel border
	const int window_size = 2 * half_size + 1;
	const float max_difference = 2 + 1 + roi.projected_line_length[0] * TAN(1*g_PI/180.0f);	// measurement error (+-1) + quantisation error (+-0.5) + angle error 1� (doesnt matter which direction)

	//double time_ms_factor = 1000000.0/cv::getTickFrequency();
	//double start_time = (double)cv::getTickCount() * time_ms_factor;
	cv::Point2f roi_tl = roi.roi.tl();
	cv::Point2f vertices[4] = { roi.corners[0]-roi_tl, roi.corners[1]-roi_tl, roi.corners[2]-roi_tl, roi.corners[3]-roi_tl };
	cv::Mat color_image;
	/*cvtColor(image, color_image, CV_GRAY2BGR);
	for(size_t i = 0; i < 4; ++i) {
		circle(color_image, vertices[i], 5, -1);
	}*/
	// get center point for the predicted marker as point of origin for our search
	cv::Point2f cp1 = 0.5f * (vertices[1] + vertices[2]);
	cv::Point2f cp3 = 0.5f * (vertices[3] + vertices[0]);
	cv::Point2f center = 0.5f * (cp1 + cp3);

	// determine axes
	cv::Point2f axis_x(cp1-cp3);	// x-axis |1|
	axis_x *= 1.0f/SQRT(axis_x.x*axis_x.x + axis_x.y*axis_x.y);
	cv::Point2f axis_x_grid(grid_size*axis_x.x, grid_size*axis_x.y);	// x-axis |grid_size|
	cv::Point2f axis_x_neg(-axis_x); // negative x-axis |1|
	cv::Point2f axis_y(axis_x.y, -axis_x.x);	// y-axis |1|
	cv::Point2f axis_y_grid(axis_x_grid.y, -axis_x_grid.x);	// x-axis |grid_size|
	cv::Point2f axis_y_neg(-axis_y);	// negative y-axis |1|

	// undistorted projected points in case we cannot determine a line
	vector<cv::Point2f> corners(roi.corners);
	Undistorter::undistortPoints(corners);

	forward_list<PointWithDist> line_points;
	cv::Vec4f lines[4];
	cv::Point2f v(vertices[2]-vertices[1]);
	cv::Vec4f projected_line(v.x,v.y,vertices[1].x, vertices[1].y);
	// positive x-direction
	cv::Point2f line_start(center-(float)(half_size)*axis_x);
	getLinePoints(image, line_start, axis_x, axis_y_grid, projected_line, window_size, line_points);
	line_start = center-(float)(half_size)*axis_x - axis_y_grid;
	getLinePoints(image, line_start, axis_x, -axis_y_grid, projected_line, window_size, line_points);
	if(!getLine(color_image, line_points, roi_tl, max_difference, lines[0])) {
		v = cv::Point2f(corners[2]-corners[1]);
		lines[0] = cv::Vec4f(v.x, v.y, corners[1].x, corners[1].y);
	}

	// negative x-direction
	line_points.clear();
	v = cv::Point2f(vertices[0]-vertices[3]);
	projected_line = cv::Vec4f(v.x,v.y,vertices[3].x, vertices[3].y);
	line_start = center+(float)(half_size)*axis_x;
	getLinePoints(image, line_start, -axis_x, axis_y_grid, projected_line, window_size, line_points);
	line_start = center+(float)(half_size)*axis_x - axis_y_grid;
	getLinePoints(image, line_start, -axis_x, -axis_y_grid, projected_line, window_size, line_points);
	if(!getLine(color_image, line_points, roi_tl, max_difference, lines[2])) {
		v = cv::Point2f(corners[0]-corners[3]);
		lines[2] = cv::Vec4f(v.x, v.y, corners[3].x, corners[3].y);
	}

	// positive y-direction
	line_points.clear();
	v = cv::Point2f(vertices[1]-vertices[0]);
	projected_line = cv::Vec4f(v.x,v.y,vertices[0].x, vertices[0].y);
	line_start = center-(float)(half_size)*axis_y;
	getLinePoints(image, line_start, axis_y, axis_x_grid, projected_line, window_size, line_points);
	line_start = center-(float)(half_size)*axis_y - axis_x_grid;
	getLinePoints(image, line_start, axis_y, -axis_x_grid, projected_line, window_size, line_points);
	if(!getLine(color_image, line_points, roi_tl, max_difference, lines[1])) {
		v = cv::Point2f(corners[1]-corners[0]);
		lines[1] = cv::Vec4f(v.x, v.y, corners[0].x, corners[0].y);
	}

	// negative y-direction
	line_points.clear();
	v = cv::Point2f(vertices[3]-vertices[2]);
	projected_line = cv::Vec4f(v.x,v.y,vertices[2].x, vertices[2].y);
	line_start = center+(float)(half_size)*axis_y;
	getLinePoints(image, line_start, -axis_y, axis_x_grid, projected_line, window_size, line_points);
	line_start = center+(float)(half_size)*axis_y - axis_x_grid;
	getLinePoints(image, line_start, -axis_y, -axis_x_grid, projected_line, window_size, line_points);
	if(!getLine(color_image, line_points, roi_tl, max_difference, lines[3])) {
		v = cv::Point2f(corners[3]-corners[2]);
		lines[3] = cv::Vec4f(v.x, v.y, corners[2].x, corners[2].y);
	}

	// determine the vertices as crossing of the corresponding lines (reuse the corners vector)
	corners[0] = getCrossing(lines[1], lines[2]);
	corners[1] = getCrossing(lines[0], lines[1]);
	corners[2] = getCrossing(lines[0], lines[3]);
	corners[3] = getCrossing(lines[2], lines[3]);
	// undistort and subtract roi_tl to move the corners to the roi
	Undistorter::distortPoints(corners);
	// this is our new candidate
	vector<cv::Point> quatrilateral(4);
	for(size_t i = 0; i < 4; ++i) {
		quatrilateral[i].x = (int)(corners[i].x - roi_tl.x + 0.5f);
		quatrilateral[i].y = (int)(corners[i].y - roi_tl.y + 0.5f);
	}
	candidates.push_front(quatrilateral);

	//double end_time = (double)cv::getTickCount() * time_ms_factor;
	//cout << end_time-start_time << endl;

	// show the result: draw line between two subsequent points
	/*cv::Scalar line_color(0,0,255);
	cv::Scalar axis_color(255,0,0);
	cv::Scalar center_color(0,255,0);
	circle(color_image, center, 5, line_color);
	line(color_image, center, center+5*axis_x_grid, axis_color);
	line(color_image, center, center+5*axis_y_grid, axis_color);
	for(size_t i = 0; i < 4; ++i) {
		int next = MOD4((i+1));
		line(color_image, quatrilateral[i], quatrilateral[next], line_color);
	}
	imshow("stage2_new", color_image);
	cv::waitKey();*/
}

MarkerInfo NewRoiMarkerExtractor::findBestCandidate(const cv::Mat &image, const MarkerROI &roi, list<vector<cv::Point> > &candidates) const {
	// for every remaining candidate this will contain the sum of squares of the projected line length between detected square and target marker
	for(list<vector<cv::Point> >::iterator candidates_it = candidates.begin(); candidates_it != candidates.end();) {
		// check if we have a possible candidate
		bool viable_candidate = true;
		float line_lengths[4];
		for(size_t j = 0; j < 4; ++j) {
			int next = MOD4((j+1));
			int dx = candidates_it->at(j).x - candidates_it->at(next).x;
			int dy = candidates_it->at(j).y - candidates_it->at(next).y;
			line_lengths[j] = SQRT(dx*dx + dy*dy);
		}
		int min_index1 = 0, min_index2 = 1;
		float min_diff = FLT_MAX;
		for(int j = 0; j < 3; ++j) {
			for(int k = j+1; k < 4; ++k) {
				float diff = FABS(line_lengths[j]-line_lengths[k]);
				if(diff < min_diff) {
					min_diff = diff;
					min_index1 = j;
					min_index2 = k;
				}
			}
		}

		// only allow quatrilaterals which do not deviate too much
		float length_reference = 0.5f * (line_lengths[min_index1] + line_lengths[min_index2]);
		float length_upper_border = ub_factor_ * length_reference;
		length_upper_border = min(length_upper_border, roi.projected_line_length[2]);
		float length_lower_border = lb_factor_ * length_reference;
		length_lower_border = max(length_lower_border, roi.projected_line_length[1]);
		for(size_t j = 0; j < 4; ++j) {
			// check line length constraint
			if(line_lengths[j] > length_upper_border || line_lengths[j] < length_lower_border) {
				viable_candidate = false;
				// remove upon failure
				candidates_it = candidates.erase(candidates_it);
				break;
			}
		}
		if(viable_candidate) {
			++candidates_it;
		}
	}

	// determine best suitable of all remaining candidates
	if(!candidates.empty()) {
		// find square with minimal target difference
		list<vector<cv::Point> >::const_iterator smallest_difference = candidates.begin();
		cv::Point2f roi_tl = roi.roi.tl();
		cv::Point2f projected_points[4] = { roi.corners[0]-roi_tl, roi.corners[1]-roi_tl, roi.corners[2]-roi_tl, roi.corners[3]-roi_tl };
		if(candidates.size() > 1) {
			float area_target = computeQuadArea(projected_points);
			//float area_target = roi.projected_line_length[0] * roi.projected_line_length[0];
			float min_error = numeric_limits<float>::max();
			for(list<vector<cv::Point> >::const_iterator candidates_it = candidates.begin(); candidates_it != candidates.end(); ++candidates_it) {
				// compute error value
				cv::Point2f vertices[4] = { candidates_it->at(0), candidates_it->at(1), candidates_it->at(2), candidates_it->at(3) };
				array<float, 4> angles = computeAngles(vertices);
				float area_status = computeQuadArea(vertices);

				float area_error = (area_status > area_target) ? area_target/area_status : area_status/area_target;
				float angle_error = 1;
				for(size_t i = 0; i < 4; ++i) {
					float error = FABS(angles[i]);
					error = error > g_PI_2 ? g_PI/(2*error) : (2.0f/g_PI)*error;
					angle_error *= error;
				}
				float error = 1 - angle_error * area_error;

				if(error < min_error) {
					min_error = error;
					smallest_difference = candidates_it;
				}
			}
		}

		// now we have our most probable marker location
		// now determine the least_min_square point mapping (because its only a 4x4 matrix and hence fast)
		cv::Mat map_cost(4, 4, MAT_TYPE); // matrix is symmetric
		// squared distance between candidate[j] and predicted point[k]
		for(int j = 0; j < 4; ++j) {
			float* row = map_cost.ptr<float>(j);
			for(int k = 0; k < 4; ++k) {
				float dx = smallest_difference->at(j).x - projected_points[k].x;
				float dy = smallest_difference->at(j).y - projected_points[k].y;
				row[k] = dx*dx + dy*dy;
			}
		}
		// brute force: find minimal mapping of all four points (we only have 24 so its fast)
		float cost[24];
		float* row = (float*)map_cost.data;
		cost[0] = row[0] + row[4+1] + row[8+2] + row[12+3];
		cost[1] = row[0] + row[4+1] + row[8+3] + row[12+2];
		cost[2] = row[0] + row[4+2] + row[8+1] + row[12+3];
		cost[3] = row[0] + row[4+2] + row[8+3] + row[12+1];
		cost[4] = row[0] + row[4+3] + row[8+1] + row[12+2];
		cost[5] = row[0] + row[4+3] + row[8+2] + row[12+1];

		cost[6] = row[1] + row[4+0] + row[8+2] + row[12+3];
		cost[7] = row[1] + row[4+0] + row[8+3] + row[12+2];
		cost[8] = row[1] + row[4+2] + row[8+0] + row[12+3];
		cost[9] = row[1] + row[4+2] + row[8+3] + row[12+0];
		cost[10] = row[1] + row[4+3] + row[8+0] + row[12+2];
		cost[11] = row[1] + row[4+3] + row[8+2] + row[12+0];

		cost[12] = row[2] + row[4+0] + row[8+1] + row[12+3];
		cost[13] = row[2] + row[4+0] + row[8+3] + row[12+1];
		cost[14] = row[2] + row[4+1] + row[8+0] + row[12+3];
		cost[15] = row[2] + row[4+1] + row[8+3] + row[12+0];
		cost[16] = row[2] + row[4+3] + row[8+0] + row[12+1];
		cost[17] = row[2] + row[4+3] + row[8+1] + row[12+0];

		cost[18] = row[3] + row[4+0] + row[8+1] + row[12+2];
		cost[19] = row[3] + row[4+0] + row[8+2] + row[12+1];
		cost[20] = row[3] + row[4+1] + row[8+0] + row[12+2];
		cost[21] = row[3] + row[4+1] + row[8+2] + row[12+0];
		cost[22] = row[3] + row[4+2] + row[8+0] + row[12+1];
		cost[23] = row[3] + row[4+2] + row[8+1] + row[12+0];

		// find minimal index
		int min_index = 0;
		float min_cost = cost[0];
		for(int j = 1; j < 24; ++j) {
			if(cost[j] < min_cost) {
				min_cost = cost[j];
				min_index = j;
			}
		}

		// apply mapping and create marker object
		Pose2d p(roi.id, 0, 0, 0, 0, 0, 0, 0);
		float inner_marker_length = Config::tracker_parameters.special_marker_sizes_mm.count(roi.id) ? Config::tracker_parameters.special_marker_sizes_mm.at(roi.id) : Config::tracker_parameters.marker_size_mm;
		MarkerInfo marker(inner_marker_length, p);
		marker.vertices[map_corner_buffer[min_index][0]] = smallest_difference->at(0);
		marker.vertices[map_corner_buffer[min_index][1]] = smallest_difference->at(1);
		marker.vertices[map_corner_buffer[min_index][2]] = smallest_difference->at(2);
		marker.vertices[map_corner_buffer[min_index][3]] = smallest_difference->at(3);

		// erase the result if the angle towards the projection is too high (> 45� or g_PI/4)
		float angle = FLT_MAX;
		for(size_t i = 0; i < 4; ++i) {
			int next = MOD4((i+1));
			cv::Point2f vp(projected_points[next] - projected_points[i]);
			cv::Point2f vm(marker.vertices[next] - marker.vertices[i]);
			angle = min(angle, ACOS(min((vp.x*vm.x + vp.y*vm.y)/(SQRT(vp.x*vp.x + vp.y*vp.y) * SQRT(vm.x*vm.x + vm.y*vm.y)), 1.0f)));
		}
		if(angle > (5.0f*g_PI)/180.0f) {	// > 5� => not feasible that we can come to a good measurement here
			/*cout << angle*g_180_PI << endl;
			cv::Mat color_image;
			cv::cvtColor(image, color_image, CV_GRAY2BGR);
			for(size_t i = 0; i < 4; ++i) {
				int next = MOD4(i+1);
				line(color_image, marker.vertices[i], marker.vertices[next], cv::Scalar(0,255,0), 1);
				line(color_image, projected_points[i], projected_points[next], cv::Scalar(0,0,255), 1);
				cout << marker.vertices[i] << projected_points[i] << endl;
			}
			imshow(intToString(ROUND(angle)), color_image);
			cv::waitKey();*/
			return(MarkerInfo());
		}

		// denoise does not work here (too slow and poor results)!

		// refine the corners in the enhanced image
		//refineCorners_harris(image, marker);
		//refineCorners(image, marker);
		refineMarkerLines(image, roi_tl, marker.vertices, roi.projected_line_length[0]);

		if(roi.projected_line_length[0] > 0) {
			// reconstruct shape
			reconstructShape(marker.vertices, roi.corner_inside_image, roi.projected_line_length[0]);

			// distort points so we can work with them again
			vector<cv::Point2f> distorted_points(marker.vertices, marker.vertices+4);
			Undistorter::distortPoints(distorted_points);
			marker.vertices[0] = distorted_points[0] - roi_tl;
			marker.vertices[1] = distorted_points[1] - roi_tl;
			marker.vertices[2] = distorted_points[2] - roi_tl;
			marker.vertices[3] = distorted_points[3] - roi_tl;

			// refine the lines of the reconstructed shape
			refineMarkerLines(image, roi_tl, marker.vertices, roi.projected_line_length[0]);
		}

		// filter the ones that are not accurate enough
		return(marker);
		// check if we got a marker with a close-enough id
		/*vector<cv::Point2f> distorted_points(marker.vertices, marker.vertices+4);
		Undistorter::distortPoints(distorted_points);
		for(size_t j = 0; j < 4; ++j) {
			distorted_points[j] -= roi_tl;
		}

		MarkerInfo marker2;
		marker2.id = roi.id;
		identifier_.identify(image, distorted_points, marker2);
		marker.detection_error = marker2.detection_error;
		return((marker2.detection_error <= 2) ? marker : MarkerInfo());*/
	} else {
		return(MarkerInfo());
	}
}

cv::Vec4f NewRoiMarkerExtractor::refineEdge(cv::Mat &image, cv::Point2f &start, cv::Point2f &end) {
	const int search_length = 3;
	const int num_points = 3;

	int max_row = image.rows - 1;
	int max_col = image.cols - 1;

	const int max_index = 2 * search_length;
	const int min_req_index = max_index - 1;

	vector<cv::Point2f> points;
	points.reserve(num_points);

	cv::Point2f vector(end - start); // direction of the line
	cv::Point2f offset((float)(0.8f/(num_points-1)) * vector);
	cv::Point2f sample_float(start + 0.1f * vector);
	cv::Point2f end_point(start + 0.9f * vector);

	// handle case where start or end point is outside image (to get enough measurement points where possible)
	bool start_inside_image = sample_float.x >= 0 && sample_float.x < max_col && sample_float.y >= 0 && sample_float.y < max_row;
	bool end_inside_image = end_point.x >= 0 && end_point.x < max_col && end_point.y >= 0 && end_point.y < max_row;
	bool failed = false;
	//bool adjustment = false;
	if(start_inside_image && end_inside_image) {
		// this is fine => continue below
	} else if(start_inside_image) {
		// check if we could get n = num_points/3+1 points (start + n*offset) in image?
		cv::Point2f point_test(sample_float + (num_points/4+1) * offset);
		if(point_test.x >= 0 && point_test.x < max_col && point_test.y >= 0 && point_test.y < max_row) {
			// start inside image => determine new end and offset
			bool vector_leave_above = vector.y < 0;
			bool vector_leave_left = vector.x < 0;
			float rvector_x = FABS(1.0f/vector.x);
			float rvector_y = FABS(1.0f/vector.y);

			float dx[2], dy[2];
			dx[0] = image.cols - sample_float.x;	// leave right
			dx[1] = sample_float.x;	// leave left
			dy[0] = image.rows - sample_float.y; // leave below
			dy[1] = sample_float.y;	// leave above
			// if we need a smaller amount of steps to get outside, then we leave in that direction faster
			float ratio_x = dx[vector_leave_left] * rvector_x;
			float ratio_y = dy[vector_leave_above] * rvector_y;
			float max_steps = min(ratio_x, ratio_y);
			end_point = sample_float + max_steps * vector;
			offset = 0.9f/(num_points-1) * (end_point - sample_float);
		} else {
			failed = true;
		}
	} else if(end_inside_image) {
		// check if we could get n = num_points/3+1 points (end - n*offset) in image?
		cv::Point2f point_test(end_point - (num_points/4+1) * offset);
		if(point_test.x >= 0 && point_test.x < max_col && point_test.y >= 0 && point_test.y < max_row) {
			//adjustment = true;
			// end inside image => determine new start and offset
			// to keep it simple we reverse the situation, do exactly the same as above and reverse the result
			// reverse vector
			cv::Point2f vector_copy(-vector);
			// make end_point start, this is now inside the image
			cv::Point2f sample_float_copy(end_point);
			// now our situation is the same as above, so we can apply the same method
			bool vector_leave_above = vector_copy.y < 0;
			bool vector_leave_left = vector_copy.x < 0;
			float rvector_x = FABS(1.0f/vector_copy.x);
			float rvector_y = FABS(1.0f/vector_copy.y);

			float dx[2], dy[2];
			dx[0] = image.cols - sample_float_copy.x;	// leave right
			dx[1] = sample_float_copy.x;	// leave left
			dy[0] = image.rows - sample_float_copy.y; // leave below
			dy[1] = sample_float_copy.y;	// leave above
			// if we need a smaller amount of steps to get outside, then we leave in that direction faster
			float ratio_x = dx[vector_leave_left] * rvector_x;
			float ratio_y = dy[vector_leave_above] * rvector_y;
			float max_steps = min(ratio_x, ratio_y);
			end_point = sample_float_copy + max_steps * vector_copy;
			cv::Point2f new_vector(end_point - sample_float_copy);
			offset = -0.9f/(num_points-1) * new_vector;
			// now transform back
			//offset *= -1; // already integrated above
			sample_float = end_point - 0.1f * new_vector;	// our transformed "end_point" is the actual start_point
		} else {
			failed = true;
		}
	} else {
		// start and end not inside image => do nothing
		failed = true;
	}
	// in case no measurement is feasible, we dont try it
	if(failed) {
		// we have no chance of improving the line
		return(cv::Vec4f(end.x-start.x, end.y-start.y, start.x, start.y));
	}

	vector *= 1.0f/SQRT(vector.x*vector.x + vector.y*vector.y); // normalized to |v|=1
	cv::Point2f normal(vector.y, -vector.x); // normal to vector with |v|=1 pointing towards the white border
	sample_float -= search_length * normal; // start point

	for(size_t i = 0; i < num_points; ++i) {
		// determine pixels along the normal
		float x = sample_float.x;
		float y = sample_float.y;
		// check first, last-1 and last
		// first
		int this_max_index = (x >= 0 && x < max_col && y >= 0 && y < max_row); // max index for this point
		// last-1 (the minimum required one)
		x += min_req_index * normal.x;
		y += min_req_index * normal.y;
		this_max_index += (x >= 0 && x < max_col && y >= 0 && y < max_row);
		bool go_on = this_max_index == 2;	// we have a pixel at first and last-1 => we can continue
		// also include last?
		x += normal.x;
		y += normal.y;
		// also add the ones in between implicitly (we tested 3 already)
		this_max_index += (x >= 0 && x < max_col && y >= 0 && y < max_row) + max_index - 3;

		if(likely(go_on)) {
			float intensity[2*search_length+1];
			float x = sample_float.x;
			float y = sample_float.y;
			for(int j = 0; j <= this_max_index; ++j) {
				// do bilinear interpolation
				// speedup: fixed-point arithmetic (accuracy up to 1/1000 pixel)
				/*int x1 = x;
				int y1 = y;
				int x_f = 1024.0f * x;
				int y_f = 1024.0f * y;
				int delta_x1 = x_f - (x1 << 10);
				int delta_x2 = 1024 - delta_x1;
				int delta_y1 = y_f - (y1 << 10);

				uint8_t* row = image.data + y1 * image.step + x1;
				int v0 = *row, v1 = *(row+1);
				int interp_intensity = (1024 - delta_y1) * (delta_x2 * v0 + delta_x1 * v1);

				row += image.step;
				v0 = *row, v1 = *(row+1);
				interp_intensity += delta_y1 * (delta_x2 * v0 + delta_x1 * v1);
				intensity[j] = interp_intensity;*/

				// do nearest neighbor interpolation
				// speedy version of ROUND(for positive values);
				int x1 = (int)(x + 0.5f);
				int y1 = (int)(y + 0.5f);
				intensity[j] = image.at<uint8_t>(y1, x1);

				x += normal.x;
				y += normal.y;
			}

			// compare their intensity values (in both directions)
			int diff[2*search_length+1];

			// try conventional approach: sobel, laplacian
			// sobel: const int kernel[3] = { -1, 0, 1 };
			diff[0] = intensity[1] - intensity[0];
			for(int j = 1; j < this_max_index; ++j) {
				diff[j] = intensity[j+1] - intensity[j-1];
			}
			diff[this_max_index] = intensity[this_max_index] - intensity[this_max_index-1];

			// sobel version with maxsum
			int A = diff[0]; // contains the max sum
			int B = diff[0]; // temporary maxsum candidate
			int lba = 0, uba = 0, lbb = 0;
			for(int j = 1; j <= this_max_index; ++j) {
				int current = diff[j];
				B += current;
				if(current > B) {
					B = current;
					lbb = j;
				}
				if(B > A) {
					A = B;
					lba = lbb;
					uba = j;
				}
			}

			if(likely(A > 0)) {
				// subpixel accuracy: weighted mean over all the pixels from [lba;uba+1]
				// we do not need to convert back to float in the end because that factor is included in A
				float factor = 1.0f/A;
				int center = lba * diff[lba];
				for(int j = lba+1; j <= uba; ++j) {
					int gz = diff[j] > 0;
					center += (gz * j) * diff[j];
				}
				float position = center * factor;
				// finally add the determined point
				points.push_back(sample_float + position*normal);
			}
		}
		sample_float += offset;
	}

	/*set<int> analyze_frames;
	analyze_frames.insert(59902603);
	analyze_frames.insert(59463090);
	analyze_frames.insert(59754970);
	analyze_frames.insert(59598271);

	if(unlikely(analyze_frames.count(num_frames) > 0)) {
		cv::Mat color_image;
		cvtColor(image, color_image, CV_GRAY2BGR);
		//cerr << "start" << endl;
		//cerr << num_frames << endl;
		//cerr << a << b << endl;
		line(color_image, start, end, cv::Scalar(255,0,0));
		for(size_t i = 0; i < points.size(); ++i) {
			cv::Point p(ROUND(points[i].x-roi_tl.x), ROUND(points[i].y-roi_tl.y));
			//cerr << i << p << *it << endl;
			if(p.x >= 0 && p.x < image.cols && p.y >= 0 && p.y < image.rows) {
				color_image.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(0,0,255);
			}
		}
		imshow("", color_image);
		cv::waitKey();
		//cerr << "end" << endl;
	}*/
	/*if(unlikely(adjustment)) {
		cout << points.size() << endl;
		cv::Mat color_image;
		cvtColor(image, color_image, CV_GRAY2BGR);
		//cerr << "start" << endl;
		//cerr << num_frames << endl;
		//cerr << a << b << endl;
		line(color_image, start, end, cv::Scalar(255,0,0));
		for(size_t i = 0; i < points.size(); ++i) {
			cv::Point p(points[i].x-roi_tl.x+0.5f, points[i].y-roi_tl.y+0.5f);
			//cerr << i << p << *it << endl;
			if(p.x >= 0 && p.x < image.cols && p.y >= 0 && p.y < image.rows) {
				color_image.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(0,0,255);
			}
		}
		imshow("", color_image);
		cv::waitKey();
		//cerr << "end" << endl;
	}*/

	int num_points_determined = points.size();
	if(likely(num_points_determined > 1)) {
		// fit a line through these points
		float rnum = 1.0f/num_points_determined;
		cv::Point2f mean_point(points[0]);
		for(int i = 1; i < num_points_determined; ++i) {
			mean_point += points[i];
		}
		mean_point *= rnum;

		// other kind of line fit
		// if the actual slope is close to infinity,
		// the fit will be very bad, hence we simply fit to y instead of x in this case and swap x,y later to get the correct slope
		cv::Vec4f out_line(0, 0, mean_point.x, mean_point.y);
		float dx = FABS(points[0].x - points[num_points_determined-1].x);
		float dy = FABS(points[0].y - points[num_points_determined-1].y);
		if(dx > dy) {
			float x_y_dev = 0, x_dev_p2 = 0;
			for(int i = 0; i < num_points_determined; ++i) {
				float x_dev = points[i].x - mean_point.x;
				float y_dev = points[i].y - mean_point.y;
				x_y_dev += x_dev * y_dev;
				x_dev_p2 += x_dev * x_dev;
			}

			out_line[0] = x_dev_p2;
			out_line[1] = x_y_dev;
		} else {
			float x_y_dev = 0, y_dev_p2 = 0;
			for(int i = 0; i < num_points_determined; ++i) {
				float x_dev = points[i].x - mean_point.x;
				float y_dev = points[i].y - mean_point.y;
				x_y_dev += x_dev * y_dev;
				y_dev_p2 += y_dev * y_dev;
			}

			out_line[0] = x_y_dev;
			out_line[1] = y_dev_p2;
		}

		return(out_line);
	} else {
		// we have no chance of improving the line
		return(cv::Vec4f(end.x-start.x, end.y-start.y, start.x, start.y));
	}
}

void NewRoiMarkerExtractor::refineEdges(cv::Mat &image, vector<cv::Point2f> &candidate, const int iterations) {
	for(int i = 0; i < iterations; ++i) {
		cv::Vec4f lines[4];
		// first refine the lines
		lines[0] = refineEdge(image, candidate[0], candidate[1]);
		lines[1] = refineEdge(image, candidate[1], candidate[2]);
		lines[2] = refineEdge(image, candidate[2], candidate[3]);
		lines[3] = refineEdge(image, candidate[3], candidate[0]);

		// marker corners are then refined to the crossing of two subsequent lines (very accurately)
		candidate[0] = getCrossing(lines[3], lines[0]);
		candidate[1] = getCrossing(lines[0], lines[1]);
		candidate[2] = getCrossing(lines[1], lines[2]);
		candidate[3] = getCrossing(lines[2], lines[3]);
	}
}

list<MarkerInfo> NewRoiMarkerExtractor::getMarkers(const cv::Mat &image, const vector<MarkerROI> &rois, double &after_preprocessing, double &after_detect, double &after_identify) {
	const int max_id_deviation = 5;
	list<MarkerInfo> markers;

	for(size_t i = 0; i < rois.size(); ++i) {
		cv::Mat raw_roi(image, rois[i].roi);
		cv::Mat predicted_area = raw_roi.clone();

		// stage 1 + 2 + 3: conventional approach
		bool stage1_failed = true, stage2_failed = true, stage3_failed = true;
		// run stage1, try to refine predicted marker corner points and see if we can decode the id
		int num_corners_in_image = rois[i].corner_inside_image[0] + rois[i].corner_inside_image[1] + rois[i].corner_inside_image[2] + rois[i].corner_inside_image[3];
		cv::Point2f roi_tl = rois[i].roi.tl();

		cv::Mat equalized;
		// stage1
		Pose2d p(rois[i].id, 0, 0, 0, 0, 0, 0, 0);
		float inner_marker_length = Config::tracker_parameters.special_marker_sizes_mm.count(rois[i].id) ? Config::tracker_parameters.special_marker_sizes_mm.at(rois[i].id) : Config::tracker_parameters.marker_size_mm;
		MarkerInfo marker(inner_marker_length, p);
		// copy predicted corners to vertices
		marker.vertices[0] = rois[i].corners[0] - roi_tl;
		marker.vertices[1] = rois[i].corners[1] - roi_tl;
		marker.vertices[2] = rois[i].corners[2] - roi_tl;
		marker.vertices[3] = rois[i].corners[3] - roi_tl;

		// do some local image enhancements that help segmentation
		equalizeHist(predicted_area, equalized);	// 20�s (no improvement feasible)

		/*cv::Mat color_image;
		cvtColor(equalized, color_image, CV_GRAY2BGR);
		cv::Scalar proj_color(0,0,255);
		cv::Scalar refine_color(0,255,0);
		for(size_t j = 0; j < 4; ++j) {
			int next = MOD4((j+1));
			line(color_image, marker.vertices[j], marker.vertices[next], proj_color);
		}*/

		// systematic stage 1
		MarkerInfo searchMarker(marker), bestMarker(marker);
		vector<cv::Point2f> candidate(marker.vertices, marker.vertices+4);
		// refine best and check if it could be our candidate
		refineEdges(equalized, candidate);
		// check if we found the expected id (or something close)
		identifier_.identify(equalized, candidate, bestMarker);
		//int best_marker_region = -1;
		if(bestMarker.detection_error == 0) {
//			for(size_t j = 0; j < 4; ++j) {
//				int next = MOD4((j+1));
//				line(color_image, candidate[j], candidate[next], refine_color);
//			}
			copy(candidate.begin(), candidate.end(), bestMarker.vertices);
		} else {
			// search immediate surroundings
			bool found = false;
			for(size_t k = 0; k < 8; ++k) {
				// prepare marker candidate
				float offset_x = search_pattern_offsets[k][0];
				float offset_y = search_pattern_offsets[k][1];
				for(int l = 0; l < 4; ++l) {
					candidate[l].x = marker.vertices[l].x + offset_x;
					candidate[l].y = marker.vertices[l].y + offset_y;
				}

				// find corners through refinement
				refineEdges(equalized, candidate);
//					for(size_t j = 0; j < 4; ++j) {
//						int next = MOD4((j+1));
//						line(color_image, candidate[j], candidate[next], refine_color);
//					}
//					imshow("validation", color_image);
//					cv::waitKey();

				// check if we found the expected id (or something close)
				identifier_.identify(equalized, candidate, searchMarker);
				if(searchMarker.detection_error == 0) {
					copy(candidate.begin(), candidate.end(), bestMarker.vertices);
					bestMarker.detection_error = 0;
					found = true;
					//best_marker_region = k;
					break;
				} else if(searchMarker.detection_error <= max_id_deviation+1 && searchMarker.detection_error < bestMarker.detection_error) {
					// not perfect but still acceptable (though we will try to find the best)
					copy(candidate.begin(), candidate.end(), bestMarker.vertices);
					bestMarker.detection_error = searchMarker.detection_error;
					//best_marker_region = k;
				}

				// if no need to continue if marker is already found
				if(found) break;
			}
		}

		if(bestMarker.detection_error <= max_id_deviation+1) {
			//cout << best_marker_region << "," << bestMarker.detection_error << endl;
			copy(bestMarker.vertices, bestMarker.vertices+4, marker.vertices);
			marker.detection_error = bestMarker.detection_error;
			// refine corners
			refineMarkerLines(equalized, roi_tl, marker.vertices, rois[i].projected_line_length[0]);

			if(rois[i].projected_line_length[0] > 0) {
				// reconstruct shape
				reconstructShape(marker.vertices, rois[i].corner_inside_image, rois[i].projected_line_length[0]);

				// distort points so we can work with them again
				vector<cv::Point2f> distorted_points(marker.vertices, marker.vertices+4);
				Undistorter::distortPoints(distorted_points);
				marker.vertices[0] = distorted_points[0] - roi_tl;
				marker.vertices[1] = distorted_points[1] - roi_tl;
				marker.vertices[2] = distorted_points[2] - roi_tl;
				marker.vertices[3] = distorted_points[3] - roi_tl;

				// refine the lines of the reconstructed shape
				refineMarkerLines(equalized, roi_tl, marker.vertices, rois[i].projected_line_length[0]);
			}

			if(keepMarker(marker)) {
				vector<cv::Point2f> distorted_points(marker.vertices, marker.vertices+4);
				Undistorter::distortPoints(distorted_points);
				marker.vertices[0] = distorted_points[0] - roi_tl;
				marker.vertices[1] = distorted_points[1] - roi_tl;
				marker.vertices[2] = distorted_points[2] - roi_tl;
				marker.vertices[3] = distorted_points[3] - roi_tl;
				refineMarkerLinesUltra(predicted_area, roi_tl, marker.vertices, marker.line_accuracy);
				if(isAccurateEnough(marker.line_accuracy)) {
					// accept marker if id was correctly decoded
					stage1_failed = false;
					markers.push_back(marker);
					++stage1_;
				}
			}
		}

		/*
		// now refine them
		refineMarkerLines(equalized, roi_tl, marker.vertices, rois[i].projected_line_length[0]);

		if(rois[i].projected_line_length[0] > 0) {
			const int num_reconstructions = 3;
			for(int j = 0; j < num_reconstructions; ++j) {
				// reconstruct shape
				reconstructShape(marker.vertices, rois[i].corner_inside_image, rois[i].projected_line_length[0]);

				// distort points so we can work with them again
				vector<cv::Point2f> distorted_points(marker.vertices, marker.vertices+4);
				Undistorter::distortPoints(distorted_points);
				marker.vertices[0] = distorted_points[0] - roi_tl;
				marker.vertices[1] = distorted_points[1] - roi_tl;
				marker.vertices[2] = distorted_points[2] - roi_tl;
				marker.vertices[3] = distorted_points[3] - roi_tl;

				// refine the lines of the reconstructed shape
				refineMarkerLines(equalized, roi_tl, marker.vertices, rois[i].projected_line_length[0]);

				if(keepMarker(marker)) {
//					vector<cv::Point2f> distorted_points(4);
//					copy(marker.vertices, marker.vertices+4, distorted_points.begin());
//					Undistorter::distortPoints(distorted_points);
//					distorted_points[0] -= roi_tl;
//					distorted_points[1] -= roi_tl;
//					distorted_points[2] -= roi_tl;
//					distorted_points[3] -= roi_tl;
//					cv::Point2f marker_test[4];
//					copy(distorted_points.begin(), distorted_points.end(), marker_test);
//					refineMarkerFromInside(tmp, roi_tl, marker_test, marker.id);
					vector<cv::Point2f> distorted(marker.vertices, marker.vertices+4);
					Undistorter::distortPoints(distorted);

					distorted[0] -= roi_tl;
					distorted[1] -= roi_tl;
					distorted[2] -= roi_tl;
					distorted[3] -= roi_tl;

					MarkerInfo marker2;
					marker2.id = rois[i].id;
					identifier_.identify(equalized, distorted, marker2);
					marker.detection_error = marker2.detection_error;
					if(marker2.detection_error <= 2*(j+1)) {
						copy(distorted.begin(), distorted.end(), marker.vertices);
						refineMarkerLinesUltra(predicted_area, roi_tl, marker.vertices, marker.line_accuracy);
						// check accuracy
						//if(isAccurateEnough(marker.line_accuracy)) {
							// accept marker if id was correctly decoded
							stage1_failed = false;
							markers.push_back(marker);
							++stage1_;
							break;
						//}
					}
				}
			}
		}*/

		if(stage1_failed) {
			// stage2
			// determine candidate
			forward_list<vector<cv::Point> > candidate;
			findCandidate(equalized, rois[i], candidate);
			forward_list<vector<cv::Point> >::iterator it = candidate.begin();
			// copy predicted corners to vertices
			for(size_t j = 0; j < 4; ++j) {
				marker.vertices[j] = (*it)[j];
			}

			// refine corners
			refineMarkerLines(equalized, roi_tl, marker.vertices, rois[i].projected_line_length[0]);

			if(rois[i].projected_line_length[0] > 0) {
				// reconstruct shape
				reconstructShape(marker.vertices, rois[i].corner_inside_image, rois[i].projected_line_length[0]);

				// distort points so we can work with them again
				vector<cv::Point2f> distorted_points(marker.vertices, marker.vertices+4);
				Undistorter::distortPoints(distorted_points);
				marker.vertices[0] = distorted_points[0] - roi_tl;
				marker.vertices[1] = distorted_points[1] - roi_tl;
				marker.vertices[2] = distorted_points[2] - roi_tl;
				marker.vertices[3] = distorted_points[3] - roi_tl;

				// refine the lines of the reconstructed shape
				refineMarkerLines(equalized, roi_tl, marker.vertices, rois[i].projected_line_length[0]);
			}

			if(keepMarker(marker)) {
				// did we find a marker?
				vector<cv::Point2f> distorted(marker.vertices, marker.vertices+4);
				Undistorter::distortPoints(distorted);

				// identify
				distorted[0] -= roi_tl;
				distorted[1] -= roi_tl;
				distorted[2] -= roi_tl;
				distorted[3] -= roi_tl;

				MarkerInfo marker2;
				marker2.id = rois[i].id;
				identifier_.identify(equalized, distorted, marker2);
				marker.id = marker2.id;
				marker.detection_error = marker2.detection_error;

				if(marker.detection_error <= max_id_deviation) {
					copy(distorted.begin(), distorted.end(), marker.vertices);
					refineMarkerLinesUltra(predicted_area, roi_tl, marker.vertices, marker.line_accuracy);
					// check accuracy
					if(isAccurateEnough(marker.line_accuracy)) {
						// accept marker if id was correctly decoded
						stage2_failed = false;
						markers.push_back(marker);
						++stage2_;
					}
				}
			}
		}

		if(stage1_failed && stage2_failed) {
			// stage3
			// enhance contrast (sharpen + illumination distribution)
			fastUnsharpMask(predicted_area); // 30�s (no improvement feasible, gaussianBlur is this "slow")
			cv::Mat tmp1;//, tmp2;
			list<vector<cv::Point> > candidates1, candidates2;

			// now we follow two paths: 1) histogram equalization and 2) auto-white-balance if necessary
			// detect candidates in both images
			equalizeHist(predicted_area, tmp1); // 20�s (no improvement feasible)
			detector_.detect(tmp1, candidates1);	// 70�s
			//fastNormalizedAwb(predicted_area, tmp2); // 20�s (no improvement feasible)
			//detector_.detect(tmp2, candidates2); // 50�s

			// merge lists
			//candidates1.splice(candidates1.end(), candidates2);

			// find best candidate
			MarkerInfo marker(findBestCandidate(tmp1, rois[i], candidates1));	// 15�s (refinement is the main component)
			if(marker.id >= 0 && keepMarker(marker)) {
				// did we find a marker?
				vector<cv::Point2f> distorted(marker.vertices, marker.vertices+4);
				Undistorter::distortPoints(distorted);

				// identify
				distorted[0] -= roi_tl;
				distorted[1] -= roi_tl;
				distorted[2] -= roi_tl;
				distorted[3] -= roi_tl;

				MarkerInfo marker2;
				marker2.id = rois[i].id;
				identifier_.identify(tmp1, distorted, marker2);
				marker.id = marker2.id;
				marker.detection_error = marker2.detection_error;

				if(marker.detection_error <= max_id_deviation) {
					copy(distorted.begin(), distorted.end(), marker.vertices);
					refineMarkerLinesUltra(predicted_area, roi_tl, marker.vertices, marker.line_accuracy);
					// check accuracy
					if(isAccurateEnough(marker.line_accuracy)) {
						// accept marker if id was correctly decoded
						stage3_failed = false;
						markers.push_back(marker);
						++stage3_;
					}
				}
			}
			no_candidate_ += stage3_failed && num_corners_in_image == 4;
		}

		/*if(num_corners_in_image == 4 && stage1_failed && stage2_failed && stage3_failed) {
			string filename(intToString(num_frames).append("-").append(intToString(i)).append(".png"));
			imwrite(filename, predicted_area);
		}*/

		if(markers.size() >= max_marker_) {
			return(markers);
		}
	}

	return(markers);
}

} /* namespace fml */
