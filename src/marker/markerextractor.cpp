/*
 * markerextractor.cpp
 *
 *  Created on: 01.08.2015
 *      Author: Jung
 */

#include "marker/markerextractor.h"

namespace fml {

MarkerExtractor::MarkerExtractor() {
	max_marker_ = numeric_limits<size_t>::max();
	num_frames = 0;
}

MarkerExtractor::~MarkerExtractor() {
}

void MarkerExtractor::setMaxMarker(const size_t value) {
	max_marker_ = value;
}

cv::Vec4f MarkerExtractor::fitLine(vector<cv::Point2f> &points) const {
	// undistort all the points
	Undistorter::undistortPoints(points);

	float num_points_determined = points.size();
	if(num_points_determined > 2) {
		// fit a line through these points
		float rnum = 1.0f/num_points_determined;
		cv::Point2f mean_point(points[0]);
		for(size_t i = 1; i < num_points_determined; ++i) {
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
			for(size_t i = 0; i < num_points_determined; ++i) {
				float x_dev = points[i].x - mean_point.x;
				float y_dev = points[i].y - mean_point.y;
				x_y_dev += x_dev * y_dev;
				x_dev_p2 += x_dev * x_dev;
			}

			out_line[0] = x_dev_p2;
			out_line[1] = x_y_dev;
		} else {
			float x_y_dev = 0, y_dev_p2 = 0;
			for(size_t i = 0; i < num_points_determined; ++i) {
				float x_dev = points[i].x - mean_point.x;
				float y_dev = points[i].y - mean_point.y;
				x_y_dev += x_dev * y_dev;
				y_dev_p2 += y_dev * y_dev;
			}

			out_line[0] = x_y_dev;
			out_line[1] = y_dev_p2;
		}

		return(out_line);
	} else {	// simply take the line defined by the two points
		return(cv::Vec4f(points[1].x-points[0].x, points[1].y-points[0].y, points[0].x, points[0].y));
	}
}

/**
 * Refine a line
 *
 * maybe we can do this a couple of iterations in order to acchieve better results
 *
 * @param image
 * @param roi_tl
 * @param a
 * @param b
 * @return
 */
cv::Vec4f MarkerExtractor::refineLine(const cv::Mat &image, const cv::Point2f &roi_tl, const cv::Point2f &start, const cv::Point2f &end, const int iteration) const {
	const int search_length = 3;
	const int num_points = 3;

	int max_row = image.rows - 1;
	int max_col = image.cols - 1;

	int outside_limit = search_length - iteration;
	int max_index = 2 * outside_limit;
	int min_req_index = max_index - 1;

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
		// we have no chance of determining a line
		points.clear();
		points.reserve(2);
		points.push_back(start + roi_tl);
		points.push_back(end + roi_tl);
		// undistort all the points
		Undistorter::undistortPoints(points);

		return(cv::Vec4f(points[1].x-points[0].x, points[1].y-points[0].y, points[0].x, points[0].y));
	}

	vector *= 1.0f/SQRT(vector.x*vector.x + vector.y*vector.y); // normalized to |v|=1
	cv::Point2f normal(vector.y, -vector.x); // normal to vector with |v|=1 pointing towards the white border
	sample_float -= outside_limit * normal; // start point

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
				int x1 = x;
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
				intensity[j] = interp_intensity;

				// do nearest neighbor interpolation
				// speedy version of ROUND(for positive values);
				/*int x1 = (int)(x + 0.5f);
				int y1 = (int)(y + 0.5f);
				intensity[j] = image.at<uint8_t>(y1, x1);*/

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
				points.push_back(roi_tl + sample_float + position*normal);
			}
		}
		sample_float += offset;
	}

	int num_points_determined = points.size();
	if(likely(num_points_determined > 1)) {
		// undistort all the points
		Undistorter::undistortPoints(points);

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
		//if(num_points_determined < 2) {
			points.clear();
			points.reserve(2);
			points.push_back(start + roi_tl);
			points.push_back(end + roi_tl);
		//}
		// undistort all the points
		Undistorter::undistortPoints(points);

		return(cv::Vec4f(points[1].x-points[0].x, points[1].y-points[0].y, points[0].x, points[0].y));
	}
}

cv::Vec4f MarkerExtractor::refineLineUltra(const cv::Mat &image, const cv::Point2f &roi_tl, const cv::Point2f &start, const cv::Point2f &end, float &line_residual, const int iteration) const {
	const int search_length = 3;
	const int num_points = 12;

	int max_row = image.rows - 1;
	int max_col = image.cols - 1;

	int outside_limit = search_length - iteration;
	int max_index = 2 * outside_limit;
	int min_req_index = max_index - 1;

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
		// we have no chance of determining a line
		points.clear();
		points.reserve(2);
		points.push_back(start + roi_tl);
		points.push_back(end + roi_tl);
		// undistort all the points
		Undistorter::undistortPoints(points);

		return(cv::Vec4f(points[1].x-points[0].x, points[1].y-points[0].y, points[0].x, points[0].y));
	}

	vector *= 1.0f/SQRT(vector.x*vector.x + vector.y*vector.y); // normalized to |v|=1
	cv::Point2f normal(vector.y, -vector.x); // normal to vector with |v|=1 pointing towards the white border
	sample_float -= outside_limit * normal; // start point

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
				int x1 = x;
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
				intensity[j] = interp_intensity;

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
				points.push_back(roi_tl + sample_float + position*normal);
			}
		}
		sample_float += offset;
	}

	int num_points_determined = points.size();
	if(likely(num_points_determined > 1)) {
		// undistort all the points
		Undistorter::undistortPoints(points);

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
			line_residual = residual(points, out_line, true);
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
			line_residual = residual(points, out_line, false);
		}

		return(out_line);
	} else {
		points.clear();
		points.reserve(2);
		points.push_back(start + roi_tl);
		points.push_back(end + roi_tl);
		// undistort all the points
		Undistorter::undistortPoints(points);

		return(cv::Vec4f(points[1].x-points[0].x, points[1].y-points[0].y, points[0].x, points[0].y));
	}
}

/**
 * Move this to marker.h
 *
 * @param image
 * @param vertices
 */
void MarkerExtractor::refineMarkerLines(const cv::Mat &image, const cv::Point2f &roi_tl, cv::Point2f (&vertices)[4], const float projected_length, const int iterations) const {
	bool reset = false;
	vector<cv::Point2f> refined_points(4);
	for(int i = 0; i < iterations; ++i) {
		cv::Vec4f lines[4];
		// first refine the lines
		//double time_ms = 1000000.0/cv::getTickFrequency();
		//double before = (double)cv::getTickCount() * time_ms;
		lines[0] = refineLine(image, roi_tl, vertices[0], vertices[1], i);
		lines[1] = refineLine(image, roi_tl, vertices[1], vertices[2], i);
		lines[2] = refineLine(image, roi_tl, vertices[2], vertices[3], i);
		lines[3] = refineLine(image, roi_tl, vertices[3], vertices[0], i);
		//double after = (double)cv::getTickCount() * time_ms;
		//cout << after-before << endl;

		// marker corners are then refined to the crossing of two subsequent lines (very accurately)
		vertices[0] = getCrossing(lines[3], lines[0]);
		vertices[1] = getCrossing(lines[0], lines[1]);
		vertices[2] = getCrossing(lines[1], lines[2]);
		vertices[3] = getCrossing(lines[2], lines[3]);

		// unfortunately sometimes the marker borders are not properly detected:
		// instead of the markers own b/w border the border between the outer white frame and the surrounding environment is detected
		// in this case we have to subtract the border and rescan
		if(projected_length > 0) {
			array<float, 4> side_length = computeSideLengths(vertices);
			float mean_dev = 0.25f * (side_length[0] + side_length[1] + side_length[2] + side_length[3]) - projected_length;
			float max_dev = (float)(1.0/7.0) * projected_length;
			if(unlikely(mean_dev > max_dev)) {
				// we assume the quiet zone around the border to be roughly symmetric
				float resize_factor = (float)(1.0/sqrt_2) * mean_dev;
				// correct 0->2
				cv::Point2f v1 = vertices[2] - vertices[0];
				cv::Point2f v2 = vertices[3] - vertices[1];
				resize_factor *= 2.0f/(SQRT(v1.x*v1.x + v1.y*v1.y) + SQRT(v2.x*v2.x + v2.y*v2.y));
				v1 *= resize_factor;
				vertices[0] = vertices[0] + v1;
				vertices[2] = vertices[2] - v1;
				// correct 1->3
				v2 *= resize_factor;
				vertices[1] = vertices[1] + v2;
				vertices[3] = vertices[3] - v2;
				if(!reset) {
					// avoid bouncing indefinitely
					reset = true;
					--i;
				}
			}
		}

		// distort points for additional iteration
		// last iteration stays undistorted
		if(i < iterations-1) {
			copy(vertices, vertices+4, refined_points.begin());
			Undistorter::distortPoints(refined_points);
			vertices[0] = refined_points[0] - roi_tl;
			vertices[1] = refined_points[1] - roi_tl;
			vertices[2] = refined_points[2] - roi_tl;
			vertices[3] = refined_points[3] - roi_tl;
		}
	}
}

void MarkerExtractor::refineMarkerLinesUltra(const cv::Mat &image, const cv::Point2f &roi_tl, cv::Point2f (&vertices)[4], float (&residuals)[4], const int iterations) const {
	vector<cv::Point2f> refined_points(4);
	for(int i = 0; i < iterations; ++i) {
		cv::Vec4f lines[4];
		// first refine the lines
		lines[0] = refineLineUltra(image, roi_tl, vertices[0], vertices[1], residuals[0], i);
		lines[1] = refineLineUltra(image, roi_tl, vertices[1], vertices[2], residuals[1], i);
		lines[2] = refineLineUltra(image, roi_tl, vertices[2], vertices[3], residuals[2], i);
		lines[3] = refineLineUltra(image, roi_tl, vertices[3], vertices[0], residuals[3], i);

		// marker corners are then refined to the crossing of two subsequent lines (very accurately)
		vertices[0] = getCrossing(lines[3], lines[0]);
		vertices[1] = getCrossing(lines[0], lines[1]);
		vertices[2] = getCrossing(lines[1], lines[2]);
		vertices[3] = getCrossing(lines[2], lines[3]);

		// distort points for additional iteration
		// last iteration stays undistorted
		if(i < iterations-1) {
			copy(vertices, vertices+4, refined_points.begin());
			Undistorter::distortPoints(refined_points);
			vertices[0] = refined_points[0] - roi_tl;
			vertices[1] = refined_points[1] - roi_tl;
			vertices[2] = refined_points[2] - roi_tl;
			vertices[3] = refined_points[3] - roi_tl;
		}
	}
}

void MarkerExtractor::reconstructShape(cv::Point2f (&vertices)[4], const bool (&inside_image)[4], const float projected_length) const {
	cv::Point2f center_before = computeCenter(vertices) + computeCenter2(vertices); // factor 0.5f is cancelled out
	array<float, 4> angles = computeAngles(vertices);

	// find best angle and its index
	float best_angle_deviation = FLT_MAX;
	size_t best_angle_index = 0;
	for(size_t i = 0; i < 4; ++i) {
		//if(inside_image[i]) {
			float angle_deviation = g_PI-FABS(g_PI-FABS(angles[i]-g_PI_2));
			if(angle_deviation < best_angle_deviation) {
				best_angle_index = i;
				best_angle_deviation = angle_deviation;
			}
		//}
	}

	// these indices denote the remaining points
	size_t next_index = MOD4((best_angle_index+1));
	size_t prev_index = MOD4((best_angle_index+3));
	size_t opp_index = MOD4((best_angle_index+2));

	// now we start at the allegedly best point to reconstruct the square
	// compute successor and predecessor
	cv::Point2f v1(vertices[next_index] - vertices[best_angle_index]);
	v1 *= projected_length/SQRT(v1.x*v1.x + v1.y*v1.y);
	cv::Point2f v2(vertices[prev_index] - vertices[best_angle_index]);
	v2 *= projected_length/SQRT(v2.x*v2.x + v2.y*v2.y);

	vertices[next_index] = vertices[best_angle_index] + v1;
	vertices[prev_index] = vertices[best_angle_index] + v2;
	// compute opposing point
	vertices[opp_index] = vertices[next_index] + vertices[prev_index] - vertices[best_angle_index];

	// difference between before and after including the 0.5f factor from both values
	cv::Point2f offset(0.25f * (center_before - (computeCenter(vertices) + computeCenter2(vertices))));
	// move center_after half-way between center_after and center_before
	vertices[0] += offset;
	vertices[1] += offset;
	vertices[2] += offset;
	vertices[3] += offset;
}

/**
 * Some markers might only be partially detected. in those cases we chose to drop them in favor of other measurements, if any
 *
 * @param marker
 * @return
 */
bool MarkerExtractor::keepMarker(const MarkerInfo &marker) const {
	float length0 = squared_distance(marker.vertices[0], marker.vertices[1]);
	float length1 = squared_distance(marker.vertices[2], marker.vertices[3]);
	float length_ratio = (length0 > length1) ? length0/length1 : length1/length0;
	length0 = squared_distance(marker.vertices[1], marker.vertices[2]);
	length1 = squared_distance(marker.vertices[3], marker.vertices[0]);
	length_ratio = max(length_ratio, (length0 > length1) ? length0/length1 : length1/length0);
	length0 = squared_distance(marker.vertices[0], marker.vertices[2]);
	length1 = squared_distance(marker.vertices[1], marker.vertices[3]);
	length_ratio = max(length_ratio, (length0 > length1) ? length0/length1 : length1/length0);

	return(length_ratio < opp_length_ratio);
}

bool MarkerExtractor::isAccurateEnough(const float (&line_accuracy)[4]) const {
	bool pair1_ok = (line_accuracy[0] >= 0 && line_accuracy[0] <= Config::tracker_parameters.min_line_quality) || (line_accuracy[2] >= 0 && line_accuracy[2] <= Config::tracker_parameters.min_line_quality);
	bool pair2_ok = (line_accuracy[1] >= 0 && line_accuracy[1] <= Config::tracker_parameters.min_line_quality) || (line_accuracy[3] >= 0 && line_accuracy[3] <= Config::tracker_parameters.min_line_quality);
	bool pass = pair1_ok && pair2_ok;
	/*if(!pass) {
		cout << line_accuracy[0] << "," << line_accuracy[1] << "," << line_accuracy[2] << "," << line_accuracy[3] << endl;
	}*/
	return(pass);
}

} /* namespace fml */
