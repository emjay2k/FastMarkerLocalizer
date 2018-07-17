/**
 * definitions.h
 *
 *  Created on: 13.02.2012
 *      Author: jung
 */

#ifndef MARKER_H_
#define MARKER_H_

#include <array>
#include <list>
#include <opencv2/opencv.hpp>
#include "location/pose2d.h"

using namespace std;

namespace fml {

struct MarkerRelation2D {
	clock_t timestamp;
	int src_id;
	int dst_id;
	int corners_in_image;
	float x, y, angle;
	float error;

	MarkerRelation2D(const clock_t timestamp, const int src, const int dst, const float x, const float y, const float angle, const float error, const int corners_in_image=0) {
		this->timestamp = timestamp;
		this->src_id = src;
		this->dst_id = dst;
		this->x = x;
		this->y = y;
		this->angle = angle;
		this->error = error;
		this->corners_in_image = corners_in_image;
	}
};

struct MarkerInfo {
	int id;
	int detection_error;
	float length_mm;
	Pose2d w_T_m;
	cv::Point2f vertices[4];
	float line_accuracy[4] = { -1, -1, -1, -1 };

	MarkerInfo() {
		id = -1;
		detection_error = 0;
		length_mm = 0;
		w_T_m = Pose2d();
	}

	MarkerInfo(const float length_mm, const Pose2d &w_T_m) {
		this->id = w_T_m.id;
		this->detection_error = 0;
		this->length_mm = length_mm;
		this->w_T_m = w_T_m;
	}

	void addOffset(const cv::Point roi_base) {
		for(size_t i = 0; i < 4; ++i) {
			vertices[i].x += roi_base.x;
			vertices[i].y += roi_base.y;
		}
	}

    friend bool operator<(const MarkerInfo &mi1, const MarkerInfo &mi2) {
        return(mi1.id < mi2.id);
    }
};

struct MarkerROI {
	int id;
	int distance;
	cv::Rect roi;
	float projected_line_length[3] = { 0, 0, 0 }; // target length, min threshold, max threshold
	bool corner_inside_image[4] = { true, true, true, true };
	vector<cv::Point2f> corners;

	MarkerROI(const int id) {
		this->id = id;
		this->distance = 0;
		corners.reserve(4);
	}
};

struct MarkerProjectedCorners {
	Pose2d points[8];

	MarkerProjectedCorners(const float marker_check_length, const float half_inner_marker) {
		points[0] = Pose2d(1, 0, 0, 0, -marker_check_length, -marker_check_length, 0, 0);
		points[1] = Pose2d(1, 0, 0, 0, +marker_check_length, -marker_check_length, 0, 0);
		points[2] = Pose2d(1, 0, 0, 0, +marker_check_length, +marker_check_length, 0, 0);
		points[3] = Pose2d(1, 0, 0, 0, -marker_check_length, +marker_check_length, 0, 0);
		points[4] = Pose2d(1, 0, 0, 0, -half_inner_marker, -half_inner_marker, 0, 0);
		points[5] = Pose2d(1, 0, 0, 0, +half_inner_marker, -half_inner_marker, 0, 0);
		points[6] = Pose2d(1, 0, 0, 0, +half_inner_marker, +half_inner_marker, 0, 0);
		points[7] = Pose2d(1, 0, 0, 0, -half_inner_marker, +half_inner_marker, 0, 0);
	}
};

inline bool getMarkerROI(const vector<cv::Point2f> &corners, MarkerROI &marker_roi, const cv::Size &image_size) {
	float min_x = corners[0].x, min_y = corners[0].y;
	float max_x = min_x, max_y = min_y;
	for(size_t i = 1; i < 4; ++i) {
		if(corners[i].x < min_x)
			min_x = corners[i].x;
		else if(corners[i].x > max_x)
			max_x = corners[i].x;
		if(corners[i].y < min_y)
			min_y = corners[i].y;
		else if(corners[i].y > max_y)
			max_y = corners[i].y;
	}

	min_x = FLOOR(max(min_x, 0.f));
	min_y = FLOOR(max(min_y, 0.f));
	max_x = CEIL(min(max_x, (float)image_size.width));
	max_y = CEIL(min(max_y, (float)image_size.height));

	// not in image (now x,y matters, before it was just a cutoff radius)
	if(min_x >= max_x || min_y >= max_y) {
		return(false);
	}

	marker_roi.roi.x = min_x;
	marker_roi.roi.y = min_y;
	marker_roi.roi.width = max_x - min_x;
	marker_roi.roi.height = max_y - min_y;
	return(true);
}

inline cv::Rect getBoundingBox(const cv::Point2f (&corners)[4], const cv::Size image_size, const int border) {
	float min_x = corners[0].x, min_y = corners[0].y;
	float max_x = min_x, max_y = min_y;
	for(size_t i = 1; i < 4; ++i) {
		if(corners[i].x < min_x)
			min_x = corners[i].x;
		else if(corners[i].x > max_x)
			max_x = corners[i].x;
		if(corners[i].y < min_y)
			min_y = corners[i].y;
		else if(corners[i].y > max_y)
			max_y = corners[i].y;
	}

	min_x = FLOOR(max(min_x-border, 0.f));
	min_y = FLOOR(max(min_y-border, 0.f));
	max_x = CEIL(min(max_x+border, (float)(image_size.width-1)));
	max_y = CEIL(min(max_y+border, (float)(image_size.height-1)));

	return(cv::Rect(min_x, min_y, max_x-min_x+1, max_y-min_y+1));
}

/**
 * Find the crossing of the two lines described by X = b + k * f. Use analytical formula instead of solving the equations all the time.
 *
 * @param b1
 * @param f1
 * @param b2
 * @param f2
 * @return
 */
inline cv::Point2f getCrossing(const cv::Vec4f &line1, const cv::Vec4f &line2) {
	// extract the points from the line
	cv::Point2f f1(line1.val[0], line1.val[1]);
	cv::Point2f f2(line2.val[0], line2.val[1]);
	cv::Point2f b1(line1.val[2], line1.val[3]);
	cv::Point2f b2(line2.val[2], line2.val[3]);
	// compute parameter k:
	// insert into first equation: crossing is then b1 + k * f1*/
	float k = (f2.x * (b1.y-b2.y) - f2.y * (b1.x-b2.x)) / (f1.x*f2.y - f1.y*f2.x);
	return(b1 + k * f1);
	// compute parameter l:
	// insert into second equation: crossing is then b2 + l * f2
	/*float l = (f1.x * (b1.y-b2.y) - f1.y * (b1.x-b2.x)) / (f1.x*f2.y - f1.y*f2.x);
	return(b2 + l * f2);*/
}

// determine the minimal squared distance from a point to a line
inline float signedDistToLine(const int x, const int y, const cv::Vec4f &line) {
	const float factor[2] { -1, 1 };
	float dx2 = x - line[2];
	float dy2 = y - line[3];
	float t = (line[0]*dx2 + line[1]*dy2)/(line[0]*line[0] + line[1]*line[1]);
	float dx = line[0] * t - dx2;
	float dy = line[1] * t - dy2;
	float retval = SQRT(dx*dx + dy*dy);
	//float sign = copysignf(1.0f, dx+dy);

	int o = line[0]*dy2 - dx2*line[1];
	float sign = factor[o < 0];

	return(sign*retval);
}

inline float residual(vector<cv::Point2f> &points, cv::Vec4f &line, bool f_x) {
	// mean by construction of the line
	float mean_x = line[2];
	float mean_y = line[3];
	float rnum = 1.0f/points.size();

	float var_unexplained = 0;
	if(f_x) {
		float vdiv = line[1]/line[0];
		for(size_t i = 0; i < points.size(); ++i) {
			float y_value = (points[i].x - mean_x) * vdiv + mean_y;
			float dy_unexplained = points[i].y - y_value;
			var_unexplained += dy_unexplained * dy_unexplained;
		}
	} else {
		float vdiv = line[0]/line[1];
		for(size_t i = 0; i < points.size(); ++i) {
			float x_value = (points[i].y - mean_y) * vdiv + mean_x;
			float dx_unexplained = points[i].x - x_value;
			var_unexplained += dx_unexplained * dx_unexplained;
		}
	}

	return(var_unexplained * rnum);
}

// when applying distance constraints we spare ourselves the square root and compare the squared values instead
inline size_t squared_distance(const cv::Point &p1, const cv::Point &p2) {
	int a = p2.y - p1.y;
	int b = p2.x - p1.x;

	return(a*a + b*b);
}

// when applying distance constraints we spare ourselves the square root and compare the squared values instead
inline float squared_distance(const cv::Point2f &p1, const cv::Point2f &p2) {
	float a = p2.y - p1.y;
	float b = p2.x - p1.x;

	return(a*a + b*b);
}

inline cv::Point2f computeCenter(const cv::Point2f (&vertices)[4]) {
	float sum_x = vertices[0].x + vertices[1].x + vertices[2].x + vertices[3].x;
	float sum_y = vertices[0].y + vertices[1].y + vertices[2].y + vertices[3].y;

	return(cv::Point2f(0.25f*sum_x, 0.25f*sum_y));
}

inline cv::Point2f computeCenter2(const cv::Point2f (&vertices)[4]) {
	cv::Point2f b1(vertices[0]);
	cv::Point2f b2(vertices[1]);
	cv::Point2f f1(vertices[2] - vertices[0]);
	cv::Point2f f2(vertices[3] - vertices[1]);
	cv::Vec4f line1(f1.x, f1.y, b1.x, b1.y);
	cv::Vec4f line2(f2.x, f2.y, b2.x, b2.y);

	return(getCrossing(line1, line2));
}

/**
 * Determine the center point based on two sequential edges of the quatrilateral. We determine the angle bisector and compute its crossing between the vector p1->p2
 * This can be done analytically
 *
 * @param p1
 * @param p2
 * @param p3
 * @return
 */
inline cv::Point2f angleBisectorCrossing(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Point2f &p3) {
	cv::Point2f a(p3-p2);
	cv::Point2f b(p1-p2);
	cv::Point2f c(p3-p1);
	float length_a = SQRT(a.x*a.x + a.y*a.y);
	float length_b = SQRT(b.x*b.x + b.y*b.y);
	//float length_c = SQRT(c.x*c.x + c.y*c.y);

	float c1_by_c = length_b/(length_a + length_b);
	return(p1 + c1_by_c * c);
}

inline cv::Point2f computeCenter(const cv::Point2f (&vertices)[4], const cv::Point2f p_center) {
	const size_t num_measurements = 8;
	const size_t num_considered_values = 6; //(num_measurements >> 1) + 1;
	cv::Point2f centers[num_measurements];
	float weights[num_measurements];

	size_t index = 0;
	// the 2 opposing corner combos
	// 02
	centers[index++] = 0.5f * (vertices[0] + vertices[2]);
	// 13
	centers[index++] = 0.5f * (vertices[1] + vertices[3]);
	// no min enclosing circle computation here, because it will give the same result as above

	// all 3 combos (determine perpendicular bisector crossing and min enclosing circle)
	// 012
	centers[index++] = angleBisectorCrossing(vertices[0], vertices[1], vertices[2]);
	//centers[index++] = perpendicularBisectorCrossing(vertices[0], vertices[1], vertices[2]);
	// min enclosing circle
	/*vector<cv::Point2f> vertices_copy;
	vertices_copy.reserve(4);

	vertices_copy.push_back(vertices[0]);
	vertices_copy.push_back(vertices[1]);
	vertices_copy.push_back(vertices[2]);
	centers[index++] = circleCenter(vertices_copy);*/

	// 123
	centers[index++] = angleBisectorCrossing(vertices[1], vertices[2], vertices[3]);
	//centers[index++] = perpendicularBisectorCrossing(vertices[1], vertices[2], vertices[3]);
	// min enclosing circle
	/*vertices_copy[0] = vertices[1];
	vertices_copy[1] = vertices[2];
	vertices_copy[2] = vertices[3];
	centers[index++] = circleCenter(vertices_copy);*/

	// 230
	centers[index++] = angleBisectorCrossing(vertices[2], vertices[3], vertices[0]);
	//centers[index++] = perpendicularBisectorCrossing(vertices[2], vertices[3], vertices[0]);
	// min enclosing circle
	/*vertices_copy[0] = vertices[2];
	vertices_copy[1] = vertices[3];
	vertices_copy[2] = vertices[0];
	centers[index++] = circleCenter(vertices_copy);*/

	// 301
	centers[index++] = angleBisectorCrossing(vertices[3], vertices[0], vertices[1]);
	//centers[index++] = perpendicularBisectorCrossing(vertices[3], vertices[0], vertices[1]);
	// min enclosing circle
	/*vertices_copy[0] = vertices[3];
	vertices_copy[1] = vertices[0];
	vertices_copy[2] = vertices[1];
	centers[index++] = circleCenter(vertices_copy);*/

	// all 4 (compute center as a) average of the points b) crossing of the two angle bisectors and c) min enclosing circle)
	centers[index++] = computeCenter(vertices);
	centers[index++] = computeCenter2(vertices);
	// min enclosing circle
	/*vertices_copy[0] = vertices[0];
	vertices_copy[1] = vertices[1];
	vertices_copy[2] = vertices[2];
	vertices_copy.push_back(vertices[3]);
	centers[index] = circleCenter(vertices_copy);*/

	// set weights as euclidean distances
	for(size_t i = 0; i < num_measurements; ++i) {
		//cout << centers[i].x << "," << centers[i].y << ";" << endl;
		float dx = centers[i].x - p_center.x;
		float dy = centers[i].y - p_center.y;
		weights[i] = dx*dx + dy*dy;
	}
	//cout << endl << endl;

	vector<float> sorted_weights(weights, weights+num_measurements);
	nth_element(sorted_weights.begin(), sorted_weights.begin()+num_considered_values, sorted_weights.end());

	// relax the border a little and keep all values lower than the border
	float max = sorted_weights[num_considered_values];
	float rmax = 1.0f/max;
	cv::Point2f center(0,0);
	float weight_sum = 0;
	for(size_t i = 0; i < num_measurements; ++i) {
		if(weights[i] < max) {
			float weight = 1 - SQRT(weights[i]*rmax);
			center += weight * centers[i];
			weight_sum += weight;
		}
	}

	return(center * (1.0f/weight_sum));
}

inline float computeFXWeight(const cv::Point2f (&vertices)[4], const cv::Point2f &image_center) {
	cv::Point2f vertices_center(computeCenter(vertices));
	float dx = FABS(vertices_center.x - image_center.x);
	float dy = FABS(vertices_center.y - image_center.y);
	return(dx/(dx+dy));
}

inline array<float, 4> computeSideLengths(const cv::Point2f (&vertices)[4]) {
	array<float, 4> lengths;
	for(size_t i = 0; i < 4; ++i) {
		int next = MOD4((i+1));
		float dx = vertices[next].x - vertices[i].x;
		float dy =  vertices[next].y - vertices[i].y;
		lengths[i] = SQRT(dx*dx + dy*dy);
	}

	return(lengths);
}

inline float computePerimeter(const cv::Point2f (&vertices)[4]) {
	array<float, 4> side_length = computeSideLengths(vertices);

	return(side_length[0] + side_length[1] + side_length[2] + side_length[3]);
}

inline float computePerimeter(const vector<cv::Point2f> &vertices) {
	float perimeter = 0;
	for(size_t i = 0; i < 4; ++i) {
		int next = MOD4((i+1));
		float dx = vertices[next].x - vertices[i].x;
		float dy =  vertices[next].y - vertices[i].y;
		perimeter += SQRT(dx*dx + dy*dy);
	}

	return(perimeter);
}

inline array<float, 4> computeAngles(const cv::Point2f (&vertices)[4]) {
	array<float, 4> angles;
	array<float, 4> lengths = computeSideLengths(vertices);

	for(size_t i = 0; i < 4; ++i) {
		int next = MOD4((i+1));
		int prev = MOD4((i+3));

		cv::Point2f v1(vertices[next] - vertices[i]);
		cv::Point2f v2(vertices[prev] - vertices[i]);
		angles[i] = ACOS(min((v1.x*v2.x + v1.y*v2.y)/(lengths[i] * lengths[prev]), 1.0f));
	}

	return(angles);
}

inline float computeQuadArea(const cv::Point2f (&vertices)[4]) {
    // use the cross products
    cv::Point2f v01(vertices[1] - vertices[0]);
    cv::Point2f v03(vertices[3] - vertices[0]);
    float area1 = FABS(v01.x*v03.y - v01.y*v03.x);
    cv::Point2f v21(vertices[1] - vertices[2]);
    cv::Point2f v23(vertices[3] - vertices[2]);
    float area2 = FABS(v21.x*v23.y - v21.y*v23.x);
    return(0.5f * (area2 + area1));
}

// sort comparators
/**
 * Compare by marker x position
 * @param a
 * @param b
 * @return
 */
inline bool cmp_sort_x(const MarkerInfo* const a, const MarkerInfo* const b) {
	return(a->w_T_m.translation3d[0] < b->w_T_m.translation3d[0]);
}

/**
 * Compare by roi distance to border
 * @param a
 * @param b
 * @return
 */
inline bool cmp_sort_dist(const MarkerROI a, const MarkerROI b) {
	return(a.distance < b.distance);
}

} /* namespace fml */

#endif /* MARKER_H_ */
