/*
 * PerfectMarkerPredictor.cpp
 *
 *  Created on: 19.01.2015
 *      Author: jung
 */

#include "marker/perfectmarkerpredictor.h"

namespace fml {

PerfectMarkerPredictor::PerfectMarkerPredictor() {
	// create sorted vector for x and y direction
	comparison_value_ = new MarkerInfo();
	sorted_by_x_ = vector<MarkerInfo*>(Config::marker_parameters.base_T_m.size());
	float marker_max_height = 0;
	int i = 0;
	for(map<int, Pose2d>::const_iterator it = Config::marker_parameters.base_T_m.begin(); it != Config::marker_parameters.base_T_m.end(); ++it) {
		marker_max_height = max(marker_max_height, it->second.translation3d[2] - Config::camera_parameters.z_offset);
		float length = Config::tracker_parameters.special_marker_sizes_mm.count(it->first) ? Config::tracker_parameters.special_marker_sizes_mm[it->first] : Config::tracker_parameters.marker_size_mm;
		sorted_by_x_[i] = new MarkerInfo(length, it->second);
		++i;
	}
	// sort according to x position
	sort(sorted_by_x_.begin(), sorted_by_x_.end(), cmp_sort_x);

	// compute rotationally invariant and hence static cutoff radius for our camera's field of view
	// this is a conservative approach, the radius could be smaller, but don't care for now
	// a good approximation would also be to just compute the difference between 0,0 and c_x,c_y, because the image center is near the principal point (c_x, c_y)
	float factor = marker_max_height/Config::calibration_parameters.f.x;
	float x_min = factor * -Config::calibration_parameters.c.x;
	float x_max = factor * (Config::camera_parameters.camera_size.width - Config::calibration_parameters.c.x);
	factor = marker_max_height/Config::calibration_parameters.f.y;
	float y_min = factor * -Config::calibration_parameters.c.y;
	float y_max = factor * (Config::camera_parameters.camera_size.height - Config::calibration_parameters.c.y);
	// use whichever offset is bigger (since the calibrated center is not necessarily in the middle)
	offset_xy_ = max(SQRT(x_min*x_min + y_min*y_min), SQRT(x_max*x_max + y_max*y_max));

	// determine precomputed marker corners
	// additional 9.0/7.0 factor is because we need the white border of the marker for edge detection as well, whilst the marker size only covers the black contents.
	projected_corners_by_size_.insert(pair<float, MarkerProjectedCorners*>(Config::tracker_parameters.marker_size_mm, new MarkerProjectedCorners(max_error_mm + (float)(9.0/14.0) * Config::tracker_parameters.marker_size_mm, 0.5f * Config::tracker_parameters.marker_size_mm)));
	for(map<int, float>::const_iterator it = Config::tracker_parameters.special_marker_sizes_mm.begin(); it != Config::tracker_parameters.special_marker_sizes_mm.end(); ++it) {
		// add new one if we do not have this one yet,
		if(projected_corners_by_size_.count(it->second) == 0) {
			projected_corners_by_size_.insert(pair<float, MarkerProjectedCorners*>(it->second, new MarkerProjectedCorners(max_error_mm + (float)(9.0/14.0) * it->second, 0.5f * it->second)));
		}
	}

	num_rois_ = 0;
	num_rois_completely_inside_ = 0;
}

PerfectMarkerPredictor::~PerfectMarkerPredictor() {
	cerr << "number_of_rois considered: " << num_rois_ << endl;
	cerr << "number_of_rois completely in image: " << num_rois_completely_inside_ << endl;
	for(size_t i = 0; i < sorted_by_x_.size(); ++i) {
		delete sorted_by_x_[i];
	}
	// cleanup the map with the precomputed marker corners ready for projection
	for(map<float, MarkerProjectedCorners*>::iterator it = projected_corners_by_size_.begin(); it != projected_corners_by_size_.end(); ++it) {
		delete it->second;
	}
	delete comparison_value_;
}

/**
 * Saves us some Pose products
 *
 * @param lhs
 * @param rhs
 * @return
 */
cv::Point3f PerfectMarkerPredictor::pointCoordinate(const Pose2d &lhs, const Pose2d &rhs) const {
	float x_left = lhs.transformation[0];
	float y_left = lhs.transformation[4+0];

	float x = lhs.transformation[3] + rhs.transformation[3] * x_left - rhs.transformation[4+3] * y_left;
	float y = lhs.transformation[4+3] + rhs.transformation[4+3] * x_left + rhs.transformation[3] * y_left;
	float z = lhs.transformation[8+3] + rhs.transformation[8+3];

	return(cv::Point3f(x, y, z));
}

bool PerfectMarkerPredictor::predict(const Pose2d &w_T_c, vector<MarkerROI> &rois) {
	rois.clear();
	rois.reserve(20);	// TODO: is this a reasonable value?
	// w_T_c is predicted position, so invert predicted pose to get c_T_w
	Pose2d c_T_w(w_T_c);
	c_T_w.invert();
	// determine markers that could be in our fov
	// boundary values for x and y in world
	float margin = offset_xy_ - max_error_mm;
	float fov_x_min = w_T_c.translation3d[0] - margin;
	float fov_x_max = w_T_c.translation3d[0] + margin;
	float fov_y_min = w_T_c.translation3d[1] - margin;
	float fov_y_max = w_T_c.translation3d[1] + margin;
	// find iterators begin and end in x-range

	comparison_value_->w_T_m.translation3d[0] = fov_x_min;
	// determine upper and lower bound in the sorted section
	vector<MarkerInfo*>::iterator x_min_it = lower_bound(sorted_by_x_.begin(), sorted_by_x_.end(), comparison_value_, cmp_sort_x);
	comparison_value_->w_T_m.translation3d[0] = fov_x_max;
	vector<MarkerInfo*>::iterator x_max_it = upper_bound(x_min_it, sorted_by_x_.end(), comparison_value_, cmp_sort_x);

	// ADVICE: if you find the following loop to be your bottleneck (unlikely), implement a quadtree to find all the candidates fast
	for(vector<MarkerInfo*>::const_iterator it = x_min_it; it != x_max_it; ++it) {
		float y = (*it)->w_T_m.translation3d[1];

		// use if it is within cutoff
		if(y >= fov_y_min && y <= fov_y_max) {	// we already checked the x-value, so now we have to test for suitable y-values
			// if we get this far, this means the marker center point will be visible inside the cutoff radius => test if it is in our actual fov
			int marker_id = (*it)->id;

			// marker in camera coordinates: c_T_m = c_T_w * w_T_m
			Pose2d c_T_m(c_T_w * (*it)->w_T_m);

			// the goal is to accept all that could theoretically be in there and try to use them in the order of increasing distance from the center
			vector<cv::Point3f> object_points;
			object_points.reserve(8);

			float marker_length = Config::tracker_parameters.special_marker_sizes_mm.count(marker_id) ? Config::tracker_parameters.special_marker_sizes_mm[marker_id] : Config::tracker_parameters.marker_size_mm;
			MarkerProjectedCorners *corners = projected_corners_by_size_[marker_length];
			// marker area including tolerance
			for(size_t i = 0; i < 8; ++i) {
				object_points.push_back(pointCoordinate(c_T_m, corners->points[i]));
			}

			vector<cv::Point2f> projected_points;
			// project the object points to the distorted image
			Undistorter::projectPoints(object_points, projected_points);
			// omit the roi if one of the two following conditions is met:
			// 1) more than 3 points are projected outside the image or very near the image border (within border margin or beyond)
			// 2) if two points are projected outside the image, the must not be too far out (see -border_margin)
			// reason: we get biased measurements and hence cannot expect a good result in this case
			MarkerROI marker_roi(marker_id);

			const float border_margin = 0;
			int points_outside_image = 0;
			for(size_t i = 4; i < 8; ++i) {
				float x_dist = min(projected_points[i].x, Config::camera_parameters.camera_size.width - 1 - projected_points[i].x);
				float y_dist = min(projected_points[i].y, Config::camera_parameters.camera_size.height - 1 - projected_points[i].y);
				bool outside_image = min(x_dist, y_dist) < border_margin;
				marker_roi.corner_inside_image[i-4] = !outside_image;
				points_outside_image += outside_image;
			}
			num_rois_completely_inside_ += points_outside_image == 0;

			// compute distance to all image borders and determine shortest one for each roi
			if(points_outside_image < 3 && getMarkerROI(projected_points, marker_roi, Config::camera_parameters.camera_size)) {
				//if(points_outside_image < 2 && getMarkerROI(projected_points, marker_roi, image_size_)) {
				// how long each side will be in the image
				cv::Point2f corners[4] = { projected_points[4], projected_points[5], projected_points[6], projected_points[7] };
				float fx_weight = computeFXWeight(corners, Config::calibration_parameters.c);
				float f_wmean = fx_weight * Config::calibration_parameters.f.x + (1-fx_weight) * Config::calibration_parameters.f.y;
				float line_length = (marker_length * f_wmean)/((*it)->w_T_m.translation3d[2] - Config::camera_parameters.z_offset);

				/*int min_line_length = (int)FLOOR(1.0f/sqrt_2 * line_length);
				int max_line_length = (int)CEIL(sqrt_2 * line_length);*/
				if(min(marker_roi.roi.height, marker_roi.roi.width) >= line_length) {
					++num_rois_;
					// set min/max allowed line length to roi
					marker_roi.projected_line_length[0] = line_length;	// target
					marker_roi.projected_line_length[1] = FLOOR((1.0f - Config::tracker_parameters.marker_tolerance) * line_length); // min threshold
					marker_roi.projected_line_length[2] = CEIL((1.0f + Config::tracker_parameters.marker_tolerance) * line_length); // max threshold

					// add projected points for the original marker corners (places 4-8)
					marker_roi.corners.push_back(projected_points[4]);
					marker_roi.corners.push_back(projected_points[5]);
					marker_roi.corners.push_back(projected_points[6]);
					marker_roi.corners.push_back(projected_points[7]);

					float dx = c_T_m.translation3d[0];
					float dy = c_T_m.translation3d[1];
					marker_roi.distance = dx*dx + dy*dy;

					rois.push_back(marker_roi);
				}
			}
		}
	}
	sort(rois.begin(), rois.end(), cmp_sort_dist);
	return(!rois.empty());
}

}
