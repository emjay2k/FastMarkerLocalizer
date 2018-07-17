/*
 * pppestimator.cpp
 *
 *  Created on: 04.02.2015
 *      Author: jung
 */

#include "location/pppestimator.h"

namespace fml {

PPPEstimator::PPPEstimator() {
}

PPPEstimator::~PPPEstimator() {
}

// we only supply the weight for f_x, because f_y is 1-f_x
array<float, 4> PPPEstimator::computeFXWeight(const cv::Point2f (&vertices)[4]) const {
	array<float, 4> weights;
	for(size_t i = 0; i < 4; ++i) {
		int next = MOD4((i+1));
		float dx = FABS(0.5f * (vertices[next].x + vertices[i].x) - Config::calibration_parameters.c.x);
		float dy = FABS(0.5f * (vertices[next].y + vertices[i].y) - Config::calibration_parameters.c.y);
		weights[i] = dx/(dx+dy);
	}

	return(weights);
}

/**
 * Boah nach 2h Geometrie�berlegung hatte ich echt keine Lust mehr. Au�erdem ist diese Variante sch�n �bersichtlich
 * und leicht nachzuvollziehen :-)
 *
 * @param vertices
 * @return
 */
float PPPEstimator::computeRotation(const cv::Point2f (&vertices)[4], float (&accuracy)[4]) const {
	vector<float> angles(4);

	/*{
		float x = vertices[2].x - vertices[0].x;
		float y = vertices[2].y - vertices[0].y;
		float angle = ATAN2(y, x) - g_PI_4;
		angle += (angle > g_PI) * -g_PI2;
		angle += (angle < -g_PI) * g_PI2;
		angles[0] = angle;
	}
	{
		float x = vertices[1].x - vertices[3].x;
		float y = vertices[1].y - vertices[3].y;
		float angle = ATAN2(y, x) + g_PI_4;
		angle += (angle > g_PI) * -g_PI2;
		angle += (angle < -g_PI) * g_PI2;
		angles[1] = angle;
	}*/
	for(size_t i = 0; i < 4; ++i) {
		int next = MOD4((i+1));
		float x = vertices[next].x - vertices[i].x;
		float y = vertices[next].y - vertices[i].y;

		float angle = ATAN2(y, x) - (float)i * g_PI_2;
		angle += (angle > g_PI) * -g_PI2;
		angle += (angle < -g_PI) * g_PI2;
		angles[i] = angle;
	}
	for(size_t i = 0; i < 4; ++i) {
		int opp = MOD4((i+2));
		if(accuracy[i] > Config::tracker_parameters.min_line_quality && accuracy[opp] >= 0 && accuracy[opp] < accuracy[i]) {
			angles[i] = angles[opp];
		}
	}

	return(meanAngle(angles));
}

float PPPEstimator::computeRotation(const cv::Point2f (&vertices)[4], const float ref, float (&accuracy)[4]) const {
	const size_t num_measurements = 17;
	const size_t num_considered_values = 9; //(num_measurements >> 1) + 1;
	vector<float> angles(num_measurements);
	vector<float> weights(num_measurements);
	float separate_angles[4][2];

	// all individual angles
	int index = 0;
	for(; index < 4; ++index) {
		int next = MOD4((index+1));
		float x = vertices[next].x - vertices[index].x;
		float y = vertices[next].y - vertices[index].y;

		float angle = ATAN2(y, x) - (float)index * g_PI_2;
		angle += (angle > g_PI) * -g_PI2;
		angle += (angle < -g_PI) * g_PI2;
		angles[index] = angle;
		weights[index] = g_PI-FABS(g_PI-FABS(angle-ref));
		separate_angles[index][0] = COS(angle);
		separate_angles[index][1] = SIN(angle);
	}
	// replace angles with bad residual
	for(size_t i = 0; i < 4; ++i) {
		int opp = MOD4((i+2));
		if(accuracy[i] > Config::tracker_parameters.min_line_quality && accuracy[opp] >= 0 && accuracy[opp] < accuracy[i]) {
			angles[i] = angles[opp];
			weights[i] = weights[opp];
			separate_angles[i][0] = separate_angles[opp][0];
			separate_angles[i][1] = separate_angles[opp][1];
		}
	}
	{
		float x = vertices[2].x - vertices[0].x;
		float y = vertices[2].y - vertices[0].y;
		float angle = ATAN2(y, x) - g_PI_4;
		angle += (angle > g_PI) * -g_PI2;
		angle += (angle < -g_PI) * g_PI2;
		angles[index] = angle;
		weights[index] = g_PI-FABS(g_PI-FABS(angle-ref));
		++index;
	}
	{
		float x = vertices[1].x - vertices[3].x;
		float y = vertices[1].y - vertices[3].y;
		float angle = ATAN2(y, x) + g_PI_4;
		angle += (angle > g_PI) * -g_PI2;
		angle += (angle < -g_PI) * g_PI2;
		angles[index] = angle;
		weights[index] = g_PI-FABS(g_PI-FABS(angle-ref));
		++index;
	}

	// all two combos
	vector<array<float, 2> > helper(2);
	for(int i = 0; i < 3; ++i) {
		for(int j = i+1; j < 4; ++j, ++index) {
			helper[0][0] = separate_angles[i][0];
			helper[0][1] = separate_angles[i][1];
			helper[1][0] = separate_angles[j][0];
			helper[1][1] = separate_angles[j][1];
			float angle = fastMeanAngle(helper);
			angles[index] = angle;
			weights[index] = g_PI-FABS(g_PI-FABS(angle-ref));
		}
	}

	// all three combos
	// 1, 2, 3
	helper.resize(3);
	for(int i = 0; i < 2; ++i) {
		for(int j = i+1; j < 3; ++j) {
			for(int k = j+1; k < 4; ++k, ++index) {
				helper[0][0] = separate_angles[i][0];
				helper[0][1] = separate_angles[i][1];
				helper[1][0] = separate_angles[j][0];
				helper[1][1] = separate_angles[j][1];
				helper[2][0] = separate_angles[k][0];
				helper[2][1] = separate_angles[k][1];
				float angle = fastMeanAngle(helper);
				angles[index] = angle;
				weights[index] = g_PI-FABS(g_PI-FABS(angle-ref));
			}
		}
	}

	// all 4
	helper.resize(4);
	helper[0][0] = separate_angles[0][0];
	helper[0][1] = separate_angles[0][1];
	helper[1][0] = separate_angles[1][0];
	helper[1][1] = separate_angles[1][1];
	helper[2][0] = separate_angles[2][0];
	helper[2][1] = separate_angles[2][1];
	helper[3][0] = separate_angles[3][0];
	helper[3][1] = separate_angles[3][1];
	float angle = fastMeanAngle(helper);
	angles[index] = angle;
	weights[index] = g_PI-FABS(g_PI-FABS(angle-ref));

	vector<float> sorted_weights(weights);
	nth_element(sorted_weights.begin(), sorted_weights.begin()+num_considered_values, sorted_weights.end());

	float max = sorted_weights[num_considered_values];
	float rmax = 1.0f/max;
	vector<float> angles_subset;
	angles_subset.reserve(num_considered_values);
	vector<float> weights_subset;
	weights_subset.reserve(num_considered_values);
	for(size_t i = 0; i < weights.size(); ++i) {
		if(weights[i] < max) {
			angles_subset.push_back(angles[i]);
			weights_subset.push_back(1 - weights[i]*rmax);
		}
	}

	return(weightedMeanAngle(angles_subset, weights_subset));
}

cv::Point2f PPPEstimator::computePosition(const cv::Point2f &p, const float z) const {
	float x = ((z * Config::calibration_parameters.rfx_) * (p.x - Config::calibration_parameters.c.x));
	float y = ((z * Config::calibration_parameters.rfy_) * (p.y - Config::calibration_parameters.c.y));
	return(cv::Point2f(x, y));
}

void PPPEstimator::estimatePose(list<MarkerInfo> &markers) {
	// z-coordinate (real_perimeter*f_mean/screen_perimeter)
	// much more accurate measurements can be determined, if the real z value is already known from the map
	// and the position of the camera above ground. In that case we dont even care about the markers real size
	for(list<MarkerInfo>::iterator it = markers.begin(); it != markers.end(); ++it) {
		float z;
		if(Config::camera_parameters.z_offset >= 0 && Config::marker_parameters.base_T_m.count(it->id)) {	// we have the absolute camera pose (already)
			z = z_value.at(it->id);
		} else {
			array<float, 4> fx_weight = computeFXWeight(it->vertices);
			float weight_sum = fx_weight[0] + fx_weight[1] + fx_weight[2] + fx_weight[3];
			float f_wmean_sum = Config::calibration_parameters.f.x * weight_sum + Config::calibration_parameters.f.y * (4.0f - weight_sum);
			float target_length = it->length_mm * f_wmean_sum;
			z = target_length/computePerimeter(it->vertices);
		}

		// the relevant positioning information
		// 2d position of the marker center
		cv::Point2f marker_center(computeCenter(it->vertices));
		cv::Point2f position_xy(computePosition(marker_center, z));
		// angle around the z-axis (the other two are 0)
		float yaw = computeRotation(it->vertices, it->line_accuracy);
		//cout << "x,y,yaw=," << position_xy.x << "," << position_xy.y << "," << yaw << endl;

		// put the values into the pose: we only have x, y, z and yaw, the rest equals 0
		float translation3d[3] = { position_xy.x, position_xy.y, z };
		// update result
		it->w_T_m.setVectors(translation3d, yaw);

		// compute error value
		float area_status = computeQuadArea(it->vertices);
		float dx = FABS(marker_center.x - Config::calibration_parameters.c.x);
		float dy = FABS(marker_center.y - Config::calibration_parameters.c.y);
		float fx_weight = dx/(dx+dy);
		float f_wmean = fx_weight * Config::calibration_parameters.f.x + (1.0f-fx_weight) * Config::calibration_parameters.f.y;
		float target_length = (it->length_mm * f_wmean)/z;
		float area_target = target_length * target_length;
		float area_error = (area_status > area_target) ? area_target/area_status : area_status/area_target;

		array<float, 4> angles = computeAngles(it->vertices);
		float angle_error = 1;
		for(size_t i = 0; i < 4; ++i) {
			float error = FABS(angles[i]);
			error = error > g_PI_2 ? g_PI/(2*error) : (2.0f/g_PI)*error;
			angle_error *= error;
		}

		it->w_T_m.error = 1 - angle_error * area_error;

		// compute and set error (backprojection to get residual)
		/*float half_length = 0.5f * it->length_mm;
		Pose2d c_T_corner0(it->w_T_m * Pose2d(1, 0, 0, 0, -half_length, -half_length, 0, 0));
		Pose2d c_T_corner1(it->w_T_m * Pose2d(1, 0, 0, 0, +half_length, -half_length, 0, 0));
		Pose2d c_T_corner2(it->w_T_m * Pose2d(1, 0, 0, 0, +half_length, +half_length, 0, 0));
		Pose2d c_T_corner3(it->w_T_m * Pose2d(1, 0, 0, 0, -half_length, +half_length, 0, 0));

		float rz = 1.0f/z;
		float fx_by_z = f_.x * rz;
		float fy_by_z = f_.y * rz;
		cv::Point2f projected_vertices[4];
		projected_vertices[0].x = fx_by_z * c_T_corner0.translation3d[0] + c_.x;
		projected_vertices[0].y = fy_by_z * c_T_corner0.translation3d[1] + c_.y;
		projected_vertices[1].x = fx_by_z * c_T_corner1.translation3d[0] + c_.x;
		projected_vertices[1].y = fy_by_z * c_T_corner1.translation3d[1] + c_.y;
		projected_vertices[2].x = fx_by_z * c_T_corner2.translation3d[0] + c_.x;
		projected_vertices[2].y = fy_by_z * c_T_corner2.translation3d[1] + c_.y;
		projected_vertices[3].x = fx_by_z * c_T_corner3.translation3d[0] + c_.x;
		projected_vertices[3].y = fy_by_z * c_T_corner3.translation3d[1] + c_.y;

		float rms = 0;
		for(size_t j = 0; j < 4; ++j) {
			float dx = projected_vertices[j].x - it->vertices[j].x;
			float dy = projected_vertices[j].y - it->vertices[j].y;
			rms += dx*dx + dy*dy;
		}
		rms = SQRT(0.125f * rms);*/
	}
}
void PPPEstimator::estimatePose(list<MarkerInfo> &markers, const Pose2d &predict) const {
	// z-coordinate (real_perimeter*f_mean/screen_perimeter)
	// much more accurate measurements can be determined, if the real z value is already known from the map
	// and the position of the camera above ground. In that case we dont even care about the markers real size
	for(list<MarkerInfo>::iterator it = markers.begin(); it != markers.end(); ++it) {
		float z;
		if(Config::camera_parameters.z_offset >= 0 && Config::marker_parameters.base_T_m.count(it->id)) {	// we have the absolute camera pose (already)
			z = z_value.at(it->id);
		} else {
			array<float, 4> fx_weight = computeFXWeight(it->vertices);
			float weight_sum = fx_weight[0] + fx_weight[1] + fx_weight[2] + fx_weight[3];
			float f_wmean_sum = Config::calibration_parameters.f.x * weight_sum + Config::calibration_parameters.f.y * (4.0f - weight_sum);
			float target_length = it->length_mm * f_wmean_sum;
			z = target_length/computePerimeter(it->vertices);
		}

		//cerr << "begin: " << it->id << endl;
		Pose2d p_c_T_m(predict); // copy predict (which is w_T_c)
		p_c_T_m.invert(); // invert it to c_T_w
		p_c_T_m = p_c_T_m * Config::marker_parameters.base_T_m.at(it->id);	// apply markers w_T_m to obtain c_T_m
		// what we do need is the x,y coordinates, though, so compute them from the Pose in reverse
		float rz = 1.0f/z;
		float fx_by_z = Config::calibration_parameters.f.x * rz;
		float fy_by_z = Config::calibration_parameters.f.y * rz;
		cv::Point2f p_center(p_c_T_m.translation3d[0]*fx_by_z + Config::calibration_parameters.c.x, p_c_T_m.translation3d[1]*fy_by_z + Config::calibration_parameters.c.y);

		// the relevant positioning information
		// 2d position of the marker center
		cv::Point2f marker_center(computeCenter(it->vertices, p_center));
		cv::Point2f position_xy(computePosition(marker_center, z));
		// angle around the z-axis (the other two are 0)
		float yaw = computeRotation(it->vertices, p_c_T_m.rotation, it->line_accuracy);
		//cout << it->line_accuracy[0] << "," << it->line_accuracy[1] << "," << it->line_accuracy[2] << "," << it->line_accuracy[3] << endl;
		//cout << "x,y,yaw=," << position_xy.x << "," << position_xy.y << "," << yaw << "," << marker_center.x << "," << marker_center.y << endl;

		// put the values into the pose: we only have x, y, z and yaw, the rest equals 0
		float translation3d[3] = { position_xy.x, position_xy.y, z };
		// update result
		it->w_T_m.setVectors(translation3d, yaw);

		// compute error value
		float dx = FABS(marker_center.x - Config::calibration_parameters.c.x);
		float dy = FABS(marker_center.y - Config::calibration_parameters.c.y);
		float fx_weight = dx/(dx+dy);
		float f_wmean = fx_weight * Config::calibration_parameters.f.x + (1.0f-fx_weight) * Config::calibration_parameters.f.y;
		float target_length = it->length_mm * f_wmean * rz;
		float area_status = computeQuadArea(it->vertices);
		float area_target = target_length * target_length;
		float area_error = (area_status > area_target) ? area_target/area_status : area_status/area_target;

		array<float, 4> angles = computeAngles(it->vertices);
		float angle_error = 1;
		for(size_t i = 0; i < 4; ++i) {
			float error = FABS(angles[i]);
			error = error > g_PI_2 ? g_PI/(2*error) : (2.0f/g_PI)*error;
			angle_error *= error;
		}

		it->w_T_m.error = 1 - angle_error * area_error;
		it->w_T_m.dev = g_PI-FABS(g_PI-FABS(yaw-p_c_T_m.rotation));

		// compute and set error (backprojection to get residual)
		/*float half_length = 0.5f * it->length_mm;
		Pose2d c_T_corner0(it->w_T_m * Pose2d(1, 0, 0, 0, -half_length, -half_length, 0, 0));
		Pose2d c_T_corner1(it->w_T_m * Pose2d(1, 0, 0, 0, +half_length, -half_length, 0, 0));
		Pose2d c_T_corner2(it->w_T_m * Pose2d(1, 0, 0, 0, +half_length, +half_length, 0, 0));
		Pose2d c_T_corner3(it->w_T_m * Pose2d(1, 0, 0, 0, -half_length, +half_length, 0, 0));

		cv::Point2f projected_vertices[4];
		projected_vertices[0].x = fx_by_z * c_T_corner0.translation3d[0] + c_.x;
		projected_vertices[0].y = fy_by_z * c_T_corner0.translation3d[1] + c_.y;
		projected_vertices[1].x = fx_by_z * c_T_corner1.translation3d[0] + c_.x;
		projected_vertices[1].y = fy_by_z * c_T_corner1.translation3d[1] + c_.y;
		projected_vertices[2].x = fx_by_z * c_T_corner2.translation3d[0] + c_.x;
		projected_vertices[2].y = fy_by_z * c_T_corner2.translation3d[1] + c_.y;
		projected_vertices[3].x = fx_by_z * c_T_corner3.translation3d[0] + c_.x;
		projected_vertices[3].y = fy_by_z * c_T_corner3.translation3d[1] + c_.y;

		float rms = 0;
		cv::Point2f sum_offset = cv::Point2f(0,0);
		for(size_t j = 0; j < 4; ++j) {
			float dx = projected_vertices[j].x - it->vertices[j].x;
			float dy = projected_vertices[j].y - it->vertices[j].y;
			sum_offset.x += dx;
			sum_offset.y += dy;
			rms += dx*dx + dy*dy;
		}
		rms = SQRT(0.125f * rms);*/
	}
}

} /* namespace fml */
