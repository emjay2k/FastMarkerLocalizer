/*
 * pose.cpp
 *
 *  Created on: 29.09.2014
 *      Author: jung
 */

#include "location/pose2d.h"

namespace fml {

Pose2d::Pose2d() {
	init(-1, -1, 0, 0, 0, 0, 0, 0);
	// memcopy all from eye
	memcpy(transformation, eye_4x4, 16*sizeof(float));
}

Pose2d::Pose2d(const int id, const int source_id, const clock_t timestamp, const float error, const float x, const float y, const float z, const float yaw) {
	init(id, source_id, timestamp, error, x, y, z, yaw);
	// memcopy all from eye
	//memcpy(transformation, eye_4x4, 16*sizeof(float));
	vectors_to_matrix();
}

Pose2d::Pose2d(const int id, const int source_id, const clock_t timestamp, const float error, const float (&trans)[3], const float rot) {
	init(id, source_id, timestamp, error, trans[0], trans[1], trans[2], rot);
	// memcopy all from eye
	//memcpy(transformation, eye_4x4, 16*sizeof(float));
	vectors_to_matrix();
}

Pose2d::Pose2d(const Pose2d& rhs) {
	/*init(rhs.id, rhs.source_id, rhs.timestamp, rhs.error, rhs.translation3d[0], rhs.translation3d[1], rhs.translation3d[2], rhs.rotation3d[2]);
	memcpy(transformation, rhs.transformation, 16*sizeof(float));
	dev = rhs.dev;*/
	memcpy(this, &rhs, sizeof(Pose2d));
}

Pose2d::~Pose2d() {
}

void Pose2d::init(const int id, const int source_id, const clock_t timestamp, const float error, const float x, const float y, const float z, const float yaw) {
	this->id = id;
	this->source_id = source_id;
	this->timestamp = timestamp;
	this->error = error;
	this->dev = -1;
	this->count = 0;
	this->translation3d[0] = x;
	this->translation3d[1] = y;
	this->translation3d[2] = z;
	this->rotation = yaw;
}

void Pose2d::matrix_to_vectors() {
	// only access the linear data for maximum speed (row1: 4+index, row2: 8+index, row3: 12+index)
	translation3d[0] = transformation[3];	// x-coordinate
	translation3d[1] = transformation[4+3];	// y-coordinate
	translation3d[2] = transformation[8+3];	// z-coordinate
	rotation = ATAN2(transformation[4+0], transformation[0]);	// alpha/yaw-angle
}

void Pose2d::vectors_to_matrix() {
	float sin_yaw = SIN(rotation);
	float cos_yaw = COS(rotation);

	// only access the linear data for maximum speed (row1: 4+index, row2: 8+index, row3: 12+index)
	transformation[0] = cos_yaw;
	transformation[1] = -sin_yaw;
	transformation[3] = translation3d[0];
	transformation[4+0] = sin_yaw;
	transformation[4+1] = cos_yaw;
	transformation[4+3] = translation3d[1];
	//transformation[8+2] = 1;
	transformation[8+3] = translation3d[2];
	//transformation[12+3] = 1;
}

void Pose2d::invert_matrix() {
	// only access the linear data for maximum speed (row1: 4+index, row2: 8+index, row3: 12+index)
	// invert rotation matrix R -> RT: (0,1) <-> (1,0)
	swap(transformation[1], transformation[4+0]);

	// buffer values
	float x = transformation[3];
	float y = transformation[4+3];
	float cos_yaw = transformation[0];		// cos yaw
	float sin_yaw = transformation[4+0];	// sin yaw

	// assign values from above
	transformation[3] = sin_yaw * y - cos_yaw * x;
	transformation[4+3] = -(sin_yaw * x + cos_yaw * y);
	transformation[8+3] = -transformation[8+3];
}

void Pose2d::invert() {
	// get inverse transformation matrix
	invert_matrix();
	// determine translation and rotation vectors from it
	matrix_to_vectors();
}

void Pose2d::setVectors(const float (&translation)[3], const float rotation) {
	translation3d[0] = translation[0];
	translation3d[1] = translation[1];
	translation3d[2] = translation[2];
	this->rotation = rotation;

	vectors_to_matrix();
}

void Pose2d::setMatrix(const cv::Mat &matrix) {
	memcpy(transformation, matrix.data, 16*sizeof(float));
	matrix_to_vectors();
}

void Pose2d::print() const {
	cout << id << "," << source_id << " (err=" << error << ", timestamp=" << timestamp << "):" << endl;
	cout << "(x, y, z, yaw) = (" << translation3d[0] << ", " << translation3d[1] << ", " << translation3d[2] << ", " << rotation*g_180_PI << ")" << endl;
	cout << "--" << endl;
}

void Pose2d::printMatrix() const {
	cout << "[";
	for(size_t i = 0; i < 4; ++i) {
		cout << "(";
		for(size_t j = 0; j < 4; ++j) {
			cout << transformation[j+4*i] << ",";
		}
		cout << ")" << endl;
	}
	cout << "]" << endl;
}

void Pose2d::printErr() const {
	cerr << id << "," << source_id << " (err=" << error << ", timestamp=" << timestamp << "):" << endl;
	cerr << "(x, y, z, yaw) = (" << translation3d[0] << ", " << translation3d[1] << ", " << translation3d[2] << ", " << rotation*g_180_PI << ")" << endl;
	cerr << "--" << endl;
}

bool Pose2d::operator<(Pose2d const &rhs) const {
	return(this->timestamp < rhs.timestamp);
}

Pose2d Pose2d::operator*(const Pose2d &rhs) const {
	const Pose2d * const actual = (rhs.id >= 0) ? &rhs : this;	// use righthandside data when useful
	Pose2d result(*actual);
	result.dev = max(rhs.dev, this->dev);
	
	// alternative matrix multiplication implementation (should be much faster as we have a special case)
	float x_left = this->transformation[0];
	float y_left = this->transformation[4+0];
	float x_right = rhs.transformation[0];
	float y_right = rhs.transformation[4+0];
	float A = x_left * x_right - y_left * y_right;
	float B = y_left * x_right + x_left * y_right;

	result.transformation[0] = A;
	result.transformation[1] = -B;
	result.transformation[3] = this->transformation[3] + rhs.transformation[3] * x_left - rhs.transformation[4+3] * y_left;
	result.transformation[4+0] = B;
	result.transformation[4+1] = A;
	result.transformation[4+3] = this->transformation[4+3] + rhs.transformation[4+3] * x_left + rhs.transformation[3] * y_left;
	//result.transformation[8+2] = 1;
	result.transformation[8+3] = this->transformation[8+3] + rhs.transformation[8+3];
	//result.transformation[12+3] = 1;

	result.matrix_to_vectors();
	return(result);
}

Pose2d& Pose2d::operator=(const Pose2d &rhs) {
	memcpy(this, &rhs, sizeof(Pose2d));
	return(*this);
}

} /* namespace fml */
