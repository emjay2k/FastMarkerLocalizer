/*
 * pose.h
 *
 *  Created on: 29.09.2014
 *      Author: jung
 */

#ifndef POSE2D_H_
#define POSE2D_H_

#include <ctime>
#include <cmath>
#include <map>
#include <vector>
#include <iostream>
#include "generic/definitions.h"

using namespace std;

namespace fml {

static const float eye_4x4[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }; // 4x4 identity matrix flatend row major

class Pose2d {
	inline void vectors_to_matrix();
	inline void matrix_to_vectors();
	inline void invert_matrix();
	inline void init(const int id, const int source_id, const clock_t timestamp, const float error, const float x, const float y, const float z, const float yaw);

public:
	int id;
	int source_id;	// sensor id
	int count;
	clock_t timestamp;
	float error;
	float dev;
	float translation3d[3]; // x, y, z in mm
	float rotation;	// yaw from [-pi, pi]
	float transformation[16];

	Pose2d();
	Pose2d(const int id, const int source_id, const clock_t timestamp, const float error, const float x, const float y, const float z, const float yaw);
	Pose2d(const int id, const int source_id, const clock_t timestamp, const float error, const float (&trans)[3], const float rot);
	Pose2d(const Pose2d& rhs);	// copy constructor
	virtual ~Pose2d();

	void setVectors(const float (&translation)[3], const float rotation);
	void setMatrix(const cv::Mat &matrix);

	void invert();
	void print() const;
	void printMatrix() const;
	void printErr() const;

	bool operator<(const Pose2d &rhs) const;
	Pose2d operator*(const Pose2d &rhs) const;
	Pose2d& operator=(const Pose2d &rhs);
};

} /* namespace fml */

#endif /* POSE2D_H_ */
