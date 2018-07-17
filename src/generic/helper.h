/*
 * helper.h
 *
 *  Created on: 12.09.2014
 *      Author: jung
 */

#ifndef HELPER_H_
#define HELPER_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <ctime>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include "location/pose2d.h"

using namespace std;

namespace fml {

/**
 * Return timestamp
 *
 * @return
 */
inline clock_t currentTimeOfDayMilliseconds() {
	boost::posix_time::ptime current_date_microseconds = boost::posix_time::microsec_clock::local_time();
	return((clock_t)current_date_microseconds.time_of_day().total_milliseconds()); // get local timestamp
}

inline void sleepMilliSeconds(const size_t ms) {
	boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

inline string intToString(const int i) {
	ostringstream temp;
	temp << i;
	return(temp.str());
}

inline string floatToString(const float f) {
	ostringstream temp;
	temp << f;
	return(temp.str());
}

/**
 * Returns polar coordinates (x,y) of an angle
 *
 * @param angle
 * @return
 */
inline array<float, 2> toPolar(const float &angle) {
	array<float, 2> polar;
	polar[0] = COS(angle);
	polar[1] = SIN(angle);
	return(polar);
}

/**
 * determine mean angle of a set of angles
 *
 * @param angles
 * @return
 */
inline float meanAngle(const vector<float> &angles) {
	// determine mean
	float x = 0, y = 0;
	for(size_t i = 0; i < angles.size(); ++i) {
		x += COS(angles[i]);
		y += SIN(angles[i]);
	}

	// compute resulting angle value
	return(ATAN2(y, x));
}

/**
 * determine mean angle of a set of angles
 *
 * @param angles
 * @return
 */
inline float fastMeanAngle(const vector<array<float, 2> > &xy) {
	// determine mean
	float x = 0, y = 0;
	for(size_t i = 0; i < xy.size(); ++i) {
		x += xy[i][0];
		y += xy[i][1];
	}

	// compute resulting angle value
	return(ATAN2(y, x));
}

inline void computeStdev(const list<Pose2d> &poses, const Pose2d &mean, float (&stdev)[4]) {
	// initialize to zero
	stdev[0] = 0, stdev[1] = 0, stdev[2] = 0, stdev[3] = 0;

	// handle x and y separately for the angles
	for(list<Pose2d>::const_iterator it = poses.begin(); it != poses.end(); ++it) {
		for(size_t i = 0; i < 3; ++i) {
			// translation component: (squared distance)
			float d = it->translation3d[i] - mean.translation3d[i];
			stdev[i] += d*d;
		}
		// rotation component
		float d = g_PI-FABS(g_PI-FABS(it->rotation - mean.rotation));
		stdev[3] += d*d;
	}

	float factor = 1.0f/(poses.size()-1);
	for(size_t i = 0; i < 4; ++i) {
		stdev[i] = SQRT(factor * stdev[i]);
	}
}

/**
 * determine mean angle of a set of angles
 *
 * @param angles
 * @return
 */
inline float weightedMeanAngle(const vector<float> &angles, vector<float> &weights) {
	float weight_sum = weights[0];
	// norm the weights
	for(size_t i = 1; i < weights.size(); ++i) {
		weight_sum += weights[i];
	}
	weight_sum = 1.0f/weight_sum;

	// determine mean
	float x = 0, y = 0;
	for(size_t i = 0; i < angles.size(); ++i) {
		weights[i] *= weight_sum;
		x += weights[i] * COS(angles[i]);
		y += weights[i] * SIN(angles[i]);
	}

	// compute resulting angle value
	return(ATAN2(y, x));
}

/**
 * Determine mean pose over a list of poses
 *
 * @param poses
 * @return
 */
inline Pose2d meanPose(const list<Pose2d> &poses, float (&stdev)[4], const bool compute_stdev=false) {
	int num_poses = poses.size();
	if(num_poses == 1) {
		return(poses.back());
	} else if(num_poses > 1) {
		vector<float> angles(poses.size());	// store angles for mean computation
		int id = poses.begin()->id;
		int source_id = poses.begin()->source_id;
		clock_t mean_timestamp = poses.begin()->timestamp;
		float mean_error = 0;
		float mean_translation[3] = { 0, 0, 0 };
		float mean_rotation = 0;

		int index = 0;
		for(list<Pose2d>::const_iterator it = poses.begin(); it != poses.end(); ++it, ++index) {
			mean_error += it->error;

			mean_translation[0] += it->translation3d[0];
			mean_translation[1] += it->translation3d[1];
			mean_translation[2] += it->translation3d[2];
			angles[index] = it->rotation;
		}

		float rnum_poses = 1.0f/num_poses;
		mean_error *= rnum_poses;
		mean_translation[0] *= rnum_poses;
		mean_translation[1] *= rnum_poses;
		mean_translation[2] *= rnum_poses;
		mean_rotation = meanAngle(angles);

		Pose2d mean(id, source_id, mean_timestamp, mean_error, mean_translation[0], mean_translation[1], mean_translation[2], mean_rotation);

		if(compute_stdev) {
			computeStdev(poses, mean, stdev);
		}

		return(mean);
	} else {
		return(Pose2d());
	}
}

/**
 * Determine weighted mean sum over a list of Poses, it will automatically norm the weights to [0,1] with a total sum of 1
 *
 * @param poses
 * @param weight_from_error: if true: use error value as weight, otherwise use area
 * @return
 */
inline Pose2d weightedMeanPose(const list<Pose2d> &poses, vector<float> &weights) {
	int num_poses = poses.size();
	if(num_poses == 1) {
		return(poses.back());
	} else if(num_poses > 1) {
		vector<float> angles(num_poses);	// store angles for mean computation
		int id = poses.begin()->id;
		int source_id = poses.begin()->source_id;
		clock_t mean_timestamp = poses.begin()->timestamp;
		float mean_error = 0;
		float mean_translation[3] = { 0, 0, 0 };

		size_t weight_index = 0;
		for(list<Pose2d>::const_iterator it = poses.begin(); it != poses.end(); ++it, ++weight_index) {
			angles[weight_index] = it->rotation;
		}
		// determine mean rotation (will also norm the weights for us)
		float mean_rotation = weightedMeanAngle(angles, weights);
		// compute mean translation
		weight_index = 0;
		for(list<Pose2d>::const_iterator it = poses.begin(); it != poses.end(); ++it, ++weight_index) {
			float weight = weights[weight_index];
			mean_error += weight * it->error;
			mean_translation[0] += weight * it->translation3d[0];
			mean_translation[1] += weight * it->translation3d[1];
			mean_translation[2] += weight * it->translation3d[2];
		}

		return(Pose2d(id, source_id, mean_timestamp, mean_error, mean_translation[0], mean_translation[1], mean_translation[2], mean_rotation));
	} else {
		return(Pose2d());
	}
}

#ifdef DEBUG
# define DEBUG_PRINT(x) printf x
#else
# define DEBUG_PRINT(x) do {} while (0)
#endif

/**
 * run some function for debug purposes
 *
 * @param function
 */
inline void DEBUG_RUN(void (*fp)()) {
#ifdef DEBUG
	fp();
#endif
}

} /* namespace fml */

#endif /* HELPER_H_ */
