/*
 * evaluationposestorage.cpp
 *
 *  Created on: 07.02.2015
 *      Author: jung
 */

#include "location/evaluationposestorage.h"

namespace fml {

EvaluationPoseStorage::EvaluationPoseStorage(const string out, const bool enabled) : SimplePoseStorage(out, enabled) {
}

EvaluationPoseStorage::~EvaluationPoseStorage() {
	this->write();
}

void EvaluationPoseStorage::write() const {
	if(enabled_) {
		const char separator=',';
		ofstream csv;
		csv.open(out_file_.c_str(), ofstream::out | ofstream::app);
		if(csv.is_open()) {
			// along with the raw data write some status information
			size_t num_values = pose_list_.size();
			if(num_values > 0) {
				vector<float> trans(num_values);
				vector<float> rot(num_values);
				int index = 0;
				double trans_mean = 0, trans_stdev = 0;
				double rot_mean = 0, rot_stdev = 0;
				for(list<Pose2d>::const_iterator it = pose_list_.begin(); it != pose_list_.end(); ++it) {
					float dx = it->translation3d[0];
					float dy = it->translation3d[1];
					float dz = it->translation3d[2];
					float euclidean = SQRT(dx*dx + dy*dy + dz*dz);
					trans[index] = euclidean;
					rot[index++] = FABS(it->rotation);
					trans_mean += euclidean;
					rot_mean += FABS(it->rotation);
				}

				// mean
				float rnum_values = 1.0f/num_values; 	// = 1.0f/num_values
				trans_mean *= rnum_values;
				rot_mean *= rnum_values;
				// standard deviation
				for(size_t i = 0; i < trans.size(); ++i) {
					float d = trans_mean - trans[i];
					trans_stdev += d*d;
					d = rot_mean - rot[i];
					rot_stdev += d*d;
				}
				rnum_values = 1.0f/(num_values-1);	// = 1.0f/(num_values-1)
				trans_stdev = sqrt(trans_stdev*rnum_values);
				rot_stdev = g_180_PI * sqrt(rot_stdev*rnum_values);
				rot_mean *= g_180_PI;

				// create some more status information
				// median
				float trans_median, rot_median;
				index = num_values >> 1;
				// determine value at num_values/2
				vector<float>::iterator it_trans_split = trans.begin()+index;
				vector<float>::iterator it_rot_split = rot.begin()+index;
				nth_element(trans.begin(), it_trans_split, trans.end());
				nth_element(rot.begin(), it_rot_split, rot.end());
				if(!MOD2(num_values)) {
					// also determine the value to the left for the correct median
					nth_element(trans.begin(), it_trans_split-1, it_trans_split);
					nth_element(rot.begin(), it_rot_split-1, it_rot_split);
					trans_median = 0.5f * (trans[index] + trans[index-1]);
					rot_median = (0.5f*g_180_PI) * (rot[index] + rot[index-1]);
				} else {
					trans_median = trans[index];
					rot_median = g_180_PI * rot[index];
				}

				ostringstream buffer;
				buffer.str().reserve(1000);
				buffer << "" << separator << "median" << separator << "mean" << separator << "stdev" << separator << "num" << separator << separator << endl;
				buffer << "translation" << separator << trans_median << separator << trans_mean << separator << trans_stdev << separator << trans.size() << separator << separator << endl;
				buffer << "rotation" << separator << rot_median << separator << rot_mean << separator << rot_stdev << separator << rot.size() << separator << separator << endl;
				buffer << endl;
				buffer << "sigma" << separator << "1" << separator << "2" << separator << "3" << separator << "4" << separator << "5" << separator << "6" << separator << separator << endl;

				// 1-6 sigma values
				float trans_sigma_values[6], rot_sigma_values[6];
				vector<float>::iterator next;
				for(size_t i = 0; i < 6; ++i) {
					size_t sigma_index = ceil(sigma_[i] * num_values) - 1;
					next = trans.begin() + sigma_index;
					nth_element(it_trans_split, next, trans.end());
					it_trans_split = next + 1;
					next = rot.begin() + sigma_index;
					nth_element(it_rot_split, next, rot.end());
					trans_sigma_values[i] = trans[sigma_index];
					rot_sigma_values[i] = g_180_PI * rot[sigma_index];
					it_rot_split = next + 1;
				}
				buffer << "translation" << separator << trans_sigma_values[0] << separator << trans_sigma_values[1] << separator << trans_sigma_values[2] << separator << trans_sigma_values[3] << separator << trans_sigma_values[4] << separator << trans_sigma_values[5] << separator << separator << endl;
				buffer << "rotation" << separator << rot_sigma_values[0] << separator << rot_sigma_values[1] << separator << rot_sigma_values[2] << separator << rot_sigma_values[3] << separator << rot_sigma_values[4] << separator << rot_sigma_values[5] << separator << separator << endl;
				buffer << endl << endl;
				csv << buffer.str();
			}
			csv.close();
		} else {
			cerr << "Error: could not write to " << out_file_.c_str() << "." << endl;
		}
	}
}

} /* namespace fml */
