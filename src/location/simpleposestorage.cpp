/*
 * simpleposestorage.cpp
 *
 *  Created on: 07.02.2015
 *      Author: jung
 */

#include "location/simpleposestorage.h"

namespace fml {

SimplePoseStorage::SimplePoseStorage(string out, bool enabled) {
	out_file_ = out;
	enabled_ = enabled;
}

SimplePoseStorage::~SimplePoseStorage() {
	this->write();
}

void SimplePoseStorage::add(Pose2d &p) {
	if(enabled_) {
		pose_list_.push_back(p);
	}
}

void SimplePoseStorage::read() const {
}

void SimplePoseStorage::write() const {
	if(enabled_) {
		const char separator=',';
		const size_t num_buffer_lines = 100;
		const size_t num_chars_per_line = 200;

		ofstream csv;
		ostringstream line_buffer;
		line_buffer.str().reserve(num_buffer_lines*num_chars_per_line);
		csv.open(out_file_.c_str(), ofstream::out | ofstream::app);
		if(csv.is_open()) {
			line_buffer << "timestamp" << separator << "ID" << separator << "SRC_ID" << separator << "x" << separator << "y" << separator << "z" << separator << "yaw" << separator << "euclidean" << separator << "error" << separator << "count" << separator << separator << endl;
			size_t buffer_index = 1;
			float sign_bit[2] = { 1, -1 };
			for(list<Pose2d>::const_iterator it = pose_list_.begin(); it != pose_list_.end(); ++it, ++buffer_index) {
				float dx = it->translation3d[0];
				float dy = it->translation3d[1];
				float dz = it->translation3d[2];
				float sign = sign_bit[(dx < 0) ^ (dy < 0)];
				line_buffer << it->timestamp << separator << it->id << separator << it->source_id << separator << it->translation3d[0] << separator << it->translation3d[1] << separator << it->translation3d[2] << separator << it->rotation << separator << sign*SQRT(dx*dx + dy*dy + dz*dz) << separator << it->error << separator << it->count << separator << separator << endl;
				if(buffer_index == num_buffer_lines) {
					buffer_index = 0;
					csv << line_buffer.str();
					line_buffer.clear();
					line_buffer.str("");
					line_buffer.str().reserve(num_buffer_lines*num_chars_per_line);
				}
			}
			// write the remainder of the buffer
			csv << line_buffer.str();
			csv.close();
		} else {
			cerr << "Error: could not write to " << out_file_.c_str() << "." << endl;
		}
	}
}

} /* namespace fml */
