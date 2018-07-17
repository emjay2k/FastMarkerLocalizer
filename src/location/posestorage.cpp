/*
 * posestorage.cpp
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#include "location/posestorage.h"

namespace fml {

PoseStorage::PoseStorage() {
	enabled_ = true;
}

PoseStorage::~PoseStorage() {
}

void PoseStorage::enable(bool value) {
	enabled_ = value;
}

} /* namespace fml */
