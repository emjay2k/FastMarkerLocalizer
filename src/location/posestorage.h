/*
 * posestorage.h
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#ifndef SRC_POSESTORAGE_H_
#define SRC_POSESTORAGE_H_

#include <string>
#include "location/pose2d.h"

using namespace std;

namespace fml {

class PoseStorage {

protected:
	bool enabled_;
	virtual void read() const = 0;
	virtual void write() const = 0;

public:
	PoseStorage();
	virtual ~PoseStorage();

	virtual void add(Pose2d &p) = 0;
	void enable(bool value);
};

} /* namespace fml */

#endif /* SRC_POSESTORAGE_H_ */
