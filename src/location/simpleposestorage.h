/*
 * simpleposestorage.h
 *
 *  Created on: 07.02.2015
 *      Author: jung
 */

#ifndef SRC_LOCATION_SIMPLEPOSESTORAGE_H_
#define SRC_LOCATION_SIMPLEPOSESTORAGE_H_

#include <list>
#include <fstream>
#include "location/posestorage.h"

using namespace std;

namespace fml {

class SimplePoseStorage : public PoseStorage {

protected:
	string out_file_;
	list<Pose2d> pose_list_;

	void read() const;
	void write() const;

public:
	SimplePoseStorage(string out, bool enabled=true);
	virtual ~SimplePoseStorage();

	void add(Pose2d &p);
};

} /* namespace fml */

#endif /* SRC_LOCATION_SIMPLEPOSESTORAGE_H_ */
