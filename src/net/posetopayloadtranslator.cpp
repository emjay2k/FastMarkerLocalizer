/**
 * PoseToPayloadTranslator.cpp
 *
 * Export localization information via a datagram
 *
 *  Created on: 28.06.2012
 *      Author: jung
 */

#include "net/posetopayloadtranslator.h"

namespace fml {

/**
 * Constructor
 *
 * No parameters for the filter function to initialize.
 */
PoseToPayloadTranslator::PoseToPayloadTranslator(const bool smooth) : v_filter(0.8), a_filter(0.8) {
	smooth_ = smooth;
}

/**
 * Destructor
 *
 * Nothing to destroy.
 */
PoseToPayloadTranslator::~PoseToPayloadTranslator() {
}

/**
 * The payload of the packet always contains the following information in the given order:
 *
 * 1. packet type as char: 1 Byte ("0" for localization data)
 * 2. vehicle id as char: 1 Byte (allows for up to 256 vehicles)
 * 3. timestamp as int: 4 Bytes (milliseconds since beginning of this day)
 * 4. translation coordinate x as int: 4 Bytes (in cm)
 * 5. translation coordinate y as int: 4 Bytes (in cm)
 * 6. translation coordinate z as int: 4 Bytes (in cm)
 * 7. rotation angle roll as short: 2 Bytes (in 1/100 degrees)
 * 8. rotation angle pitch as short: 2 Bytes (in 1/100 degrees)
 * 9. rotation angle yaw as short: 2 Bytes (in 1/100 degrees)
 * 10. translation velocity as short: 2 Bytes (in 1/1000 m/s)
 * 11. translation acceleration as short: 2 Bytes (in 1/1000 m/s^2)
 * 12. rotation velocity as short: 2 Bytes (in 1/100 degrees/s)
 * 13. rotation acceleration as short: 2 Bytes (in 1/100 degrees/s^2)
 *
 *
 * => total 24 Bytes
 *
 * in case the vehicle does not support a sensor it shall simply set the not available information to zero. it is up to the recipient to make sense of it.
 */
void PoseToPayloadTranslator::translate(const Pose2d &pose, vector<char> &payload, float v_trans, float a_trans, const float v_rot, const float a_rot) {
	payload.resize(32, 0);

	int index = 0;
	payload[index++] = signedChar((unsigned char)0);	// packet type = 0
	payload[index++] = signedChar(Config::tracker_parameters.network_id);			// vehicle id

	array<char, 4> temp = splitWordToBytes((int)pose.timestamp);		// 4 timestamp bytes
	payload[index++] = temp[0];
	payload[index++] = temp[1];
	payload[index++] = temp[2];
	payload[index++] = temp[3];

	for(size_t i = 0; i < 3; ++i) {	// x, y, z
		temp = splitWordToBytes((int)ROUND(pose.translation3d[i]));	// each value 4 bytes
		payload[index++] = temp[0];
		payload[index++] = temp[1];
		payload[index++] = temp[2];
		payload[index++] = temp[3];
	}

	// roll and pitch area always zero in our scenario, no matter what the sensors says
	float rotation[3] = { 0, 0, pose.rotation };
	for(size_t i = 0; i < 3; ++i) {	// roll, pitch, yaw
		array<char, 2> temp = splitHalfwordToBytes((short)ROUND((100.0f*g_180_PI)*rotation[i]));
		payload[index++] = temp[0];
		payload[index++] = temp[1];
	}

	// translation velocity
	if(smooth_) {
		v_filter.smooth(v_trans);
		a_filter.smooth(a_trans);
	}
	array<char, 2> temp_speed = splitHalfwordToBytes((short)ROUND(v_trans*1000.0f));
	payload[index++] = temp_speed[0];
	payload[index++] = temp_speed[1];

	// translation acceleration
	temp_speed = splitHalfwordToBytes((short)ROUND(a_trans*1000.0f));
	payload[index++] = temp_speed[0];
	payload[index++] = temp_speed[1];

	// rotation velocity
	temp_speed = splitHalfwordToBytes((short)ROUND((100.0f*g_180_PI)*v_rot));
	payload[index++] = temp_speed[0];
	payload[index++] = temp_speed[1];

	// rotation acceleration
	temp_speed = splitHalfwordToBytes((short)ROUND((100.0f*g_180_PI)*a_rot));
	payload[index++] = temp_speed[0];
	payload[index++] = temp_speed[1];
}

array<char, 4> PoseToPayloadTranslator::splitWordToBytes(const int value) const {
	array<char, 4> retval;
	retval[0] = (char)(value >> 24);
	retval[1] = (char)((0x00ff0000 & value) >> 16);
	retval[2] = (char)((0x0000ff00 & value) >> 8);
	retval[3] = (char)(0x000000ff & value);
	return(retval);
}

array<char, 2> PoseToPayloadTranslator::splitHalfwordToBytes(const short value) const {
	array<char, 2> retval;
	retval[0] = (char)(value >> 8);
	retval[1] = (char)(0xff & value);
	return(retval);
}

char PoseToPayloadTranslator::signedChar(const unsigned char unsigned_char) const {
	short retval = (short)unsigned_char;
	return((char)(retval-128));
}

short PoseToPayloadTranslator::signedShort(const unsigned short unsigned_short) const {
	int retval = (int)unsigned_short;
	return((short)(retval-32768));
}

int PoseToPayloadTranslator::signedInt(const unsigned int unsigned_int) const {
	long long retval = (long long)unsigned_int;
	return((int)(retval-2e31));
}

}
