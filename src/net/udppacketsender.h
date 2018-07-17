/**
 * UDPPacketSender.h
 *
 *	Send data over a network channel to a recipient. We do not care if data is lost on the way, so UDP works out of the box.
 *
 *  Created on: 28.06.2012
 *      Author: jung
 */

#ifndef UDPPACKETSENDER_H_
#define UDPPACKETSENDER_H_

#include <array>
#include <iostream>
#include <boost/asio.hpp>
#include <generic/helper.h>

using namespace std;
using boost::asio::ip::udp;

namespace fml {

class UDPPacketSender {

	udp::socket *socket_;
	boost::asio::io_service *io_service_;
    udp::endpoint receiver_endpoint_;

	bool openConnection(const string &dst, const uint16_t port);
	bool closeConnection();

public:

	UDPPacketSender(const string dst="localhost", const uint16_t port=24191);
	virtual ~UDPPacketSender();

	bool isReady() const;
	int send(const vector<char> &payload) const;
};

}
#endif /* UDPPACKETSENDER_H_ */
