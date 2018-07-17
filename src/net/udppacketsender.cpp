/**
 * UDPPacketSender.cpp
 *
 *	Send data over a network channel to a recipient. We do not care if data is lost on the way, so UDP works out of the box.
 *
 *  Created on: 28.06.2012
 *      Author: jung
 */

#include "net/udppacketsender.h"

namespace fml {

/**
 * Constructor
 */
UDPPacketSender::UDPPacketSender(const string dst, const uint16_t port) {
	bool connected = openConnection(dst, port);
	if(!connected) {
		cerr << "Could not connect to Socket" << endl;
	}
}

/**
 * Destructor
 *
 * Nothing to destroy.
 */
UDPPacketSender::~UDPPacketSender() {
	closeConnection();
}


/**
 * An ImageFilter is considered ready, when it's input image exists and has data
 *
 * @return true if we have a real image to process, false otherwise
 */
bool UDPPacketSender::isReady() const {
	return(socket_->is_open());
}

bool UDPPacketSender::openConnection(const string &dst, const uint16_t port) {
	try {
		// resolve recipient name
		io_service_ = new boost::asio::io_service();
		udp::resolver resolver(*io_service_);
		udp::resolver::query query(udp::v4(), dst, intToString((int)port));
		receiver_endpoint_ = *resolver.resolve(query);
		// open socket to reciever
		socket_ = new udp::socket(*io_service_);
		socket_->open(udp::v4());
		socket_->is_open();
		socket_->connect(receiver_endpoint_);
		return(true);
	} catch(exception &e) {
		cerr << e.what() << endl;
		return(false);
	}
}

bool UDPPacketSender::closeConnection() {
	try {
		if(socket_->is_open())
			socket_->close();
		delete socket_;
		delete io_service_;
		return(true);
	} catch(exception &e) {
		cerr << e.what() << endl;
		return(false);
	}
}

int UDPPacketSender::send(const vector<char> &payload) const {
	if(!payload.empty()) {
		try {
			array<char, UDP_BUFFER_SIZE> send_buf;
			copy(payload.begin(), payload.end(), send_buf.begin());
			//return(socket_->send_to(boost::asio::buffer(send_buf, input->size()*sizeof(char)), receiver_endpoint_));
			return(socket_->send(boost::asio::buffer(send_buf, payload.size()*sizeof(char))));
		} catch(exception &e) {
			cerr << e.what() << endl;
			return(-1);
		}
	} else return(0);
}

}
