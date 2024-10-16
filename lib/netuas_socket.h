#ifndef _NETUAS_SOCKET_H_
#define _NETUAS_SOCKET_H_

#include <stdint.h>

#include "comm_interface.h"

#include "netuas_lib/socket.h"

//#define SOCKET_TIME_OUT_USEC 1000
#define SOCKET_TIME_OUT_USEC 0
#define SOCKET_TIME_OUT_SEC 0

#define CONNECTION_TIMEOUT 1.0 //s

class NetuasSocket : public CommunicationsInterface {
	public:
		NetuasSocket();
		~NetuasSocket();

		bool initialize(const char * hostname, const char * port, const char * type);

		bool open();
		bool close();

		int16_t read(uint8_t * buf, uint16_t buf_size);
		int16_t write(uint8_t * buf, uint16_t size);

		uint16_t getRxBytes() {return (uint16_t)sock_ptr->in();}
		uint16_t getTxBytes() {return (uint16_t)sock_ptr->out();}

		uint16_t getNumClients() { return (uint16_t)sock_ptr->getNumClients(); }

	private:
		Socket * sock_ptr;
		std::string host_str, port_str, type_str;
		Socket::SocketMode mode_type;
		TimeStamp last_connection;

		bool checkFDActive(Socket::SocketWait * status, bool is_tx);
};

#endif
