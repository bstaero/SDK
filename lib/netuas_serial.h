#ifndef _NETUAS_SERIAL_H_
#define _NETUAS_SERIAL_H_

#include <stdint.h>
#include <string>

#include "comm_interface.h"

#include "netuas/serial.h"

// how often to attempt a new connection
#define CONNECTION_TIMEOUT 1.0 // s

// values used for select timeout
//#define SERIAL_TIME_OUT_USEC 10000 
#define SERIAL_TIME_OUT_USEC 0
#define SERIAL_TIME_OUT_SEC  0

class NetuasSerial : public CommunicationsInterface {
	public:
		NetuasSerial();
		~NetuasSerial();

		bool initialize(const char * device, const char * baud, const char * = NULL);

		bool open();
		bool close();

		uint16_t read(uint8_t * buf, uint16_t buf_size);
		uint16_t write(uint8_t * buf, uint16_t size);

		uint16_t getRxBytes() {return (uint16_t)serial_ptr->in();}
		uint16_t getTxBytes() {return (uint16_t)serial_ptr->out();}

		bool setNonBlocking();

	private:
		SerialPort *serial_ptr;
		std::string device_str, baud_str;
		TimeStamp last_connection;
};

#endif
