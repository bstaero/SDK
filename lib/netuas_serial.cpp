#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include "netuas_serial.h"
#include "debug.h"


NetuasSerial::NetuasSerial () : CommunicationsInterface(), last_connection(0,0) 
{
	pmesg(VERBOSE_ALLOC, "NetuasSerial::NetuasSerial()\n");

	serial_ptr = NULL;
	comm_type = CommunicationsInterface::SERIAL;
}

NetuasSerial::~NetuasSerial () { }

bool NetuasSerial::initialize(const char * device, const char * baud, const char * param3) {
	CommunicationsInterface::initialize(device,baud,param3);

	device ? device_str = device : device_str = "/dev/ttyUSB0";
	baud ? baud_str = baud : baud_str = "115200";

	if( serial_ptr != NULL ) {
		serial_ptr->close();
		delete serial_ptr;
		serial_ptr = NULL;
	}

	serial_ptr = new SerialPort(device_str.c_str(), atoi(baud_str.c_str()));
	if( serial_ptr == NULL ) {
		pmesg(VERBOSE_ERROR,"Did not crete serial port device\n");
		return false;
	}

	open();

	serial_ptr->setNonBlocking();

	return true;
}

// set non-blocking
bool NetuasSerial::setNonBlocking() {
	if( serial_ptr )
		return serial_ptr->setNonBlocking();

	return false;
}

bool NetuasSerial::setFlowControl() {
	if( serial_ptr )
		return serial_ptr->setModem();

	return false;
}

uint16_t NetuasSerial::read(uint8_t * buf, uint16_t buf_size) {
	int maxFD, val;
	fd_set readFDs;
	struct timeval timeout;

	/*<---Check Serial Conn--->*/
	if(serial_ptr == NULL || !open()) return 0;

	/*<---Check Inputs --->*/
	if( !buf || buf_size == 0 ) return 0;

	/*<------Select FDs------->*/
	FD_ZERO(&readFDs);
	maxFD = serial_ptr->setFD(readFDs);

	/*<------Select Timeout------->*/
	timeout.tv_sec = SERIAL_TIME_OUT_SEC; timeout.tv_usec = SERIAL_TIME_OUT_USEC;
	val = select(maxFD + 1, &readFDs, NULL, NULL, &timeout);

	if( val < 0 ) {
		if (errno != EINTR) pmesg(VERBOSE_ERROR,"select error\n");
	}
	else if(val != 0) {
	/*<----One FD is Active--->*/

		bool status = serial_ptr->checkFD(readFDs);

		/*<---Server Operations--->*/
		if(status) {
			int n = serial_ptr->read((char *)buf, buf_size);
			if( n > 0) { // data available
				return n;
			} 
			else {
				if(n == 0) {
					pmesg(VERBOSE_WARN,"read from serial returned zero bytes\n");
					initialize(device_str.c_str(), baud_str.c_str(), NULL);
				} else if( errno != EWOULDBLOCK && errno != EINTR && errno != EAGAIN) 
					pmesg(VERBOSE_ERROR,"bad read from client\n");
			}
		}
	}

	return 0;
}


uint16_t NetuasSerial::write(uint8_t * buf, uint16_t buf_size) {
	int maxFD, val, n=0;
	fd_set writeFDs;
	//SerialPort::SerialWait status;
	struct timeval timeout;

	/*<---Check Serial Conn--->*/
	if(serial_ptr == NULL) return 0;
	if(!open()) return 0;

	/*<------Select FDs------->*/
	FD_ZERO(&writeFDs);
	maxFD = serial_ptr->setFD(writeFDs);

	/*<------Select Timeout------->*/
	timeout.tv_sec = SERIAL_TIME_OUT_SEC; timeout.tv_usec = SERIAL_TIME_OUT_USEC;
	val = select(maxFD + 1, NULL, &writeFDs, NULL, &timeout);

	if( val < 0 ) {
		if (errno != EINTR)
			pmesg(VERBOSE_ERROR,"select error\n");
	}
	else if(val != 0) {
	/*<----One FD is Active--->*/

		//bool status = serial_ptr->checkFD(writeFDs);

		n = serial_ptr->write((char *)buf, buf_size);

		// see if we wrote all data
		if( n != buf_size ) {
			if( n == 0)
				pmesg(VERBOSE_WARN,"write to client returned zero bytes\n");
			else if( errno != EWOULDBLOCK || errno != EINTR) 
				pmesg(VERBOSE_ERROR,"bad write to client\n");
			else 
				pmesg(VERBOSE_WARN,"did not write all bytes to serial\n");
		}
	}

	return n;
}


bool NetuasSerial::close() 
{
	if(serial_ptr != NULL) {
		serial_ptr->close();
		delete serial_ptr;
		serial_ptr = NULL;
	}
	connected = false;
	return true;
}

bool NetuasSerial::open() {
	if(serial_ptr == NULL) return false;

	if(serial_ptr->getStatus() != SerialPort::Connected) {
		TimeStamp ts;
		ts.stamp();

		if(ts.diff(last_connection) < CONNECTION_TIMEOUT) 
			return false;

		last_connection = ts;

		if ( !serial_ptr->open() ) {
			pmesg(VERBOSE_ERROR,"Could not open serial: dev=%s@%u error=%s\n", serial_ptr->getDev(), serial_ptr->getBaud(), strerror(errno));
			connected = false;
			return false;
		} 
	}

	connected = true;

	return true;
}
