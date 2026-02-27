/*=+--+=#=+--         SwiftCore Flight Management Software        --+=#=+--+=#*\
|               Copyright (C) 2015 Black Swift Technologies LLC.               |
|                             All Rights Reserved.                             |

     NOTICE:  All information contained herein is, and remains the property
     of Black Swift Technologies.

     The intellectual and technical concepts contained herein are
     proprietary to Black Swift Technologies LLC and may be covered by U.S.
     and foreign patents, patents in process, and are protected by trade
     secret or copyright law.

     Dissemination of this information or reproduction of this material is
     strictly forbidden unless prior written permission is obtained from
     Black Swift Technologies LLC.
|                                                                              |
|                                                                              |
\*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*/

#include <stdio.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#if !defined __APPLE__ && !defined __OPENWRT__
  #include <error.h>
#endif
#include <errno.h>
#ifdef __APPLE__
  #include <IOKit/serial/ioss.h>
#endif

#ifdef __APPLE__
  #ifndef B460800
    #define B460800   460800
  #endif
  #ifndef B921600
    #define B921600   921600
  #endif
#endif

#include "bst_serial.h"
#include "debug.h"

/* ------------------------------------------------------------------ */

BSTSerial::BSTSerial() : BSTInterface() {
	pmesg(VERBOSE_ALLOC, "BSTSerial::BSTSerial()\n");

	memset(dev, 0, sizeof(dev));
	baud_rate       = 0;
	status          = STATUS_OK;
	blocking        = false;
	bytes_in_total  = 0;
	bytes_out_total = 0;
}

BSTSerial::~BSTSerial() {
	close();
}

/* ---- Standard CommunicationsInterface ---- */

bool BSTSerial::initialize(const char * device, const char * baud, const char * param3) {
	strncpy(this->dev, device, BST_MAX_DEV_LENGTH);
	this->dev[BST_MAX_DEV_LENGTH - 1] = '\0';
	this->baud_rate = atoi(baud);

	return open();
}

bool BSTSerial::open(void) {
	/* Close any existing connection first */
	close();

	fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK, 0777);
	if (fd < 0 || !isatty(fd)) {
		pmesg(VERBOSE_ERROR, "ERROR: opening serial port %s: %s\n", dev, strerror(errno));
		close();
		return false;
	}

	tcflush(fd, TCIOFLUSH);

	if (!applyTermiosConfig()) {
		close();
		return false;
	}

	blocking  = false;
	status    = STATUS_CONNECTED;
	connected = true;
	return true;
}

/* ---- Overridden read/write with device-disconnect handling ---- */

int16_t BSTSerial::read(uint8_t * buf, uint16_t buf_size) {
	if (!checkConnected()) return 0;
	if (!buf || buf_size == 0) return 0;

	int n = ::read(this->fd, buf, buf_size);
	if (n < 0) {
		if (errno == EIO || errno == ENODEV || errno == ENXIO) {
			/* Device disconnected */
			pmesg(VERBOSE_WARN, "serial device disconnected, will attempt reconnection\n");
			connected = false;
			status = STATUS_ERROR;
		} else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
			pmesg(VERBOSE_ERROR, "BSTSerial::read error: %s\n", strerror(errno));
		}
		return 0;
	}
	if (n > 0) {
		rx_bytes += n;
		bytes_in_total += n;
	}
	return (int16_t)n;
}

int16_t BSTSerial::write(uint8_t * buf, uint16_t size) {
	if (!checkConnected()) return 0;

	int n = ::write(this->fd, buf, size);
	if (n < 0) {
		if (errno == EIO || errno == ENODEV || errno == ENXIO) {
			/* Device disconnected */
			pmesg(VERBOSE_WARN, "serial device disconnected on write, will attempt reconnection\n");
			connected = false;
			status = STATUS_ERROR;
		} else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
			pmesg(VERBOSE_ERROR, "BSTSerial::write error: %s\n", strerror(errno));
		}
		return 0;
	}
	tx_bytes += n;
	bytes_out_total += n;
	return (int16_t)n;
}

bool BSTSerial::close() {
	status = STATUS_OK;

	if (this->fd >= 0) {
		::close(this->fd);
		this->fd = -1;
	}

	connected = false;
	return true;
}

/* ---- Serial configuration ---- */

bool BSTSerial::setBaud(int baud) {
	baud_rate = baud;

	if (this->fd < 0)
		return false;

	return applyTermiosConfig();
}

bool BSTSerial::setEvenParity() {
	if (this->fd < 0) return false;

	struct termios config;
	if (tcgetattr(fd, &config) != 0) return false;

	config.c_cflag |= PARENB;       /* Enable parity */
	config.c_cflag &= ~PARODD;      /* Even parity */
	config.c_cflag &= ~CSTOPB;      /* 1 stop bit */
	config.c_iflag |= INPCK;        /* Enable input parity checking */

	return tcsetattr(fd, TCSANOW, &config) == 0;
}

bool BSTSerial::setFlowControl() {
	if (this->fd < 0) return false;

	struct termios config;
	if (tcgetattr(fd, &config) != 0) return false;

	config.c_cflag &= ~CLOCAL;      /* Listen to modem status lines */
	config.c_cflag |= CRTSCTS;      /* Enable hardware flow control */

	return tcsetattr(fd, TCSANOW, &config) == 0;
}

bool BSTSerial::setBlocking() {
	if (this->fd < 0) return false;

	int flags = fcntl(fd, F_GETFL, 0);
	if (flags < 0) return false;

	if (fcntl(fd, F_SETFL, flags & ~O_NONBLOCK) < 0)
		return false;

	blocking = true;
	return true;
}

bool BSTSerial::setNonBlocking() {
	if (this->fd < 0) return false;

	int flags = fcntl(fd, F_GETFL, 0);
	if (flags < 0) return false;

	if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0)
		return false;

	blocking = false;
	return true;
}

bool BSTSerial::isBlocking() const {
	return blocking;
}

/* ---- Select/fd_set integration ---- */

int BSTSerial::setFD(fd_set & set) {
	if (this->fd < 0 || this->fd >= FD_SETSIZE) return -1;

	FD_SET(this->fd, &set);
	return this->fd;
}

bool BSTSerial::checkFD(fd_set & set) {
	if (this->fd < 0 || this->fd >= FD_SETSIZE || status == STATUS_ERROR)
		return false;

	return FD_ISSET(this->fd, &set) != 0;
}

/* ---- Status queries ---- */

BSTSerial::SerialStatus BSTSerial::getStatus() const { return status; }
const char * BSTSerial::getDev() const { return dev; }
int BSTSerial::getBaud() const { return baud_rate; }
long BSTSerial::bytesIn() const { return bytes_in_total; }
long BSTSerial::bytesOut() const { return bytes_out_total; }

int BSTSerial::getNumClients() const {
	return (status == STATUS_CONNECTED) ? 1 : 0;
}

void BSTSerial::flushIO() {
	if (this->fd >= 0)
		tcflush(fd, TCIOFLUSH);
}

int BSTSerial::getFD() const {
	return this->fd;
}

/* ---- Internal helpers ---- */

speed_t BSTSerial::baudToSpeed(int baud) {
	switch (baud) {
		case    300: return B300;
		case   1200: return B1200;
		case   2400: return B2400;
		case   4800: return B4800;
		case   9600: return B9600;
		case  19200: return B19200;
		case  38400: return B38400;
		case  57600: return B57600;
		case 115200: return B115200;
		case 230400: return B230400;
		case 460800: return B460800;
		case 921600: return B921600;
#if !defined __APPLE__
		case 1000000: return B1000000;
		case 1152000: return B1152000;
		case 1500000: return B1500000;
		case 2000000: return B2000000;
		case 2500000: return B2500000;
		case 3000000: return B3000000;
		case 3500000: return B3500000;
		case 4000000: return B4000000;
#endif
		default:
			pmesg(VERBOSE_ERROR, "ERROR: unsupported baud rate %d\n", baud);
			return B9600;
	}
}

bool BSTSerial::applyTermiosConfig() {
	struct termios config;
	if (tcgetattr(fd, &config) != 0) {
		pmesg(VERBOSE_ERROR, "ERROR: unable to poll port settings\n");
		return false;
	}

	/* 8N1, raw mode, no flow control */
	config.c_cflag |= (CS8 & CSIZE) | CREAD | HUPCL | CLOCAL;
	config.c_cflag &= ~(PARENB | CRTSCTS | CSTOPB);
	config.c_iflag |= IGNPAR | IGNBRK;
	config.c_iflag &= ~(IXON | IXOFF | BRKINT | ICRNL | INPCK | ISTRIP);
	config.c_oflag &= ~OPOST;
	config.c_lflag &= ~(ICANON | ECHO | IEXTEN | ISIG);

	speed_t speed = baudToSpeed(baud_rate);

#ifdef __APPLE__
	if (ioctl(fd, IOSSIOSPEED, &speed) == -1) {
		pmesg(VERBOSE_ERROR, "ERROR: unable to set baud rate settings\n");
		return false;
	}
#else
	if (cfsetispeed(&config, speed) != 0) {
		pmesg(VERBOSE_ERROR, "ERROR: problem setting input baud rate\n");
		return false;
	}

	if (cfsetospeed(&config, speed) != 0) {
		pmesg(VERBOSE_ERROR, "ERROR: problem setting output baud rate\n");
		return false;
	}
#endif

	if (tcsetattr(fd, TCSANOW, &config) != 0) {
		pmesg(VERBOSE_ERROR, "ERROR: unable to set serial port settings\n");
		return false;
	}

	return true;
}
