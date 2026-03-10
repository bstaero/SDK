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
#ifndef BST_SERIAL_H
#define BST_SERIAL_H

#include <sys/select.h>
#include <termios.h>

#include "bst_interface.h"

#define BST_MAX_DEV_LENGTH 64

class BSTSerial : public BSTInterface {
	public:
		enum SerialStatus { STATUS_ERROR = -1, STATUS_OK = 0, STATUS_CONNECTED = 1 };
		enum SerialWait   { SWAIT_INT = -3, SWAIT_TIMEOUT = -2, SWAIT_ERROR = -1,
		                    SWAIT_DATA = 0 };

		BSTSerial();
		virtual ~BSTSerial();

		/* --- Standard CommunicationsInterface --- */
		bool initialize(const char * device = "/dev/ttyUSB0",
		                const char * baud = "9600",
		                const char * = NULL);
		bool open(void);

		/* Override BSTInterface read/write for device-disconnect handling */
		int16_t read(uint8_t * buf, uint16_t buf_size);
		int16_t write(uint8_t * buf, uint16_t size);

		/* Close the serial port */
		bool close();

		/* --- Serial configuration --- */

		/* Change baud rate on an open port */
		bool setBaud(int baud);

		/* Set even parity (8E1) */
		bool setEvenParity();

		/* Enable hardware flow control (RTS/CTS) and modem control */
		bool setFlowControl();

		/* Blocking mode */
		bool setBlocking();
		bool setNonBlocking();
		bool isBlocking() const;

		/* --- Select/fd_set integration --- */
		int setFD(fd_set & set);
		bool checkFD(fd_set & set);

		/* --- Status queries --- */
		SerialStatus getStatus() const;
		const char * getDev() const;
		int getBaud() const;
		int getNumClients() const;

		/* Byte counters */
		long bytesIn() const;
		long bytesOut() const;

		/* Flush I/O buffers */
		void flushIO();

		/* Access to underlying file descriptor (for select-based wrappers) */
		int getFD() const;

	private:
		char dev[BST_MAX_DEV_LENGTH];
		int  baud_rate;
		SerialStatus status;
		bool blocking;
		long bytes_in_total;
		long bytes_out_total;

		speed_t baudToSpeed(int baud);
		bool applyTermiosConfig();
};

#endif
