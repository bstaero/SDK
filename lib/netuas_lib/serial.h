/*=+--+=#=+--     Unmanned Aerial System Management Software      --+=#=+--+=#*\
|									Copyright (C) 2009 Jack Elston, Cory Dixon                   |
|                                                                              |
      This program is free software; you can redistribute it and/or             
      modify it under the terms of the GNU General Public License               
      as published by the Free Software Foundation; either version 2            
      of the License, or (at your option) any later version.                    
                                                                                
      This program is distributed in the hope that it will be useful,           
      but WITHOUT ANY WARRANTY; without even the implied warranty of            
      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             
      GNU General Public License for more details.                              
                                                                                
      You should have received a copy of the GNU General Public License         
      along with this program; if not, write to the Free Software               
      Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA                 
      02111-1307, USA.                                                          
                                                                                
 			  		Jack Elston                       Cory Dixon                        
|			  		elstonj@colorado.edu              dixonc@colorado.edu              |
|                                                                              |
\*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*/

/***********************************************************************
 *			
 * FILENAME:
 * serial.h
 *
 * PURPOSE:
 * This code implements serial communication for vxWorks, Unix, and Windows
 * targets.  For VxWorks, the macro VXWORKS must be defined, otherwise
 * Unix is assumed.
 *  
 * CREATED:
 * 11/16/2000 	- Made for TornadoChaser from cody by Cory
 *
 * LAST MODIFIED:
 * $Author: dixonc $
 * $Date: 2005/07/13 21:00:06 $
 * $Revision: 1.5 $
 *
 ***********************************************************************/
#ifndef _SERIAL_H
#define _SERIAL_H

#ifndef __cplusplus
# error This library requires C++
#endif

#include "timeLib.h"
#include <string.h>

// <summary> Provides the serial port interface </summary>
//
// Provides an interface to serial ports running on vxWorks, Unix and Windows.
// The basic steps to using are to declare a SerialPort, initialize
// it, then use it.
//
class SerialPort {
	public:
		// <summary>  Enum defining the status of the serial port</summary>
		enum StatusVal { 
			// there is an error with port
			Error=-1, 
			// the serial port is ok but not running
			Ok, 
			// the serial port is ok and running
			Connected 
		};

		// <summary>  Enum defining the type of return value from wait()</summary>
		// This type definition defines the type of the return value from the 
		// wait member function.
		enum SerialWait {       
			// wait was interrupted by signal
			WAIT_INT		= -3,
			// wait was interrupted by signal
			WAIT_TIMEOUT 	=-2,
			// there is an error while waiting
			WAIT_ERROR		= -1,
			// data is ready
			WAIT_DATA		= 0
		};

		typedef int fd_type;

	private:

		fd_type   Sd;			// serial file descriptor
		char *    DevName;		// device name
		int       Baud;		// Baud rate
		StatusVal Status;		// current Status of serialPort

		uint32_t    totalBytes[2];	// 0 is in 1 is out
		TimeStamp tsStart;
		TimeStamp tsStop;
		TimeStamp txLast;
		TimeStamp rxLast;

		// used to caluclate rate
		TimeStamp txRateTS;
		TimeStamp rxRateTS;
		uint32_t lastTXNum;
		uint32_t lastRXNum;

		bool blocking;          // non-blocking/blocking

		// set options
		// by default, sets the serial port into a raw byte mode
		bool setOptions();

	public:

		// constructors. The dev is the device file name, typically
		// "/tyCo/0", the br is the baud rate of the serial port, normally
		// 9600 baud, and finally the options associated with the port.
		// See the vxWorks users manual for all definitions, the two of 
		// most interest are OPT_RAW and OPT_LINE.  OPT_RAW is a raw
		// interface, while OPT_LINE puts the serial port into line mode
		// which means characters arent sent untill a cr/lf are sent.
		SerialPort( const char* dev, int br=9600);
		SerialPort( const char* dev, const char * br);
		SerialPort();

		~SerialPort();

		// get the status of the serial port
		StatusVal getStatus() const		{ return Status; }
		// get the file descriptor
		fd_type getFd(void) const		{ return Sd;  }
		// return device strng
		const char * getDev(void) const 	{ return DevName; }
		// return baud rate
		int getBaud( void ) const		{ return Baud; }
		// set baud rate
		bool setBaud(int b);

		// Initialize the serial port. if the serial port settings
		// where specified at constructor level, then simply use init(),
		// other wise you need to specify the seetins.<p>
		// The dev is the device file name, typically
		// "/tyCo/0", the br is the baud rate of the serial port, normally
		// 9600 baud, and finally the options associated with the port.
		// See the users manual for the operating system that this is
		// being used in to set more options than a raw terminal setting
		// 
		// open & init do the same thing, just a different function interface
		// so that open matches the socket.
		StatusVal init( const char * dev, int br);
		StatusVal init();
		bool open();

		// close the serial port
		bool close();

		// These functions read/write data to the serial port.
		// The writen/readn function is the same as the write/read functions
		// except they NEVER return untill the buf_len of data
		// was transmitted/received.  Se be careful with writen and readn
		// funcitons.
		int writen ( const char * buf, int buf_len ); 
		int write  ( const char * buf, int buf_len ); 

		int readn ( char * buf, int buf_len );
		int read  ( char * buf, int buf_len ); 

		void flushIO();

		float getLastTime(int &num) {
			TimeStamp now;
			now.stamp();
			return (float)( (now - rxLast) / SEC2MICRO) ;
		}

		// Wait on activity,  timeout can be specified in milli-seconds. 
		// If the wait timedout, then WAIT_ERROR will be returned.
		// The return values are:
		// <ul>
		//  <li> WAIT_ERROR   -> there is an error while waiting
		//  <li> WAIT_INT     -> wait was interrupted by signal
		//  <li> WAIT_TIMEOUT -> wait was interrupted by signal
		//  <li> WAIT_DATA    -> data is ready
		// </ul>
		// <p>
		// Before calling the wait function again in a loop, make sure 
		// all file descriptors have been processed and handled. 
		SerialWait wait();
		SerialWait wait(long int msec);

		// utilitie functions for the socket for using select
		// setFD sets the fd_set with the serial descriptor
		// It return the maxFD that was added to the set.
		// checkFD checks the set for the serial descriptor,
		// and returns TRUE if set
		int setFD(fd_set &set);
		bool checkFD(fd_set &set);

		// Set the serial port into a blocking/non-blocking mode
		bool setNonBlocking();
		bool setBlocking();
		bool isBlocking() { return blocking; }

		// Set the serial port into line/raw mode
		bool setLine();
		bool setRaw();
		bool setLocal();
		bool setModem();

		// get bytes sent/received
		size_t in() const { return totalBytes[0]; }
		size_t out() const { return totalBytes[1]; }

		// Print out statistics about the socket port
		void printStats();

		// get time information on port
		float getUpTime() const;
		float getLastRxTime() const;
		float getLastTxTime() const;
		float rateRx();
		float rateTx(); 
		int getNumClients() {return Status==Connected;}
		const char * getClientIP(int& num) {return "serial";}
		int getClientInd(int& num) {return 0;}
};

#endif //_SERIAL_H

