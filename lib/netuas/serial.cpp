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
 * serial.cpp
 *
 * PURPOSE:
 * This code implements serial communication for vxWorks, Unix, and Windows
 * targets.  For VxWorks, the macro VXWORKS must be defined, otherwise
 * Unix is assumed.
 *  
 * CREATED:
 * 11/16/2000 	- Made for TornadoChaser from code by Cory Dixon
 *
 * LAST MODIFIED:
 * $Author: dixonc $
 * $Date: 2005/09/09 19:35:44 $
 * $Revision: 1.6 $
 ***********************************************************************/

// standard includes

#include <stdlib.h>
#include <unistd.h>
#include <iostream>

#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>


#if !defined (__MACH__)
#include <linux/serial.h>
#endif

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>

#include "type_defs.h"
#include "serial.h"

using namespace std;

speed_t getBaudSpeed( int baud );

SerialPort::SerialPort() : tsStart(0,0), tsStop(0,0), txLast(0,0), rxLast(0,0)
{
	Status   = Ok;
	Sd	      = -1;

	DevName  = NULL;
	Baud	   = 0;
	blocking = false;

	totalBytes[0] = totalBytes[1] = 0;
}

SerialPort::SerialPort( const char * dev, int br ) 
	: tsStart(0,0), tsStop(0,0), txLast(0,0), rxLast(0,0)
{
	Status   = Ok;
	Sd	      = -1;
	blocking = false;

	DevName = new char[ strlen(dev) + 1 ];
	strcpy(DevName, dev);

	Baud	= br;
	totalBytes[0] = totalBytes[1] = 0;
}


SerialPort::SerialPort( const char *dev, const char *br) 
	: tsStart(0,0), tsStop(0,0), txLast(0,0), rxLast(0,0)
{
	Status   = Ok;
	Sd	      = -1;
	blocking = false;

	DevName = new char[ strlen(dev) + 1 ];
	strcpy(DevName, dev);
	Baud	= atoi(br);
	totalBytes[0] = totalBytes[1] = 0;
}

SerialPort::~SerialPort()
{
#ifdef DEBUG
	//printf("~SerialPort()\n"); fflush(stdout);
#endif

	if(Sd != -1)
		if( !close() )
			cerr << "SerialPort::~SerialPort - could not close port" << endl;

	if( DevName != NULL )
		delete [] DevName;
	DevName = NULL;
}

SerialPort::StatusVal SerialPort::init( const char * dev, int br )
{	
	if( DevName != NULL)
		delete [] DevName;
	DevName = NULL;

	DevName = new char[ strlen(dev) + 1 ];
	strcpy(DevName, dev); 
	Baud = br;

	return init();
}

SerialPort::StatusVal SerialPort::init()
{	
	if( DevName == NULL || Baud == 0 )
		return Error;

	// close open port
	if(Sd != -1)
		if( !close() ) {
#ifdef DEBUG
			cout << "SerialPort::init - error closing already open port" << endl;
#endif
		}

	// reset Status of Serial
	//Status = Ok;

	Sd = ::open(DevName, O_RDWR | O_NOCTTY | O_NONBLOCK, 0777);
	if ( Sd == ERROR ) {
		if (Status == Ok) {
#ifdef DEBUG
			cout << "SerialPort::init - error opening " <<  DevName << endl;
			perror("\topen");
#endif
		}
		return Status = Error;
	}

	if(!isatty(Sd)) {
		if (Status == Ok) {
#ifdef DEBUG 
			cout << "SerialPort::init - is not a tty" << endl;
#endif
		}
		(void)close();
		return Status = Error;
	}

	if( !setOptions() ) {
		(void)close();
		return Status = Error;
	}

	if ( !setBaud(Baud) ) {
		(void)close();
		return Status = Error;
	}

	flushIO();

#ifdef DEBUG
	cout << "Serial " << DevName
		<< ":" << Baud
		<< ":" << ( blocking ? "BLOCK" : "NON-BLOCK" )
		<< " is open for communication\n";
#endif

	// not sure if here, but for now
	// make time stamps
	tsStart.stamp();
	tsStop.stamp();
	txRateTS = 0;
	rxRateTS = 0;
	lastTXNum = 0;
	lastRXNum = 0;

	Status = Connected;
	return Status;
}

bool SerialPort::open()
{
	if( init() == Connected )
		return TRUE;
	else 
		return FALSE;
}

void SerialPort::flushIO()
{
#if defined( VXWORKS )
	if ( ioctl(Sd, FIOFLUSH, 0) == ERROR )  {
#ifdef DEBUG
		cout << "SerialPort::flushIO - could not flush buffers" << endl;
#endif
	}
#else
	if( tcflush(Sd, TCIOFLUSH) < 0) {
#ifdef DEBUG
		cout << "SerialPort::flushIO - could not flush buffers" << endl;
		perror("\ttcflush:");
#endif
	}
#endif
}

// set the serial port to blocking io 
bool SerialPort::setBlocking()
{
#if !defined(VXWORKS)
	int flags = 0;		  // holds flag values for fnctl

	// set to be nonblocking (asynchronous)
	if ( (flags = fcntl(Sd, F_GETFL, 0) ) < 0 ){
#ifdef DEBUG
		perror("fcntl - could not get flags for serial port");
#endif
		return false;
	}

	flags &= ~O_NONBLOCK;

	if (fcntl(Sd, F_SETFL, flags ) < 0 ){
#ifdef DEBUG
		perror("fcntl - could not set to blocking ");
#endif
		return false;
	}
#endif

	blocking = true;
	return true;
}

// set the serial port to non-blocking io 
bool SerialPort::setNonBlocking()
{
	int flags = 0;		  // holds flag values for fnctl
#if !defined(VXWORKS)

	// set stdin to be nonblocking (asynchronous)
	if ( (flags = fcntl(Sd, F_GETFL, 0) ) < 0 ){
#ifdef DEBUG
		perror("fcntl - could not get flags for serial port");
#endif
		return false;
	}
	flags |= O_NONBLOCK;

	if (fcntl(Sd, F_SETFL, flags ) < 0 ){
#ifdef DEBUG
		perror("fcntl - could not set to nonblocking ");
#endif
		return false;
	}
#else
	// set stdin to be nonblocking (asynchronous)
	if ( (flags = ioctl(Sd, FIOGETOPTIONS, 0) ) < 0 ){
#ifdef DEBUG
		perror("fcntl - could not get flags for serial port");
#endif
		return false;
	}

	flags |= _FNDELAY | _FNBIO | _FNONBLOCK;

	if ( ioctl(Sd, FIOSETOPTIONS, flags) == ERROR) {
#ifdef DEBUG
		perror("SerialPort::netNonBlocking");
#endif
	}
#endif

	blocking = false;
	return true;
}

/* by default, sets the serial port into a raw byte mode
 *
 *    change newopts to 0x0f to disable hardware flow control. 
 *
 *     0x1F 
 *       0001 1111 
 *       ^^^^ ^^^^ 
 *       |||| ||||===> ignore modem status lines 
 *       |||| |||====> Enable Device Receiver 
 *       |||| ||=====> data bits  01xx =6,  10xx = 7, and 11xx = 8 
 *       ||||========> Hang up on last close
 *       |||=========> send two stop bits (else one)
 *       ||==========> parity detection enabled (else disabled)
 *       |===========> odd parity  (else even)
 *
 */
bool SerialPort::setOptions()
{
#if defined( VXWORKS )

	if ( ioctl(Sd, FIOSETOPTIONS, OPT_RAW)  == ERROR ) {
#ifdef DEBUG
		cout << "SerialPort::setOptions - could not set raw" << endl;
#endif
		return false;
	}

#ifdef DEBUG_PRINT_ALL
	int  oldOptions; 
	ioctl(Sd, SIO_HW_OPTS_GET, (int) &oldOptions); 

	cout << "SerialPort::setOptions - " << DevName << ": " 
		<< ( (oldOptions & CLOCAL ) ? "CLOCAL " : "" )
		<< ( (oldOptions & CREAD ) ? "CREAD " : "" )
		<< (int)(((oldOptions & CSIZE ) >> 2) + 5) << " bit "  
		<< ( (oldOptions & HUPCL ) ? "HUPCL " : "" )
		<< ( (oldOptions & STOPB ) ? "STOPB " : "" )
		<< ( (oldOptions & PARENB ) ? "PARENB " : "" )
		<< ( (oldOptions & PARODD ) ? "PARODD " : "PARODE" )
		<< endl;

	int sioMode=0;
	ioctl( Sd, SIO_MODE_GET, (int)&sioMode);

	cout << "SerialPort::setOptions - " << DevName << " mode="
		<< ( sioMode == SIO_MODE_POLL ? "SIO_MODE_POLL" : "SIO_MODE_INT")	
		<< endl;
#endif

	// flush buffers
	ioctl( Sd, FIOFLUSH, 0);

#else

	//
	// set required port parameters 
	//

	struct termios config;
	if ( tcgetattr( Sd, &config ) != 0 ) {
#ifdef DEBUG
		cout << "SerialPort::setOptions - Unable to poll port settings" <<  endl;
#endif
		return false;
	}

	// control flags
	config.c_cflag |= (CS8 & CSIZE) | 	// 8-bit data
		CREAD |	// enable receiver
		HUPCL |	// hangup on last close
		CLOCAL; 	// ignore modem status lines

	config.c_cflag &= ~( 
			PARENB |	// enable parity
			CRTSCTS | 	// hardware flow control
			CSTOPB        // disable 2 stop bits
			);

	// input flags
	config.c_iflag |= IGNPAR |	// ignore chars with parity error
		IGNBRK ;	// ignore break condition
	config.c_iflag &= ~(
			IXON | IXOFF | 	// software flow control on/off
			BRKINT | 	// no SIGINT on BREAK
			ICRNL | 	// CR-to-NL off
			INPCK | 	// enable input parity checking
			ISTRIP  	// strip 8th bit on input
			);

	// output flags
	config.c_oflag &= ~OPOST;	// turn off post processing

	// local flags
	config.c_lflag &= ~(
			ICANON | 	// canonical mode off
			ECHO   |	// echo off
			IEXTEN | 	// extended input processing off
			ISIG		// signal chars off
			);
	/*
		speed_t speed = B9600;
		if ( cfsetispeed( &config, speed ) != 0 ) {
#ifdef DEBUG
		cout <<  "Problem setting input baud rate" << endl;
#endif
		return false;
		}

		if ( cfsetospeed( &config, speed ) != 0 ) {
#ifdef DEBUG
		cout <<  "Problem setting output baud rate" << endl;
#endif
		return false;
		}
		*/

	if ( tcsetattr( Sd, TCSANOW, &config ) == ERROR ){
#ifdef DEBUG
		perror("SerialPort::setOptions - Unable to set port options");
#endif
		return false;
	}

#if !defined (__CYGWIN__) && !defined(__MACH__)
	// Set low latency
	// will only work for linux
	struct serial_struct settings;
	ioctl(Sd, TIOCGSERIAL, (long int)&settings);
	settings.flags |= ASYNC_LOW_LATENCY;
	ioctl(Sd, TIOCSSERIAL, (long int)&settings);
#endif

#endif

	return true;

}


//  sets the serial port into a raw byte mode
bool SerialPort::setRaw()
{
#if defined( VXWORKS )
	if ( ioctl(Sd, FIOSETOPTIONS, OPT_RAW)  == ERROR ) {
#ifdef DEBUG
		cout << "SerialPort::setOptions - could not set options" << endl;
#endif
		return false;
	}
#else
	struct termios config;
	// set required port parameters 
	if ( tcgetattr( Sd, &config ) != 0 ) {
#ifdef DEBUG
		cout << "SerialPort::setOptions - Unable to poll port settings" <<  endl;
#endif
		return false;
	}

	// local flags
	config.c_lflag &= ~ICANON; 	// canonical mode off

	if ( tcsetattr( Sd, TCSANOW, &config ) < 0 ) {
#ifdef DEBUG
		perror("SerialPort::setOptions - Unable to set port settings to raw");
#endif
		return false;
	}

#endif

	return true;

}


// sets the serial port into a line byte mode
bool SerialPort::setLine()
{
#if defined( VXWORKS )
	if ( ioctl(Sd, FIOSETOPTIONS, OPT_LINE)  == ERROR ) {
#ifdef DEBUG
		cout << "SerialPort::setOptions - could not set options" << endl;
#endif
		return false;
	}
#else
	struct termios config;
	// set required port parameters 
	if ( tcgetattr( Sd, &config ) != 0 ) {
#ifdef DEBUG
		cout << "SerialPort::setOptions - Unable to poll port settings" <<  endl;
#endif
		return false;
	}

	// local flags
	config.c_lflag |= ICANON; 	// canonical mode on

	if ( tcsetattr( Sd, TCSANOW, &config ) < 0 ) {
#ifdef DEBUG
		perror("SerialPort::setOptions - Unable to set line port settings");
#endif
		return false;
	}

#endif

	return true;

}

bool SerialPort::setLocal()
{
#if defined( VXWORKS )

	int  options; 
	ioctl(Sd, SIO_HW_OPTS_GET, (int) &options); 
	options |= CLOCAL;
	ioctl(Sd, SIO_HW_OPTS_SET, options);

#else

	//
	// set required port parameters 
	//

	struct termios config;
	if ( tcgetattr( Sd, &config ) != 0 ) {
#ifdef DEBUG
		cout << "SerialPort::setOptions - Unable to poll port settings" <<  endl;
#endif
		return false;
	}

	// control flags
	config.c_cflag |= CLOCAL; 	// ignore modem status lines
	config.c_cflag &= ~CRTSCTS;	// hardware flow control

	if ( tcsetattr( Sd, TCSANOW, &config ) == ERROR ){
#ifdef DEBUG
		perror("SerialPort::setOptions - Unable to set port options");
#endif
		return false;
	}

#endif

	return true;

}


bool SerialPort::setModem()
{
#if defined( VXWORKS )

	int  options; 
	ioctl(Sd, SIO_HW_OPTS_GET, (int) &options); 
	options = options & ~CLOCAL;
	ioctl(Sd, SIO_HW_OPTS_SET, options);

#else

	//
	// set required port parameters 
	//

	struct termios config;
	if ( tcgetattr( Sd, &config ) != 0 ) {
#ifdef DEBUG
		cout << "SerialPort::setOptions - Unable to poll port settings" <<  endl;
#endif
		return false;
	}

	// control flags
	config.c_cflag &= ~CLOCAL; 	// ignore modem status lines
	config.c_cflag |= CRTSCTS;  // hardware flow control

	if ( tcsetattr( Sd, TCSANOW, &config ) == ERROR ){
#ifdef DEBUG
		perror("SerialPort::setOptions - Unable to set port options");
#endif
		return false;
	}

#endif

	return true;

}

bool SerialPort::setBaud( int baud )
{
#if defined( VXWORKS )

	if ( ioctl(Sd, SIO_BAUD_SET, baud) == ERROR ) {
#ifdef DEBUG
		cout << "SerialPort::SetBaud - could not set baud to " << baud << endl;
#endif
		return false;
	}

#else

	struct termios config;
	speed_t speed = getBaudSpeed( baud );

	if ( tcgetattr( Sd, &config ) != 0 ) {
#ifdef DEBUG
		cout << "Unable to poll port settings" << endl;
#endif
		return false;
	}

	if ( cfsetispeed( &config, speed ) != 0 ) {
#ifdef DEBUG
		cout <<  "Problem setting input baud rate" << endl;
#endif
		return false;
	}

	if ( cfsetospeed( &config, speed ) != 0 ) {
#ifdef DEBUG
		cout <<  "Problem setting output baud rate" << endl;
#endif
		return false;
	}

	if ( tcsetattr( Sd, TCSANOW, &config ) != 0 ) {
#ifdef DEBUG
		cout <<  "Unable to set baud rate settings" << endl;
#endif
		return false;
	}

#endif

	Baud = baud;
	return true;
}


//-------------------------------------------------------------------- 
// Write Functions
//-------------------------------------------------------------------- 
int SerialPort::write( const char * buf, int buf_len )
{
	// check given buffer
	if( buf == NULL || buf_len < 0 )
		return ERROR;

	// check Status of port
	if(Status != Connected || Sd <= 0){
#ifdef DEBUG
		cout << "SerialPort::write - Status = error or port not open" << endl;
#endif
		if(init() == ERROR){
#ifdef DEBUG
			cout << "SerialPort::write - can't open port for writing" << endl;
#endif
			return ERROR;
		} 
	}

	int nwrite=-1;

	nwrite = ::write(Sd, (char *)buf, buf_len);	

	if( nwrite > 0)
		totalBytes[1] += nwrite;

	txLast.stamp();
	return nwrite;
}

int SerialPort::writen( const char * buf, int buf_len )
{

	int nleft = buf_len, nwritten = 0;
	const char *ptr = buf;

	// check given buffer
	if( buf == NULL || buf_len <=0 )
		return ERROR;

	while(nleft > 0) {
		nwritten = SerialPort::write(ptr,nleft);	
		if(nwritten < 0 && errno != EINTR && errno != EAGAIN ) {
#ifdef DEBUG
			cout << "SerialPort::write - returned error value " << errno << endl;
#endif
			return ERROR;
		} else if ( nwritten >= 0) {
			nleft -= nwritten;
			ptr += nwritten;
		}

		// give up processor
		struct timespec ts = {0, (long)1e6};
		(void)nanosleep( &ts, NULL);
	}
	txLast.stamp();
	return (buf_len - nleft);
}


//-------------------------------------------------------------------- 
// Read Functions
//-------------------------------------------------------------------- 
int SerialPort::read( char * buf, int buf_len )
{
	int nread=0;

	// check given buffer
	if( buf == NULL || buf_len <=0 )
		return ERROR;

	// check Status of port
	if(Status != Connected || Sd <= 0){
#ifdef DEBUG
		cout << "SerialPort::read - Status = error or port not open" << endl;
#endif
		if(init() == ERROR){
#ifdef DEBUG
			cout << "SerialPort::read - can't open port for reading" << endl;
#endif
			return Status = Error;
		} 
	}

#ifdef VXWORKS
	// vxworks doesn't have non-blocking so do it myself
	if( blocking == false )
	{
		int nBytes;
		if( ioctl( Sd, FIONREAD, (int)&nBytes) == ERROR ) {
#ifdef DEBUG
			perror("SerialPort::read - ioctl");
#endif
			return Status = Error;
		}

		// if number of bytes in the buffer is 0, return
		if( nBytes == 0 )
			return 0;
	}
#endif

	nread = ::read(Sd, buf, buf_len);

	if(nread < 0 && !(errno == EINTR || errno == EAGAIN)) {
#ifdef DEBUG
		cout << "SerialPort::read - read error" << endl;
#endif
		return ERROR;
	}

	if( nread > 0) {
		totalBytes[0] += nread;
		rxLast.stamp();
	}

	return nread;
}

int SerialPort::readn( char * buf, int buf_len ) 
{
	int nleft, nread;
	char *ptr;
	ptr = buf;

	// check given buffer
	if( buf == NULL || buf_len <=0 )
		return Status = Error;

	nleft = buf_len;
	while(nleft > 0)
	{
		nread = SerialPort::read(ptr, nleft);

		if(nread < 0 && !(errno == EINTR || errno == EAGAIN)) {
			return ERROR;
		} else if ( nread >= 0) {
			nleft -= nread;
			ptr += nread;
		}

		// give up processor
		struct timespec ts = {0, (long)1e6};
		(void)nanosleep( &ts, NULL);
	}

	rxLast.stamp();
	return (buf_len - nleft);
}

//----------------------------------------------------------
// Close the serial port
//----------------------------------------------------------
bool SerialPort::close()
{
	Status = Ok;

	if(Sd < 0)
		return true;

	if( ::close(Sd) < 0)
		Status = Error;

	// marking closing time
	tsStop.stamp();

	Sd = -1;
	if( Status == Error )
		return false;

	return true;
}

//----------------------------------------------------------
// wait on data for serial port
//----------------------------------------------------------
SerialPort::SerialWait SerialPort::wait()
{
	return SerialPort::wait(-1);
}

SerialPort::SerialWait SerialPort::wait(long int msec)
{
	fd_set readFD;		  // read fd set
	SerialWait val = WAIT_DATA;

	struct timeval tv;
	struct timeval *tvPtr=NULL;

	if (msec >= 0) {
		tv.tv_sec = int(msec / 1000);
		tv.tv_usec = (msec % 1000) * 1000;
		tvPtr = &tv;
	}

	// set file descriptor sets for select
	FD_ZERO(&readFD);		// zero read fd set
	FD_SET(Sd,&readFD);		// add serial fd to set

	// wait on selection
	int ret=0;
	if( (ret = select(Sd + 1, &readFD, NULL,NULL,tvPtr)) <= 0){
		if( ret == 0 ) {
			val = WAIT_TIMEOUT;
		} else if (errno == EINTR ) 
			val = WAIT_INT;
		else {
#ifdef DEBUG
			perror("select");
#endif
			val = WAIT_ERROR;
		}
		return val;
	}

	//
	// get input from listenFD
	//
	if( ! FD_ISSET(Sd, &readFD) )
		val = WAIT_ERROR;

	return val;
}

//----------------------------------------------------------
// Print out statistics about the socket port
//----------------------------------------------------------
void SerialPort::printStats()
{
	uint32_t msec;
	TimeStamp tsNow;

	if( Status == Connected ) {
		tsNow.stamp();
		msec = tsNow - tsStart; //micro secs
	} else 
		msec = tsStop - tsStart; //micro secs

#ifdef DEBUG
	cout << "SerialPort Stats:" << endl << "--------------------" << endl
		<< "\tConnected=" << ( Status == Connected ? "TRUE" : "FALSE" )
		<< "\t\tUptime=" << msec/1000000.0 << " [s]" << endl
		<< "\tIn="  << totalBytes[0] << " [bytes]"
		<< "\t\tRate=" << totalBytes[0]/(float)(msec/1000000.0)<<" [Bps] "<<endl
		<< "\tOut="  << totalBytes[1] << " [bytes]"
		<< "\t\tRate=" << totalBytes[1]/(float)(msec/1000000.0)<<" [Bps] "<<endl
		<< endl;
#endif
}


//----------------------------------------------------------
// return up time in seconds
//----------------------------------------------------------
float SerialPort::getUpTime() const
{
	if( Status == Connected ) {
		TimeStamp now;
		now.stamp();

		return (float)( (now - tsStart) / SEC2MICRO) ;
	}

	return -1.0;
}

//----------------------------------------------------------
// return time last received a byte
//----------------------------------------------------------
float SerialPort::getLastRxTime() const
{
	if( rxLast.time() == 0.0 )
		return -1.0;

	TimeStamp now;
	now.stamp();

	return (float)( (now - rxLast) / SEC2MICRO) ;
}

float SerialPort::getLastTxTime() const
{
	if( txLast.time() == 0.0 )
		return -1.0;

	TimeStamp now;
	now.stamp();

	return (float)( (now - txLast) / SEC2MICRO) ;
}

float SerialPort::rateRx() 
{
	long msec;
	float rate;
	TimeStamp tsNow;

	tsNow.stamp();
	msec = tsNow - rxRateTS; //micro secs
	rxRateTS.stamp();

	rate = (totalBytes[0] - lastRXNum)/(float)(msec/1000000.0);
	lastRXNum = totalBytes[0];

	return rate;
}

float SerialPort::rateTx() 
{
	long msec;
	float rate;
	TimeStamp tsNow;

	tsNow.stamp();
	msec = tsNow -  txRateTS; //micro secs
	txRateTS.stamp();

	rate = (totalBytes[1] - lastTXNum)/(float)(msec/1000000.0);
	lastTXNum = totalBytes[1];

	return rate;
}
//----------------------------------------------------------
// Set the file descriptor set with clients file descriptor
// to be used for select
//----------------------------------------------------------
int SerialPort::setFD(fd_set &set)
{
	if( Sd > 0 ){
		FD_SET(Sd,&set);	  // add socket fd to set
	} else
		return ERROR;

	return Sd;
}	   

//----------------------------------------------------------
// see if the socket fd is set in the fd_set passed in
// and return a bool valid if set
//----------------------------------------------------------
bool SerialPort::checkFD(fd_set &set)
{
	if( Status == Error )
		return FALSE;
	else if(FD_ISSET(Sd, &set))
		return TRUE;
	else
		return FALSE;
}

#if ( !defined( VXWORKS ) )
speed_t getBaudSpeed( int baud )
{
	speed_t speed = B9600;

	if ( baud == 300 ) {
		speed = B300;
	} else if ( baud == 1200 ) {
		speed = B1200;
	} else if ( baud == 2400 ) {
		speed = B2400;
	} else if ( baud == 4800 ) {
		speed = B4800;
	} else if ( baud == 9600 ) {
		speed = B9600;
	} else if ( baud == 19200 ) {
		speed = B19200;
	} else if ( baud == 38400 ) {
		speed = B38400;
	} else if ( baud == 57600 ) {
		speed = B57600;
	} else if ( baud == 115200 ) {
		speed = B115200;
	} else if ( baud == 230400 ) {
		speed = B230400;
#ifdef B460800
	} else if ( baud == 460800 ) {
		speed = B460800;
#endif
#ifdef B500000
	} else if ( baud == 500000 ) {
		speed = B500000;
#endif
#ifdef B921600
	} else if ( baud == 921600 ) {
		speed = B921600;
#endif
#ifdef B1000000
	} else if ( baud == 1000000 ) {
		speed = B1000000;
#endif
#ifdef B1152000
	} else if ( baud == 1152000 ) {
		speed = B1152000;
#endif
#ifdef B1500000
	} else if ( baud == 1500000 ) {
		speed = B1500000;
#endif
#ifdef B2000000
	} else if ( baud == 2000000 ) {
		speed =  B2000000;
#endif
#ifdef B2500000
	} else if ( baud == 2500000 ) {
		speed = B2500000;
#endif
#ifdef B3000000
	} else if ( baud == 3000000 ) {
		speed = B3000000;
#endif
#ifdef B3500000
	} else if ( baud == 3500000 ) {
		speed = B3500000;
#endif
#ifdef B4000000
	} else if ( baud == 4000000 ) {
		speed = B4000000;
#endif
	} else {
#ifdef DEBUG
		cout << "Unsupported baud rate " << baud;
#endif
	}

	return speed;
}
#endif
