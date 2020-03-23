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
 * socket.h
 *
 * PURPOSE:
 *  Socket I/O routines
 *
 * CREATED:
 * 11/2000 by Cory Dixon
 *
 * LAST MODIFIED:
 * $Author: dixonc $
 * $Date: 2005/07/26 19:43:28 $
 * $Revision: 1.5 $
 *
 ***********************************************************************/

#ifndef _SOCKET_H
#define _SOCKET_H

#ifndef __cplusplus
# error This library requires C++
#endif


# include <sys/types.h>
# include <sys/socket.h>
# include <netdb.h>

# include <strings.h>
# include <string.h>
# include <string>

#include "timeLib.h"
//#include <gdk/gdk.h>

// The socket has a problem handling clients connecting
// and disconnecting randomly. It is diffinetly due
// to my stupid indexing scheme which needs to be completey
// changed. If your sockets are stable, then you can change
// these numbers to what you need, just be aware!
#define MAX_SOCKET_QUEUE 20
#define NUM_CLIENTS 	   20
#define INVALID_SOCKET 	(-1)

// <summary> A socket I/O class </summary>
//
// When calling the constructor you need to provide a host name, a
// port number, and a socket type. The convention used by the
// Socket class is that the server side listens and the client
// side sends. For a server socket, the host name should be
// empty. For a server, the port number is optional, if you do not
// specify a port, the system will assign one. For a client
// socket, you need to specify both a destination host and
// destination port. For both client and server sockets you must
// specify the socket type. Type must be either udp or tcp. Here's
// a quick breakdown of the major differences between UDP and TCP
// type sockets.
//
// TCP sockets are the type where you establish a connection and
// then can read and write to the socket from both ends. If one
// end of TCP socket connect quits, the other end will likely
// segfault if it doesn't take special precautions.  But, the nice
// thing about TCP connections is that the underlying protocol
// guarantees that your message will get through. This imposes a
// certain performance overhead though on the communication
// because the protocol must resend failed messages. TCP sockets
// are good for sending periodic command/response type messages
// where performance isn't a big issues, but reliability is.
//
// UDP sockets on the other hand are a lower level protocol and
// don't have the same sort of connection as TCP sockets. With UDP
// sockets, the server end just sits and listens for incoming
// packets from anywhere. The client end sends it's message and
// forgets about it. It doesn't care if there isn't even a server
// out there listening and all the packets are getting
// lost. Although systems/networks usually do a pretty good job
// (statistically) of getting your UDP packets to their
// destination, there is no guarantee that any particular packet
// will make it. But, because of this low level implementation and
// lack of error checking, UDP packets are much faster and
// efficient. UDP packets are good for sending positional
// information to synchronize two applications. In this case, you
// want the information to arrive as quickly as possible, and if
// you lose a packet, you'd rather get new updated information
// rather than have the system waste time resending a packet that
// is becoming older and older with every retry.

class Socket {
	public:
		typedef int SocketFD;

		// <summary>  Enum defining the mode of the socket, client/host</summary>
		// This type definition defines the side of the connection the 
		// socket is used for.
		enum SocketMode {	
			// make a server socket
			SERVER, 	
			// make a client socket
			CLIENT,
			LISTENING_CLIENT,
			RECEIVING_CLIENT,
			DUPLEX,
			DUPLEX_CLIENT,
			DUPLEX_SERVER
		};

		// <summary>  Enum defining the type of connection between sockets </summary>
		// This type definition defines the type of the connection the 
		// socket uses
		enum SocketType {	
			// use a datagram connection (SOCK_DGRAM)
			UDP=0,
			// use a guranteed stream connection (SOCK_STREAM)
			TCP=1
		};

		// <summary>  Enum defining the type of return value from wait()</summary>
		// This type definition defines the type of the return value from the 
		// wait member function.  The client numbers are represented by the 
		// bit settings with no more than 30 clients.
		enum SocketWait {	
			// there is an error while waiting
			WAIT_INT		= -3,
			WAIT_TIMEOUT	= -2,
			WAIT_ERROR		= -1,
			WAIT_DATA		=  0,
			// there is a connection from peer
			WAIT_PEER		= 0x8000
		};

	private:
		std::string   hostname;
		std::string   portStr;
		uint16_t port;
		bool     connected;
		bool     is_blocking;
		bool     status_out;


		unsigned long int totalBytes[2]; // 0 is in 1 is out
		TimeStamp tsStart;
		TimeStamp tsStop;
		TimeStamp tsLast[ NUM_CLIENTS+1 ];

		// used to caluclate rate
		TimeStamp txRateTS;
		TimeStamp rxRateTS;
		uint32_t lastTXNum;
		uint32_t lastRXNum;

		SocketFD   sock;		// the socket file descriptor
		SocketType type;		// type ( udp=SOCK_DGRAM , tcp=SOCK_STREAM)
		SocketMode mode;		// mode ( client/server )
		struct sockaddr_in serverAddr;	// server's socket address

		// if server, then some client data
		int numClients;               	// number of clients connected
		int clientIndex[NUM_CLIENTS]; 		// index of connected clients
		SocketFD clientFD[NUM_CLIENTS+1];	// fd of connected clients
		//GPtrArray *clientList;

		// clients socket address, the +1 is on purpose and is for a UDP
		// connection from a new client, basically a temp variable to track
		// connections on a UDP socket
		struct sockaddr_in clientAddr[NUM_CLIENTS+1]; 

		// make a server, master listening, socket
		void make_server_socket();
		// make a client socket
		void make_client_socket();

		// abrubtly cancel a client connection, and close
		// the open file descriptor
		void fastClose();

		// Specify the SO_REUSEADDR option  to bind a stream socket to a local
		// port  that may be still bound to another stream socket that may be
		// hanging around with a "zombie" protocol control block whose context
		// is  not yet freed from previous sessions.
		// 
		// Specify the SO_KEEPALIVE option, and the transport protocol (TCP)
		// initiates  a timer to detect a dead connection which prevents an
		// an application from  hanging on an invalid connection.
		// 
		// Specify the TCP_NODELAY option for protocols such as X Window System
		// Protocol that require immediate delivery of many small messages.
		// By default VxWorks uses congestion avoidance algorithm
		// for virtual  terminal  protocols and  bulk  data  transfer
		// protocols. When the TCP_NODELAY option is turned on and there are
		// segments to be sent out, TCP  bypasses  the  congestion
		// avoidance  algorithm  and sends the segments out if there
		// is enough space in the send window.
		// 
		// Specify the SO_LINGER option to perform a  "graceful" close.
		// A graceful close occurs when a connection is shutdown, TCP  will
		// try  to  make sure that all the unacknowledged data in transmission
		// channel are acknowledged and the peer is shutdown properly by going
		// through an elaborate  set  of  state transitions.
		// 
		bool setopt(bool broadcast = false);


	public:

		// Create an instance of Socket.
		// <ul>
		// 	<li>host - name of host
		// 	<li>port - port number if we care to choose one
		// 	<li>type - specify "udp" or "tcp"
		// </ul>
		Socket( const char * host, const char * port, SocketType type );

		// Destructor
		~Socket();

		// If specified as a server, open the master
		// listening socket.  If specified as a client,
		// open a connection to a server. Mode is the mode
		// of connection to the other side, "udp" or "tcp"
		bool open( const SocketMode mode );

		// read data from socket. The return value is the number of bytes
		// read by the socket, -1 for error.  The first two provide generic
		// reads based on the type of data, the raw buffer or a command. The
		// last two are for server sockets, who want to read from a specific
		// client 
		// <group>
		int read( char *buf, int length );
		int read( int client, char *buf, int length );
		// </group>

		// write data to a socket. The return value is the number of bytes
		// written by the socket, -1 for error.  The first provide generic
		// writes based on the type of data, the raw buffer or a command. The
		// last two are for server sockets, who want to write to a specific
		// client. By default, on a raw interface we do not track sequency numbers
		// <group>
		int write(in_addr address, const char *buf, const int length, bool noSeq = true );
		int write( const char *buf, const int length, bool noSeq = true );
		int write( int client, const char *buf, const int length, bool noSeq = true );
		// </group>

		// close a socket connections
		bool close();

		// Enable/disable non-blocking mode.
		// (O_NDELAY==O_NONBLOCK)
		// <group>
		bool setBlocking(SocketFD fd = -1);
		bool setNonBlocking(SocketFD fd = -1);
		// </group>

		// Member get functions:
		// <ul>
		// <li> return the remote host name as a String
		// <li> return the port number as a String
		// </ul>
		// <group name=getFunctions>
		//String 		  getHostname() const { return hostname; }
		//String 		  getPort() const { return portStr; }
		SocketMode 	getMode() const { return mode; }
		SocketType 	getType() const { return type; }
		int 			  getNumClients() const { return numClients; }
		float			  getUpTime() const;
		float			  getLastTime(int client) const;
		int		 	    getClientFromRead() const;

		int	getClientInd(size_t client) const { 
			/*
				if(clientList == NULL )
				return -1;
				if( clientList->len <= client)	
				return -1;
				int * temp = (int *)g_ptr_array_index(clientList, client);	
				if( temp == NULL )
				return -1;
				else
				return *temp;
				*/
			return client;
		};
		int	getClientFromInd(int ind) const {
			/*
				for(int i=0; i<clientList->len; i++) {
				if( ind == *(int*)g_ptr_array_index( clientList, i) )
				return i;
				}
				return -1;
				*/
			return ind;
		};
		// </group>

		// Wait on activity,  timeout can be specified in milli-seconds. 
		// If the wait timedout, then WAIT_ERROR will be returned.
		// If this is a client, then return values are:
		// <ul>
		//  <li> WAIT_ERROR -> error
		//  <li> WAIT_PEER -> data from server 
		// </ul>
		// If this is a server, return values are:
		// <ul>
		//  <li> WAIT_ERROR -> error
		//  <li> WAIT_PEER -> listening file descriptor, client connecting
		//  <li> 1 -> first client active
		//  <li> 2 -> second client active
		//  <li> 4 -> third client active
		// </ul>
		// For the client, the number is set by bit position of the 
		// client number.  The first client then sets the first bit, client
		// 2 sets the second bit.  The reason for this is that there can
		// be more than one client active at a time. 
		// <p>
		// Before calling the wait function again in a loop, make sure 
		// all clients have been processed and handled. 
		// See man select
		// <group>
		SocketWait wait();
		SocketWait wait(long int msec);
		// </group>

		// utilitie functions for the socket for using select
		// <group>
		int setFD(fd_set &set, bool is_tx = false);
		SocketWait checkFD(fd_set &set);
		// </group>

		// Connect
		// Make connection to other side based on whether
		// the local socket is a host or a client.  
		// <group>
		bool connectHost();
		int connectClient(); // returns client ind, or -1

		in_addr getServerIP() const;
		in_addr getClientIP(int client) const;
		// </group>

		// If the socket is a server, then the server can remove a client connection.
		// Once a host is connected, there is no way to remove the host connection 
		// unless it is closed. 
		// 	returns the number of the client removed or ERROR if error
		int removeClient(int client);

		// If this is a client, then this returns true if connected to a 
		// host.  If it is a host, then a true is returned if a single
		// client connection exsists.
		bool isConnected() const { return connected; } 
		bool isClosed() const { return sock == INVALID_SOCKET; } 

		// get bytes sent/received
		int in() const { return totalBytes[0]; }
		int out() const { return totalBytes[1]; }
		float rateRx();
		float rateTx();


		// Print out statistics about the socket port
		void printStats();

};


#endif // SOCKET_H
