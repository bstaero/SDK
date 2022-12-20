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
 * socket.cxx
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
 * $Revision: 1.7 $
 *
 ***********************************************************************/

// standard headers
#include <sys/time.h>	// select()
#include <sys/types.h>	// socket(), bind(), select(), accept()
#include <sys/socket.h>	// socket(), bind(), listen(), accept()
#include <netinet/in.h>	// struct sockaddr_in
#include <arpa/inet.h>   // inet_ntoa
#include <netdb.h>		// gethostbyname()
#include <fcntl.h>		// fcntl()

#include <sys/ioctl.h>
#include <net/if.h>

#ifndef __APPLE__
#include <netinet/ether.h>
#endif

# define ERROR -1
# define OK 0

#include <unistd.h>		// select(), fsync()/fdatasync(), fcntl()
#include <errno.h>		// errno and EINTR
#include <stdio.h>		// perror
#include <netinet/tcp.h>// TCP_NODELAY
#include <errno.h>		// errno

#include <iostream>
#include <stdlib.h>

// project header
#include "socket.h"
#include "timeLib.h"
#include "color.h"
#include "debug.h"

using namespace std;

//----------------------------------------------------------
// socket constructor
//----------------------------------------------------------
Socket::Socket( const char * host, const char * port, SocketType connection ) 
	: hostname(host), portStr(port), tsStart(0,0), tsStop(0,0)
{
	numClients 	  = 0;
	totalBytes[0] = totalBytes[1] = 0;
	connected 	  = FALSE;
	sock 		      = INVALID_SOCKET;
	type 		      = connection;
	is_blocking   = true;
	status_out    = false;

	for(int i=0; i < NUM_CLIENTS; i++) {
		clientIndex[i] = -1;
		clientFD[i] = INVALID_SOCKET;
		clientAddr[i].sin_addr.s_addr = 0;
	}

	clientFD[NUM_CLIENTS] = 0;
	clientAddr[NUM_CLIENTS].sin_addr.s_addr = INADDR_NONE;
}

//----------------------------------------------------------
// socket destructor
//----------------------------------------------------------
Socket::~Socket() 
{
	//printf("~Socket()\n");fflush(stdout);
	if(sock > 0)
		close();		// call member function close
}

//----------------------------------------------------------
// make the socket a server socket
//----------------------------------------------------------
void Socket::make_server_socket () 
{
#ifdef VXWORKS
	int length;
#else
	socklen_t length;
#endif

	// Zero out the sock_addr structures.
	// This MUST be done before the socket calls.
	memset((char *) &serverAddr, 0, sizeof (serverAddr));

	// Create the socket.
#ifdef VXWORKS
	serverAddr.sin_family = AF_INET;
	if( type == Socket::UDP)
		sock = socket (AF_INET, SOCK_DGRAM, 0);
	else if( type == Socket::TCP )
		sock = socket (AF_INET, SOCK_STREAM, 0);
	else
		pmesg(VERBOSE_ERROR, "Socket::make_server_socket - invalid type\n");
#else
	serverAddr.sin_family = AF_INET;
	if( type == Socket::UDP)
		sock = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	else if( type == Socket::TCP )
		sock = socket (AF_INET, SOCK_STREAM, IPPROTO_TCP);
	else
		pmesg(VERBOSE_ERROR, "Socket::make_server_socket - invalid type=%u\n",  type);
#endif

	if (sock == INVALID_SOCKET) {
		pmesg(VERBOSE_ERROR, "Socket::make_server_socket - socket() failed ");
		return;
	}

	// set socket options 
	if( !setopt() ) {
		this->close();
		return;
	}

	// Set up our internet address, and bind it so the client can connect.
	// set port to zero if you want the to let the system pick 
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddr.sin_family      = AF_INET;
	serverAddr.sin_port 	      = htons(port);
#ifdef VXWORKS
	serverAddr.sin_len	      = sizeof(struct sockaddr_in);
#endif

	if (::bind (sock, (struct sockaddr *) &serverAddr, sizeof (serverAddr)) != 0) {
		pmesg(VERBOSE_ERROR, "Socket::make_server_socket - bind() failed\n");

		this->close();
		return;
	}

	// Find the assigned port number
	length = sizeof(struct sockaddr_in);
	if ( getsockname(sock, (struct sockaddr *) &serverAddr, &length) ) {
		pmesg(VERBOSE_ERROR, "Socket::make_server_socket -  getsockname() failed\n");
		this->close();
		return;
	}
	port = ntohs(serverAddr.sin_port);
}

//----------------------------------------------------------
// make the socket a client socket
//----------------------------------------------------------
void Socket::make_client_socket () 
{
	// Zero out the sock_addr structures.
	// This MUST be done before the socket calls.
	memset((char  *) &serverAddr, 0, sizeof (serverAddr));

	// Create the socket.

#ifdef VXWORKS
	if( type == Socket::UDP)
		sock = socket (AF_INET, SOCK_DGRAM, 0);
	else if( type == Socket::TCP )
		sock = socket (AF_INET, SOCK_STREAM, 0);
	else
		cout << "Socket::make_server_socket - invalid type" << endl;
#else
	if( type == Socket::UDP)
		sock = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	else if( type == Socket::TCP )
		sock = socket (AF_INET, SOCK_STREAM, IPPROTO_TCP);
	else
		cout << "Socket::make_server_socket - invalid type=" << type<< endl;
#endif
	if (sock <= 0) {
		perror("Socket::make_client_socket() - socket() failed");
		sock = INVALID_SOCKET;
		return;
	}

	// Set up our internet address, and bind it so the client can connect. 
#ifdef VXWORKS
	serverAddr.sin_family	= AF_INET;
	serverAddr.sin_len		= sizeof(struct sockaddr_in);
#else
	serverAddr.sin_family	= AF_INET;
#endif // VXWORKS
	serverAddr.sin_port		= htons (port);

	// set the socket options 
	if( !setopt() )
		close();

}

//----------------------------------------------------------
// Connect to the host
//	Opens a new socket if needed
//----------------------------------------------------------
bool Socket::connectHost()
{

#ifdef VXWORKS
	// in VxWorks, it seems we need to close the connection
	// every time and reopen it for a TCP type conneciton
	if( sock != INVALID_SOCKET) 
		this->close();
#endif

	if( sock == INVALID_SOCKET) 
		this->open(CLIENT);			// open new socket

	// get the hosts official serverAddr/info
	if( sock != INVALID_SOCKET) {
#ifndef VXWORKS
		struct hostent *hp=NULL;
		if( (int)(serverAddr.sin_addr.s_addr = inet_addr((char *)hostname.c_str() ) ) == ERROR &&
				(hp = gethostbyname( hostname.c_str())) == NULL ) 
		{
			perror("Socket::make_client_socket() - hostname lookup failed");
			this->close();
			return false;
		}
		// Connect this socket to the host and the port specified 
		// memcpy( &(serverAddr.sin_addr.s_addr), hp->h_addr, hp->h_length);
		if( hp != NULL )
			bcopy ( hp->h_addr, &(serverAddr.sin_addr.s_addr), hp->h_length);

		//cout << "hp->h_addr=" << hp->h_addr << " *(hp->h_addr)=" << *(hp->h_addr) << endl;
		//cout << "h_addr_list=" << hp->h_addr_list << " h_addr_list[1]=" << hp->h_addr_list[1] << " *h_addr_list[1]=" << *(hp->h_addr_list[1]) << endl;
		//cout << "Socket::make_client_socket() - hostname=" 
		//<<  (char *)hostname.c_str() << "ip=" << inet_ntoa(serverAddr.sin_addr)
		//<< endl;
#else
		if( (long)(serverAddr.sin_addr.s_addr = inet_addr((char *)hostname.c_str() ) ) == ERROR &&
				( (long)(serverAddr.sin_addr.s_addr = hostGetByName( (char *)hostname.c_str() )) == ERROR ) ) 
		{
			cout << "Socket::make_client_socket() - hostname lookup failed" << endl;
			this->close();
			return false;
		} else 
			cout << "Socket::make_client_socket() - hostname=" 
				<<  (char *)hostname.c_str() << " ip=" << inet_ntoa(serverAddr.sin_addr)
				<< endl;
#endif
		// socket is not open
	} else { 
		perror("Socket::connectHost - socket is not open");
		return false;
	}

	// if TCP, then make the actual circuit connection
	if( type == TCP ) {
		if ( ::connect(sock, (struct sockaddr *) &serverAddr, sizeof(struct sockaddr_in)) == OK ) {
			connected=TRUE;
			status_out = false;

			clientAddr[NUM_CLIENTS].sin_addr.s_addr = serverAddr.sin_addr.s_addr;
		} else if( !status_out ) {
			perror("Socket::connectHost - could not connect to host");
			status_out = true;
		}
/*
		if(mode == CLIENT || mode == DUPLEX_CLIENT) {
			char          buf[1024];
			struct ifconf ifc;
			struct ifreq *ifr;
			int           nInterfaces;
			//int           i;

			ifc.ifc_len = sizeof(buf);
			ifc.ifc_buf = buf;
			if(ioctl(sock, SIOCGIFCONF, &ifc) < 0) {
				perror("ioctl(SIOCGIFCONF)");
			}

			ifr         = ifc.ifc_req;
			nInterfaces = ifc.ifc_len / sizeof(struct ifreq);
//			for(i = 0; i < nInterfaces; i++)
//			{
//				struct ifreq *item = &ifr[i];
//				if( strcmp("lo",item->ifr_name) == 0) 
//					continue;
//
//				//FIXME - check for what subnet we should be attached to
//				//if(((struct sockaddr_in *)&item->ifr_addr)->sin_addr.s_addr)
//			}
//		 BEGIN WORKAROUND    //
			struct ifreq *item = &ifr[0];
			clientAddr[NUM_CLIENTS].sin_addr.s_addr = ((struct sockaddr_in *)&item->ifr_addr)->sin_addr.s_addr;
//		 END WORKAROUND      //

		}
*/

	} else
		connected=TRUE;

	if(connected) {
		cout << "Client " YELLOW << hostname << ATTROFF
			<< ":" GREEN << port << ATTROFF
			<< ":" RED << (type == UDP ? "UDP" : "TCP") 
			<< ATTROFF " is connected\n";
	}

	return connected;
}

//----------------------------------------------------------
// If specified as a server  open the master listening socket.  
// If specified as a client,  open a connection to a server.
//----------------------------------------------------------
bool Socket::open( const SocketMode modeVal )
{
	mode = modeVal;

	if ( portStr == "" || portStr == "any" ) {
		port = 0; 
	} else {
		port = atoi( portStr.c_str() );
	}

	if ( mode == SERVER || mode == LISTENING_CLIENT || mode == DUPLEX_SERVER ) {
		// Setup socket to listen on.  Set "port" before making this call. 
		make_server_socket();
		if ( sock == INVALID_SOCKET ) {
			cout <<  "Socket::open - socket creation failed" << endl;

			return false;
		}

		if ( type == UDP ) {
			// Non-blocking UDP
			setNonBlocking();
		} else {
			// Blocking TCP
			// Specify the maximum length of the connection queue
			if( listen( sock, MAX_SOCKET_QUEUE ) < 0 ) {
				perror("Socket::open - listen failed");
				::close(sock);

				return false;
			}
		}
		cout << "Server " YELLOW << hostname << ATTROFF
			<< ":" GREEN << port << ATTROFF
			<< ":" RED << (type == UDP ? "UDP" : "TCP") << ATTROFF
			<< " is waiting for connection(s)\n";

	} else if ( mode == CLIENT || mode == DUPLEX_CLIENT || mode == RECEIVING_CLIENT ) {
		// make a client side socket
		make_client_socket();
		if ( sock == INVALID_SOCKET ) {
			cout <<  "Socket::open - socket creation failed" << endl;
			return false;
		}

		// Non-blocking UDP
		if ( type == UDP ) {
			setNonBlocking();
		} 

		cout << "Client " YELLOW << hostname << ATTROFF
			<< ":" GREEN << port << ATTROFF
			<< ":" RED << (type == UDP ? "UDP" : "TCP") << ATTROFF
			<< " is attempting connection(s)\n";

	} else if ( mode == DUPLEX ) {
		make_server_socket();
		if ( sock == INVALID_SOCKET ) {
			cout <<  "Socket::open - socket creation failed" << endl;
			return false;
		}

		// Non-blocking UDP
		if ( type == UDP ) {
			setNonBlocking();
		} else {
			// Blocking TCP
			// Specify the maximum length of the connection queue
			if( listen( sock, MAX_SOCKET_QUEUE ) < 0 ) {
				perror("Socket::open - listen failed");
				::close(sock);

				return false;
			}
		}

		cout << "Duplex " YELLOW << hostname << ATTROFF
			<< ":" GREEN << port << ATTROFF
			<< ":" RED << (type == UDP ? "UDP" : "TCP") << ATTROFF
			<< " has been created\n";


	} else {
		cout <<  "Socket::open - unknown mode = " << mode << endl;
		return false;
	}

	// not sure if here, but for now
	// make time stamps
	tsStart.stamp();
	tsStop.stamp();

	return true;
}

//----------------------------------------------------------
// Read data from socket 
//	This is the main read function, all others should
//   only be an interface to this one function.
// 	If in CLIENT mode, then just read a block of data regardless
//	of the client number. If in SERVER mode, then read message
//	from the specified client, if it exsist.
//----------------------------------------------------------
int Socket::read(int client, char *buf, int length ) 
{
	int result = 0;
	Socket::SocketFD tempFD;
#ifdef VXWORKS
	int sockAddrSize = sizeof ( struct sockaddr_in );
#else
	socklen_t sockAddrSize = sizeof ( struct sockaddr_in );
#endif

	if ( sock == INVALID_SOCKET )
		return INVALID_SOCKET;

	//if( client != -1 && client >= clientList->len )
	if( client != -1 && client >= numClients )
		return INVALID_SOCKET;

	int clientInd = 0;
	if( client >= 0 )
		clientInd = client;
	else {
		clientInd = NUM_CLIENTS;
		client = NUM_CLIENTS;
	}

	//if ( mode >= SERVER && type == TCP && mode <= LISTENING_CLIENT ) {
	if( type == TCP && mode != CLIENT && mode != DUPLEX_CLIENT && mode != RECEIVING_CLIENT ) { // FIXME
		//if ( client < NUM_CLIENTS && clientFD[client] != INVALID_SOCKET)
		if ( clientFD[clientInd] != INVALID_SOCKET)
			tempFD = clientFD[clientInd];
		else 
			return INVALID_SOCKET;
	} else
		tempFD = sock;

	// use the recv interface for the sockets instead of the
	// standard read
	if( type == UDP) {
			cout << "UDP" << endl;
		if( mode == CLIENT ) {
			cout << " CLIENT" << endl;
			result = recvfrom(sock, (char *)buf, length, 0,
					(struct sockaddr *)&serverAddr, &sockAddrSize);
		} else {
			cout << " NON-CLIENT" << endl;
			result = recvfrom(sock, (char *)buf, length, 0,
					(struct sockaddr *)&clientAddr[NUM_CLIENTS], &sockAddrSize);
			if( result > 0 ) {
				client = getClientFromRead();
				if( client == -1 )
					printf("waiting to complete UDP connection: result=%i\n",result);
				else {
					clientInd = (client);
					printf("read from client=%i ind=%i result=%i addr=%s\n", client, clientInd, result, inet_ntoa(clientAddr[NUM_CLIENTS].sin_addr));
				}
			}
		}
	} else if (type == TCP ) {
		result = recv(tempFD, (char *)buf, length, 0);
	}

	if( result > 0) {
		totalBytes[0] += result;

		// record the time
		if( clientInd < NUM_CLIENTS )
			tsLast[clientInd].stamp();
	} else if (result < 0 && errno != EAGAIN)
		perror("Socket::read - recv");

	return result;
}

//----------------------------------------------------------
// Read data from socket 
//	if a server socket, read only from first client 	
//----------------------------------------------------------
int Socket::read( char *buf, int length ) 
{
	return Socket::read(-1, buf, length);
}

int Socket::write(in_addr address, const char *buf, const int length, bool noSeq ) 
{
	//SocketFD tempFD = -1;
	int nwritten = -1;

	if ( sock == INVALID_SOCKET )
		return INVALID_SOCKET;

	port = atoi( portStr.c_str() );
	//cout << "sending " << length << " bytes to " << inet_ntoa(address) << endl;

	struct sockaddr_in destAddr;
	memset(&destAddr,0,sizeof(destAddr));
	destAddr.sin_family      = AF_INET;
	destAddr.sin_addr        = address;
	destAddr.sin_port        = htons(port);

	//if(mode == CLIENT || mode == DUPLEX || mode == DUPLEX_CLIENT )
		//tempFD = sock;

	if( type == UDP) {
		if( mode == CLIENT  || mode == DUPLEX) {
			nwritten = sendto(sock, (char *)buf, length, 0,
					(struct sockaddr *)&destAddr, sizeof( struct sockaddr_in ));
			//cout << "clientAddr =" << inet_ntoa(serverAddr.sin_addr) << endl
			//<< "sock =" << sock << endl
			//<< "length =" << length << endl
			//<< "buf[0] =" << (int)buf[0] << endl
			//<< "nwritten =" << nwritten << endl;
		}
	}

	if( nwritten > 0)
		totalBytes[1] += nwritten;
	else if( nwritten < 0  && errno != EAGAIN)
		//perror("Socket::write - sendto");  //FIXME removed for BATMAN
		return 0;

	return nwritten;
}
//----------------------------------------------------------
// Write to socket 
//	This is the main write function, all others should
//   only be an interface to this one function.
// 	If in CLIENT mode, then just write a block of data regardless
//	of the client number. If in SERVER mode, then send message
//	to the specified client, if it exsist.
//----------------------------------------------------------
int Socket::write(int client, const char *buf, const int length, bool noSeq ) 
{
	SocketFD tempFD = -1;
	int nwritten = -1;

	if ( sock == INVALID_SOCKET )
		return INVALID_SOCKET;

	//if( client >= clientList->len )
	//if( client >= numClients )
	//return INVALID_SOCKET;

	//int clientInd =  *(int *)g_ptr_array_index( clientList, client);
	int clientInd = client;

	if( mode == SERVER || mode == LISTENING_CLIENT || mode == DUPLEX_SERVER) {
		if ( client < NUM_CLIENTS && clientFD[clientInd] != INVALID_SOCKET)
			tempFD = clientFD[clientInd];
		else
			return 0;
	} else if(mode == CLIENT || mode == DUPLEX_CLIENT )
		tempFD = sock;

	// use the sendto interface for the sockets 
	if( type == UDP) {
		if( mode == CLIENT ) {
			nwritten = sendto(sock, (char *)buf, length, 0,
					(struct sockaddr *)&serverAddr, sizeof( struct sockaddr_in ));
			//cout << "clientAddr =" << inet_ntoa(serverAddr.sin_addr) << endl
			//<< "sock =" << sock << endl
			//<< "length =" << length << endl
			//<< "buf[0] =" << (int)buf[0] << endl
			//<< "nwritten =" << nwritten << endl;
		} else {
			//cout << "clientAddr =" << inet_ntoa(clientAddr[client].sin_addr) << endl
			//<< "sock =" << sock << endl
			//<< "length =" << length << endl
			//<< "buf =" << buf << endl;
			nwritten = sendto(sock, (char *)buf, length, 0,
					(struct sockaddr *)&(clientAddr[clientInd]), sizeof( struct sockaddr_in ));
		}
	} else if (type == TCP ) {
		nwritten = send(tempFD, (char *)buf, length, 0);
	}

	if( nwritten > 0)
		totalBytes[1] += nwritten;

	return nwritten;
}

//----------------------------------------------------------
// write data to socket
// 	if i am a client, send to server
// 	if I am a server, send to all clients
//----------------------------------------------------------
int Socket::write( const char *buf, const int length, bool noSeq ) 
{
	int written=0;

	if ( mode == SERVER || mode == LISTENING_CLIENT || mode == DUPLEX_SERVER) {
		for(int client=0; client < numClients; client++) {
			written += Socket::write(client, buf, length, noSeq);
		}
	} else {
		written = Socket::write(0, (char *)buf, length, noSeq);
	}

	return written;
}

//----------------------------------------------------------
// close the port and any clients
//----------------------------------------------------------
bool Socket::close() 
{
	// close all client sockets
	for(int i=0; i < numClients; i++){
		(void)removeClient(i);
	}

	// close the socket
	if ( sock != INVALID_SOCKET ) {
		::close(sock);
		sock = INVALID_SOCKET;
	}

	connected = FALSE;

	// marking closing time
	tsStop.stamp();

	return true;
}

//----------------------------------------------------------
// Print out statistics about the socket port
//----------------------------------------------------------
void Socket::printStats()
{
	uint32_t usec;
	TimeStamp tsNow;
	tsNow.stamp();

	if( sock != INVALID_SOCKET ) {
		usec = tsNow - tsStart; //micro secs
	} else 
		usec = tsStop - tsStart; //micro secs

	cout << "Socket stats: " << endl << "--------------------" << endl
		<< "\tConnected = " << ( connected ? "TRUE" : "FALSE" )
		<< "\tUptime = " << usec/1000000.0 << " [s]" << endl
		<< "\tIn="   << totalBytes[0]  << " [bytes]"
		<< "\tRate=" << totalBytes[0]/(float)(usec/1000000.0)<<" [Bps] "<<endl
		<< "\tOut="  << totalBytes[1] << " [bytes]"
		<< "\tRate=" << totalBytes[1]/(float)(usec/1000000.0)<<" [Bps] "<<endl
		<< endl;
}


float Socket::rateTx()
{
	float rate;
	uint32_t usec;
	TimeStamp tsNow;

	tsNow.stamp();

	if( sock != INVALID_SOCKET ) {
		usec = tsNow - txRateTS; //micro secs
		rate = (totalBytes[1] - lastTXNum)/(float)(usec/1000000.0);
		txRateTS.stamp();
		lastTXNum = totalBytes[1];
	} else {
		txRateTS.stamp();
		usec = tsStop - tsStart; //micro secs
		rate = totalBytes[1]/(float)(usec/1000000.0);
	}

	return rate;
}


float Socket::rateRx()
{

	float rate;
	uint32_t usec;
	TimeStamp tsNow;

	tsNow.stamp();

	if( sock != INVALID_SOCKET ) {
		usec = tsNow - rxRateTS; //micro secs
		rate = (totalBytes[0] - lastRXNum)/(float)(usec/1000000.0);
		rxRateTS.stamp();
		lastRXNum = totalBytes[0];
	} else {
		rxRateTS.stamp();
		usec = tsStop - tsStart; //micro secs
		rate = totalBytes[0]/(float)(usec/1000000.0);
	}

	return rate;
}

//----------------------------------------------------------
//  Enable setNonBlockinging mode:
//  get the file flags and
//  set the O_NDELAY flag. (O_NDELAY==O_NONBLOCK)
//----------------------------------------------------------
bool Socket::setNonBlocking(SocketFD fd) 
{
	if( fd <= 0 )
		fd = sock;

	if ( fd == INVALID_SOCKET ) {
		return false;
	}

#ifdef VXWORKS
	char on = true;
	if(ioctl (fd, FIONBIO, on) == ERROR)
		return false;
#else
	int flags;

	flags = fcntl( fd, F_GETFL, 0 );
	if ( flags != -1 ) {
		if( fcntl( fd, F_SETFL, (flags | O_NDELAY)) == -1)
			return false;
	}
	else return false;
#endif
	is_blocking = false;
	return true;
}


//----------------------------------------------------------
// configure the socket as blocking
// get the file flag and
// clear the O_NDELAY flag. (O_NDELAY==O_NONBLOCK)
//----------------------------------------------------------
bool Socket::setBlocking(SocketFD fd) 
{
	if( fd == INVALID_SOCKET )
		fd = sock;

	if ( fd == INVALID_SOCKET ) {
		return false;
	}
#ifdef VXWORKS
	char on = false;
	if(ioctl (fd, FIONBIO, on) == ERROR)
		return false;
#else

	int flags;

	flags = fcntl( fd, F_GETFL, 0 );
	if ( flags != -1 ) {
		if( fcntl( fd, F_SETFL, (flags & ~O_NDELAY)) == -1)
			return false;
	}
	else return false;
#endif

	is_blocking = true;
	return true;
}

//----------------------------------------------------------
// connect client
// 	returns true/false if connected new client
//----------------------------------------------------------
int Socket::connectClient()
{
	pmesg(VERBOSE_STATUS, "Socket::connectClient()\n");
	int tempFD = -1;
	int client = -1;
	bool goodOldBuddy = 0;

#ifdef VXWORKS
	int len=0;
#else
	socklen_t len=0;
#endif
	len = sizeof( struct sockaddr_in );

	// see if the node has connected before, if tcp then we need to read
	// from the socket to get the client, if udp, then we already received the message
	// and already have the address
	if( type == TCP ) 
		tempFD = accept(sock,(struct sockaddr *)&clientAddr[NUM_CLIENTS], &len);

	client = getClientFromRead();

	if( client == -1 ) {

		if(numClients < NUM_CLIENTS) {
			client = 0;
			// find open clientInd
			while(client < NUM_CLIENTS && clientIndex[client] != -1)
				client++;

			if(client == NUM_CLIENTS){
				pmesg(VERBOSE_STATUS, "Could not find open client number\n");
				fastClose();
				return -1;
			} else{
				pmesg(VERBOSE_STATUS, "Attempting to connect client %u\n", client);
				clientIndex[client] = client;
			}
		} else{
			cout << "More than " << NUM_CLIENTS << " clients have tried to connect\n";
			fastClose();
			return -1;
		}
	} else
		goodOldBuddy = 1;

	if( type == UDP ) {
		// UDP doesn't get a new descriptor like TCP
		tempFD = 0; 
	}
	// but we need to set the clientAddr struct
	// which, clientAddr[NUM_CLIENTS] holds the value from the last connection
	memcpy( &(clientAddr[client]), &(clientAddr[NUM_CLIENTS]), sizeof( struct sockaddr_in)); 

	clientFD[client] = tempFD;

	if(!is_blocking)
		setNonBlocking(tempFD);

	if( tempFD != INVALID_SOCKET ) {
		connected = TRUE;
		//g_ptr_array_add( clientList, &(clientIndex[client]) );
		//if( ! goodOldBuddy ) numClients++;
		numClients++;

		if (! goodOldBuddy ) {
		  cout << "Connection with client id=" << client 
		  << " from " << inet_ntoa(clientAddr[client].sin_addr)
		  << " on " << ntohs(serverAddr.sin_port)
		  << (goodOldBuddy ? " ==> Old Buddy" : "" )
		  << endl;
	  }

		tsLast[client].stamp();

		pmesg( VERBOSE_STATUS, "Socket::Client connected ...\n");

		return client;
	} else 
		pmesg(VERBOSE_ERROR, "Socket::connectClient - could not accept client connection");

	return -1;
}

int Socket::getClientFromRead() const
{
	for( int i=0; i < NUM_CLIENTS; i++) {
		if( clientAddr[NUM_CLIENTS].sin_addr.s_addr == clientAddr[i].sin_addr.s_addr ) 
			return i;
	}
	return -1;
}

int Socket::removeClient(int client)
{	
	pmesg(VERBOSE_ERROR, "Socket::removeClient(%u)\n", client);
	if( numClients <= 0 && client >= numClients)
		return ERROR;

	int clientInd = -1;

	// search for a UDP client to remove
	if( client == -1 )
	{ 
		for( int i=0; i < NUM_CLIENTS; i++)
			if( clientAddr[NUM_CLIENTS].sin_addr.s_addr == clientAddr[i].sin_addr.s_addr ) {
				clientInd = i;

				// break loop
				break;
			}

		if( clientInd == -1 )
			return ERROR;
	} else {
		//clientInd =  *(int *)g_ptr_array_index( clientList, client);
		clientInd =  client;
	}

	// close file descriptor
	if( clientFD[clientInd] > 0 ) {
		::close(clientFD[clientInd]);
	}

	clientFD[clientInd] = INVALID_SOCKET;
	/* CORY THIS IS A BUG -- good, then i won't do it */
	numClients--;
	clientIndex[clientInd] = -1;
	//g_ptr_array_remove_range(clientList, client, 1);

	if( numClients == 0 )
	connected = FALSE;

	return client;
}

//----------------------------------------------------------
// abrubtly cancel a client connection
//----------------------------------------------------------
void Socket::fastClose()
{
	SocketFD tempFD;

	if( mode == SERVER || mode == LISTENING_CLIENT || mode == DUPLEX_SERVER ) {
		// accept the client and then close the connection so client
		// knows they can't connect
		tempFD = accept(sock, 0, 0);

		if(::write( tempFD, 0, sizeof(char) ))
			; // added if statement to avoid compiler warning
		::close(tempFD);
	}
}

//----------------------------------------------------------
// Set the file descriptor set with clients file descriptor
// to be used for select
//----------------------------------------------------------
int Socket::setFD(fd_set &set, bool is_tx)
{
	int maxFD=0;
	if(!is_tx) {
		if( sock > 0 ){
			FD_SET(sock,&set);		// add socket fd to set
			maxFD = sock;
		} else
			return ERROR;
	}

	if( type == TCP ) {
		for(int i=0; i < NUM_CLIENTS; i++){
			if( clientFD[i] >= 0 ) {
				FD_SET(clientFD[i],&set);
				maxFD = (maxFD < clientFD[i] ? clientFD[i] : maxFD);
			}
		}
	}
	return maxFD;
}	

//----------------------------------------------------------
// wait on the socket for data
//----------------------------------------------------------
Socket::SocketWait Socket::wait()
{
	return Socket::wait(-1);
}

//----------------------------------------------------------
// Wait on the socket for data, with a timeout
//----------------------------------------------------------
Socket::SocketWait Socket::wait(long int msec)
{
	int maxFD;				  // holds value of maximum fd
	fd_set readFD;			   // read fd set

	struct timeval tv;
	struct timeval *tvPtr=NULL;

	if (msec >= 0) {
		tv.tv_sec = int(msec / 1000);
		tv.tv_usec = (msec % 1000) * 1000;
		tvPtr = &tv;
	}

	// set file descriptor sets for select
	FD_ZERO(&readFD);		   // zero read fd set
	maxFD = setFD(readFD);

	// wait on selection
	int val=0;
	if( (val = select(maxFD + 1, &readFD, NULL,NULL,tvPtr)) <= 0){
		if( val == 0 ) {
			return WAIT_TIMEOUT;
		} else if( errno == EINTR ){
			return WAIT_INT;
		} else
			perror("select");

		return WAIT_ERROR;
	}

	return checkFD(readFD);
}

//----------------------------------------------------------
// see if a clients fd is set in the fd_set passed in
// and return a bit field set to the active descriptors
//----------------------------------------------------------
Socket::SocketWait Socket::checkFD(fd_set &set)
{
	int tempFD;
	int retVal = WAIT_DATA;

	if(sock < 0) return WAIT_ERROR;

	//
	// get input from listenFD
	//
	if(FD_ISSET(sock, &set)) {
		retVal = WAIT_PEER;
	}

	//
	// get input from clients
	//
	//if( mode >= SERVER && type == TCP && mode <= LISTENING_CLIENT ) {
	if( type == TCP && mode != CLIENT && mode != DUPLEX_CLIENT && mode != RECEIVING_CLIENT) { //FIXME -- not sure if this is correct
		//for(int client = 0; client < clientList->len; client++) {
		for(int client = 0; client < numClients; client++) {
			//int clientInd = *(int *)g_ptr_array_index(clientList, client);
			int clientInd = client;
			//if(clientIndex[client] != -1) {
			if( clientFD[clientInd] != INVALID_SOCKET) {
				tempFD = clientFD[clientInd];
				if(FD_ISSET(tempFD,&set)) {
					retVal += ( 1 << client );}
			}
		}
	}

	return (SocketWait)retVal;
}

//----------------------------------------------------------
// setoptions for the socket
//----------------------------------------------------------
bool Socket::setopt(bool broadcast)
{
	int optionVal = 1; // Turn ON the diff. setsockopt options 
	bool error = false;

	if( sock == INVALID_SOCKET )
		return false;

	// Specify the SO_REUSEADDR option  to bind a stream socket to a local  
	// port that may be still bound to another stream socket that may be 
	// hanging around with a "zombie" protocol control block whose context
	// is  not yet freed from previous sessions.
	if (setsockopt (sock, SOL_SOCKET, SO_REUSEADDR, (char *) &optionVal,
				sizeof(optionVal)) == ERROR)
	{
		perror ("Socket:: setsockopt SO_REUSEADDR failed");
		error = true;
	}

	// Specify the SO_KEEPALIVE option, and the transport protocol (TCP)  
	// initiates  a timer to detect a dead connection which prevents an
	// an application from  hanging on an invalid connection.
	if (setsockopt (sock, SOL_SOCKET, SO_KEEPALIVE, (char *) &optionVal,
				sizeof(optionVal)) == ERROR)
	{
		perror ("Socket:: setsockopt SO_KEEPALIVE failed");
		error = true;
	}

	// enable broadcast messages
	if (setsockopt (sock, SOL_SOCKET, SO_BROADCAST, (char *) &optionVal,
				sizeof(optionVal)) == ERROR)
	{
		perror ("Socket:: setsockopt SO_BROADCAST failed");
		error = true;
	}

	// Specify the TCP_NODELAY option for protocols such as X Window System 
	// Protocol that require immediate delivery of many small messages. 
	//
	// By default VxWorks uses congestion avoidance algorithm 
	// for virtual  terminal  protocols and  bulk  data  transfer  
	// protocols. When the TCP_NODELAY option is turned on and there are 
	// segments to be sent out, TCP  bypasses  the  congestion
	// avoidance  algorithm  and sends the segments out if there 
	// is enough space in the send window.
	if( type == TCP ) {
		if (setsockopt (sock, IPPROTO_TCP, TCP_NODELAY, (char *) &optionVal,
					sizeof(optionVal)) == ERROR)
		{
			perror ("Socket:: setsockopt TCP_NODELAY failed");
			error = true;
		}
	}

	if ( mode == SERVER || mode == LISTENING_CLIENT || mode == DUPLEX_SERVER ) {
		struct linger	 linger; 	// amount of time to SO_LINGER 
		linger.l_onoff = 1;  		// Turn ON the SO_LINGER option 
		linger.l_linger = 0;		// Use value of TCP_LINGERTIME in tcp_timer.h

		// Specify the SO_LINGER option to perform a  "graceful" close.
		// A graceful close occurs when a connection is shutdown, TCP  will  
		// try  to  make sure that all the unacknowledged data in transmission 
		// channel are acknowledged and the peer is shutdown properly by going 
		// through an elaborate  set  of  state transitions. 

		if (setsockopt (sock, SOL_SOCKET, SO_LINGER, (char *) &linger, 
					sizeof (linger)) == ERROR) 
		{
			perror ("Socket:: setsockopt SO_LINGER failed");
			error = true;
		}
	}

	return (! error);
}

//----------------------------------------------------------
// return up time in seconds
//----------------------------------------------------------
float Socket::getUpTime() const
{
	//if( !connected )
	//return -1.0;

	TimeStamp now;
	now.stamp();

	return (float)( (now - tsStart) / SEC2MICRO) ;
}

//----------------------------------------------------------
// return time last received a byte
//----------------------------------------------------------
float Socket::getLastTime(int client) const
{
	//if( client != -1 && client >= clientList->len )
	if( client != -1 && client >= numClients )
		return -1;

	int clientInd = NUM_CLIENTS;

	//if( client != -1 && client < clientList->len )
	if( client != -1 )
		clientInd = client;

	if ( clientInd <= NUM_CLIENTS && clientFD[clientInd] != INVALID_SOCKET)
	{
		if( tsLast[clientInd].time() == 0 )
			return -1.0;

		TimeStamp now;
		now.stamp();

		return (float)( (now - tsLast[clientInd]) / SEC2MICRO) ;
	} else
		return -1.0;
}

in_addr Socket::getServerIP() const
{
	/*if(serverAddr.sin_addr.s_addr == htonl(INADDR_ANY)) {
		char hostname[255];
		gethostname(hostname,255);
		struct hostent *hp = gethostbyname(hostname);
		if(hp != NULL)
		memcpy((void*)(&serverAddr.sin_addr), hp->h_addr_list[0], hp->h_length);
		}*/

	return serverAddr.sin_addr;
}
in_addr Socket::getClientIP(int client) const
{
	if( client >= numClients )
		return inet_makeaddr(0,0);

	if( client == -1 )
		return clientAddr[NUM_CLIENTS].sin_addr;
	else if( client < 0 )
		return inet_makeaddr(0,0);

	int clientInd = client;

	if( clientFD[clientInd] == INVALID_SOCKET) 
		return inet_makeaddr(0,0);

	return clientAddr[clientInd].sin_addr;
}


