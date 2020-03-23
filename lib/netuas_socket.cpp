#include <stdio.h>
#include <errno.h>

#include "netuas_socket.h"

NetuasSocket::NetuasSocket () : CommunicationsInterface(), last_connection(0,0) {
	sock_ptr = NULL;
	mode_type = Socket::SERVER;
}

NetuasSocket::~NetuasSocket () {
}

bool NetuasSocket::initialize(const char * hostname, const char * port, const char * type) {
	CommunicationsInterface::initialize(hostname,port,type);

	hostname ? host_str = hostname : host_str = "localhost";
	port ? port_str = port : port_str = "55555";
	type ? type_str = type : type = "TCP:SERVER";
	comm_type = CommunicationsInterface::SOCKET_TCP;

	if( sock_ptr != NULL ) {
		sock_ptr->close();
		delete sock_ptr;
		sock_ptr = NULL;
	}

	Socket::SocketType sock_type = Socket::TCP;
	if(strcmp(type_str.c_str(),"UDP:SERVER") == 0) {
		sock_type = Socket::UDP;
		comm_type = CommunicationsInterface::SOCKET_UDP;
		mode_type = Socket::SERVER;
	} else if(strcmp(type_str.c_str(),"UDP:CLIENT") == 0) {
		sock_type = Socket::UDP;
		comm_type = CommunicationsInterface::SOCKET_UDP;
		mode_type = Socket::CLIENT;
	} else if(strcmp(type_str.c_str(),"TCP:SERVER") == 0) {
		sock_type = Socket::TCP;
		comm_type = CommunicationsInterface::SOCKET_TCP;
		mode_type = Socket::SERVER;
	} else if(strcmp(type_str.c_str(),"TCP:CLIENT") == 0) {
		sock_type = Socket::TCP;
		comm_type = CommunicationsInterface::SOCKET_TCP;
		mode_type = Socket::CLIENT;
	} else {
		printf("NetuasSocket::ERROR -- invalid socket type requested\n");
		return false;
	}

	sock_ptr = new Socket(host_str.c_str(), port_str.c_str(), sock_type);
	if( sock_ptr == NULL ) {
		printf("NetuasSocket::ERROR -- could not create socket\n");
		return false;
	}

	sock_ptr->setNonBlocking();
	//return true;

	if( open() ) return true;

	return false;
}

bool NetuasSocket::checkFDActive(Socket::SocketWait * status, bool is_tx) {
	int maxFD, val;
	fd_set FDs;
	struct timeval timeout;

	/*<---Check Socket Conn--->*/
	if(!connected) return 0;
	if(sock_ptr == NULL) return 0;
	if(!open()) return 0;

	/*<--------Select FDs-------->*/
	FD_ZERO(&FDs);
	maxFD = sock_ptr->setFD(FDs);
	if(maxFD == ERROR) return false;

	timeout.tv_sec = SOCKET_TIME_OUT_SEC; timeout.tv_usec = SOCKET_TIME_OUT_USEC;

	if(is_tx) { 
		val = select(maxFD + 1, NULL, &FDs, NULL, &timeout);
	} else {
		val = select(maxFD + 1, &FDs, NULL, NULL, &timeout);
	}

	if( val < 0 ) {
		if (errno != EINTR)
			perror("NetuasSocket::select error");
	}
	else if(val == 0) {;}
	/*<----One FD is Active--->*/
	else {
		*status = sock_ptr->checkFD(FDs);

		/*<---Client is Waiting--->*/
		if(mode_type == Socket::SERVER) {
			if(*status & Socket::WAIT_PEER) {
				sock_ptr->connectClient();
				return false;
			}
		}

		return true;
	}

	return false;
}

uint16_t NetuasSocket::read(uint8_t * buf, uint16_t buf_size) {
	Socket::SocketWait status;
	int n = 0;
	int i = 0;
	static uint8_t last_client = 0;

	if( mode_type == Socket::CLIENT ) {
		return sock_ptr->read((char *)buf, buf_size);
	}

	if(checkFDActive(&status,false)) {

		if((status & ~Socket::WAIT_PEER) > 0 ) {

			/*<---Server Operations--->*/
			if((status & ~Socket::WAIT_DATA) > 0) {

				do {
					// change client
					i++;
					last_client = (last_client + 1) % sock_ptr->getNumClients();

					// is that client active
					if(status & 0x1 << last_client) { // check if FD is active

						// get date from the client
						n = sock_ptr->read(last_client, (char *)buf, buf_size);
						
						// data available
						if( n > 0) { 
							//printf("read %d bytes from socket client %d\n",n, last_client);
							return n;
						} else {
							if(n == 0)
								printf("NetuasSocket::read from client returned zero bytes\n");
							else if( errno != EWOULDBLOCK || errno != EINTR) 
								printf("NetuasSocket::bad read from client\n");

							if(sock_ptr->removeClient(last_client) != last_client) {
								printf("NetuasSocket::could not remove client\n");
							} else {
								printf("NetuasSocket::removed client\n");
							}

							// reset last_client
							last_client = sock_ptr->getNumClients();
						}
					}
				} while(i < sock_ptr->getNumClients());
			}
		}
	}

	return 0;
}


uint16_t NetuasSocket::write(uint8_t * buf, uint16_t buf_size) {
	Socket::SocketWait status;

	if( mode_type == Socket::CLIENT ) {
		return sock_ptr->write((char *)buf, buf_size);
	}


	if(checkFDActive(&status,true)) {
		int n = 0;
		bool success = true;

		for(int i = 0; i < sock_ptr->getNumClients(); i++) {
			if(status & 0x1 << i) { // check if FD is active
				n = sock_ptr->write(i,(char *)buf, buf_size);
				if( buf_size != n) { 
					success = false;
					if(n == 0)
						printf("NetuasSocket::write to client returned zero bytes\n");
					else if( errno != EWOULDBLOCK || errno != EINTR) 
						printf("NetuasSocket::bad write to client\n");
					if(sock_ptr->removeClient(i) != i) {
						printf("NetuasSocket::could not remove client\n");
					} else {
						printf("NetuasSocket::removed client\n");
					}
				} 
				//else printf("wrote %d bytes to socket client %d\n",n, i);
			}
		}

		if( success )
			return n;
	}

	return 0;
}

bool NetuasSocket::close() 
{
	if(sock_ptr != NULL) {
		printf("NetuasSocket::close()\n");
		sock_ptr->close();
		delete sock_ptr;
		sock_ptr = NULL;
	}
	connected = false;
	return true;
}

bool NetuasSocket::open() {
	if(sock_ptr == NULL) return false;

	if( sock_ptr->isClosed() ) {
		TimeStamp ts;
		ts.stamp();

		if(ts.diff(last_connection) < CONNECTION_TIMEOUT) return false;

		last_connection = ts;

		if ( !sock_ptr->open( this->mode_type ) ) {
			printf("NetuasSocket::Could not open sock\n");
			connected = false;
			return false;
		} 

		if( mode_type == Socket::CLIENT ) {
			if( !sock_ptr->connectHost() ) {
				//printf("Could not connect to host\n");
				sock_ptr->close();

				connected = false;
				return false;
			}
		}


	} 

	connected = true;
	return true;
}
