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
#include <assert.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#if !defined __APPLE__ && !defined __OPENWRT__
#include <error.h>
#endif
#include <errno.h>

#include "bst_socket.h"
#include "debug.h"

/* ------------------------------------------------------------------ */

BSTSocket::BSTSocket() : BSTInterface() {
	pmesg(VERBOSE_ALLOC, "BSTSocket::BSTSocket()\n");

	memset(host, 0, sizeof(host));
	port_num     = 0;
	sock_type    = BST_UDP;
	socket_mode  = CLIENT;
	server_fd    = BST_INVALID_SOCKET;
	num_clients  = 0;
	closed       = true;
	bytes_in_total  = 0;
	bytes_out_total = 0;

	memset(&server_addr, 0, sizeof(server_addr));
	memset(&last_udp_sender, 0, sizeof(last_udp_sender));
	has_udp_sender = false;

	for (int i = 0; i < BST_MAX_CLIENTS; i++) {
		client_fds[i] = BST_INVALID_SOCKET;
		memset(&client_addrs[i], 0, sizeof(client_addrs[i]));
	}
}

BSTSocket::~BSTSocket() {
	close();
}

/* ---- Standard CommunicationsInterface ---- */

bool BSTSocket::initialize(const char * ip, const char * port, const char * protocol) {
	strncpy(this->host, ip, MAX_IP_LENGTH);
	this->host[MAX_IP_LENGTH - 1] = '\0';
	this->port_num = atoi(port);

	if (protocol == NULL) return false;

	/* Parse mode and type from protocol string */
	if (strcmp(protocol, "UDP") == 0 || strcmp(protocol, "UDP:CLIENT") == 0) {
		sock_type   = BST_UDP;
		socket_mode = CLIENT;
		comm_type   = CommunicationsInterface::SOCKET_UDP;
	} else if (strcmp(protocol, "TCP:CLIENT") == 0) {
		sock_type   = BST_TCP;
		socket_mode = CLIENT;
		comm_type   = CommunicationsInterface::SOCKET_TCP;
	} else if (strcmp(protocol, "UDP:SERVER") == 0) {
		sock_type   = BST_UDP;
		socket_mode = SERVER;
		comm_type   = CommunicationsInterface::SOCKET_UDP;
	} else if (strcmp(protocol, "TCP:SERVER") == 0) {
		sock_type   = BST_TCP;
		socket_mode = SERVER;
		comm_type   = CommunicationsInterface::SOCKET_TCP;
	} else {
		pmesg(VERBOSE_ERROR, "BSTSocket::ERROR - invalid protocol: %s\n", protocol);
		return false;
	}

	CommunicationsInterface::initialize(ip, port, protocol);

	return openAs(socket_mode);
}

bool BSTSocket::open(void) {
	return openAs(socket_mode);
}

/* ---- Overridden read/write ---- */

int16_t BSTSocket::read(uint8_t * buf, uint16_t buf_size) {
	if (socket_mode == CLIENT) {
		/* Client mode: read from the single connection fd */
		if (!checkConnected()) return 0;

		int n = ::read(this->fd, buf, buf_size);
		if (n < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK) {
				pmesg(VERBOSE_ERROR, "BSTSocket::read error: %s\n", strerror(errno));
				close();
			}
			return 0;
		}
		if (n > 0) {
			rx_bytes += n;
			bytes_in_total += n;
		}
		return (int16_t)n;
	}

	/* Server mode */
	if (sock_type == BST_UDP) {
		/* UDP server: recvfrom on server_fd */
		if (server_fd == BST_INVALID_SOCKET) return 0;

		struct sockaddr_in from;
		socklen_t fromlen = sizeof(from);
		int n = ::recvfrom(server_fd, buf, buf_size, 0,
		                   (struct sockaddr *)&from, &fromlen);
		if (n < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK)
				pmesg(VERBOSE_ERROR, "BSTSocket::read UDP server error: %s\n", strerror(errno));
			return 0;
		}
		if (n > 0) {
			last_udp_sender = from;
			has_udp_sender = true;
			rx_bytes += n;
			bytes_in_total += n;
		}
		return (int16_t)n;
	}

	/* TCP server: accept pending connections, then read from clients */
	acceptPendingClients();

	for (int i = 0; i < num_clients; i++) {
		if (client_fds[i] == BST_INVALID_SOCKET) continue;
		int n = readFrom(i, (char *)buf, buf_size);
		if (n > 0) return (int16_t)n;
		if (n == 0) {
			/* Client disconnected */
			pmesg(VERBOSE_WARN, "BSTSocket::read: client %d disconnected\n", i);
			removeClient(i);
			i--;
		} else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			pmesg(VERBOSE_WARN, "BSTSocket::read: removing client %d: %s\n", i, strerror(errno));
			removeClient(i);
			i--;
		}
	}
	return 0;
}

int16_t BSTSocket::write(uint8_t * buf, uint16_t size) {
	if (socket_mode == CLIENT) {
		/* Client mode: write to the single connection fd */
		if (!checkConnected()) return 0;

		int n = ::write(this->fd, buf, size);
		if (n < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK)
				connected = false;
			return 0;
		}
		tx_bytes += n;
		bytes_out_total += n;
		return (int16_t)n;
	}

	/* Server mode */
	if (sock_type == BST_UDP) {
		/* UDP server: sendto last known sender */
		if (server_fd == BST_INVALID_SOCKET) return 0;
		if (!has_udp_sender) return 0;

		int n = ::sendto(server_fd, buf, size, 0,
		                 (struct sockaddr *)&last_udp_sender, sizeof(last_udp_sender));
		if (n < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK)
				pmesg(VERBOSE_ERROR, "BSTSocket::write UDP server error: %s\n", strerror(errno));
			return 0;
		}
		tx_bytes += n;
		bytes_out_total += n;
		return (int16_t)n;
	}

	/* TCP server: accept pending connections, then write to all connected clients */
	acceptPendingClients();
	int total = 0;
	for (int i = 0; i < num_clients; i++) {
		if (client_fds[i] == BST_INVALID_SOCKET) continue;
		int n = writeTo(i, (const char *)buf, size);
		if (n > 0) total = n;
	}
	return (int16_t)total;
}

/* ---- Extended operations ---- */

bool BSTSocket::openAs(SocketMode mode) {
	socket_mode = mode;

	if (mode == SERVER)
		return makeServerSocket();
	else
		return makeClientSocket();
}

bool BSTSocket::close() {
	closeAllClients();

	if (server_fd != BST_INVALID_SOCKET) {
		::close(server_fd);
		server_fd = BST_INVALID_SOCKET;
	}

	if (this->fd >= 0) {
		::close(this->fd);
		this->fd = -1;
	}

	connected = false;
	closed    = true;
	return true;
}

bool BSTSocket::connectHost() {
	if (socket_mode != CLIENT) return false;
	int local_fd = this->fd;
	if (local_fd < 0 || local_fd >= FD_SETSIZE) return false;

	if (::connect(local_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
		if (errno != EINPROGRESS) {
			pmesg(VERBOSE_ERROR, "BSTSocket::connectHost failed: %s\n", strerror(errno));
			return false;
		}

		/* Non-blocking connect: poll with select */
		fd_set wset;
		FD_ZERO(&wset);
		FD_SET(local_fd, &wset);

		struct timeval tv;
		tv.tv_sec  = 0;
		tv.tv_usec = 1000;

		int res = select(local_fd + 1, NULL, &wset, NULL, &tv);
		if (res <= 0) return false;

		int valopt;
		socklen_t lon = sizeof(int);
		if (getsockopt(local_fd, SOL_SOCKET, SO_ERROR, (void *)&valopt, &lon) < 0)
			return false;
		if (valopt) return false;
	}

	connected = true;
	closed    = false;
	return true;
}

int BSTSocket::connectClient() {
	if (socket_mode != SERVER) return -1;
	if (sock_type != BST_TCP) return -1;
	if (server_fd == BST_INVALID_SOCKET) return -1;
	if (num_clients >= BST_MAX_CLIENTS) return -1;

	struct sockaddr_in addr;
	socklen_t len = sizeof(addr);
	int new_fd = ::accept(server_fd, (struct sockaddr *)&addr, &len);
	if (new_fd < 0) return -1;

	setNonBlockingFD(new_fd);

	/* Disable Nagle for TCP clients */
	int flag = 1;
	setsockopt(new_fd, IPPROTO_TCP, TCP_NODELAY, (void *)&flag, sizeof(flag));

	/* Find a slot */
	int idx = num_clients;
	client_fds[idx]   = new_fd;
	client_addrs[idx] = addr;
	num_clients++;

	pmesg(VERBOSE_STATUS, "BSTSocket::connectClient: accepted client %d\n", idx);
	return idx;
}

int BSTSocket::removeClient(int client) {
	if (client < 0 || client >= num_clients) return -1;

	::close(client_fds[client]);

	/* Shift remaining clients down */
	for (int i = client; i < num_clients - 1; i++) {
		client_fds[i]   = client_fds[i + 1];
		client_addrs[i] = client_addrs[i + 1];
	}
	num_clients--;
	client_fds[num_clients]   = BST_INVALID_SOCKET;
	memset(&client_addrs[num_clients], 0, sizeof(client_addrs[num_clients]));

	return client;
}

/* ---- Per-client I/O ---- */

int BSTSocket::readFrom(int client, char * buf, int length) {
	if (client < 0 || client >= num_clients) return -1;
	if (client_fds[client] == BST_INVALID_SOCKET) return -1;

	int n = ::read(client_fds[client], buf, length);
	if (n > 0) {
		rx_bytes += n;
		bytes_in_total += n;
	}
	return n;
}

int BSTSocket::writeTo(int client, const char * buf, int length) {
	if (client < 0 || client >= num_clients) return -1;
	if (client_fds[client] == BST_INVALID_SOCKET) return -1;

	int n = ::write(client_fds[client], buf, length);
	if (n > 0) {
		tx_bytes += n;
		bytes_out_total += n;
	}
	return n;
}

int BSTSocket::readBytes(char * buf, int length) {
	if (socket_mode == CLIENT) {
		if (this->fd < 0) return -1;
		int n = ::read(this->fd, buf, length);
		if (n > 0) {
			rx_bytes += n;
			bytes_in_total += n;
		}
		return n;
	}
	return -1;
}

int BSTSocket::writeBytes(const char * buf, int length) {
	if (socket_mode == CLIENT) {
		if (this->fd < 0) return -1;
		int n = ::write(this->fd, buf, length);
		if (n > 0) {
			tx_bytes += n;
			bytes_out_total += n;
		}
		return n;
	}
	return -1;
}

/* ---- Select/fd_set integration ---- */

int BSTSocket::setFD(fd_set & set, bool is_tx) {
	int max_fd = -1;

	if (socket_mode == SERVER) {
		/* Add the listening socket for incoming connections */
		if (!is_tx && server_fd != BST_INVALID_SOCKET && server_fd < FD_SETSIZE) {
			FD_SET(server_fd, &set);
			if (server_fd > max_fd) max_fd = server_fd;
		}

		/* Add all connected client fds */
		for (int i = 0; i < num_clients; i++) {
			if (client_fds[i] != BST_INVALID_SOCKET && client_fds[i] < FD_SETSIZE) {
				FD_SET(client_fds[i], &set);
				if (client_fds[i] > max_fd) max_fd = client_fds[i];
			}
		}
	} else {
		/* Client mode: just the connection fd */
		if (this->fd >= 0 && this->fd < FD_SETSIZE) {
			FD_SET(this->fd, &set);
			if (this->fd > max_fd) max_fd = this->fd;
		}
	}

	return max_fd;
}

BSTSocket::SocketWait BSTSocket::checkFD(fd_set & set) {
	int result = 0;

	if (socket_mode == SERVER) {
		/* Check for incoming connections on the listening socket */
		if (server_fd != BST_INVALID_SOCKET && server_fd < FD_SETSIZE && FD_ISSET(server_fd, &set))
			result |= SWAIT_PEER;

		/* Check each client for data */
		for (int i = 0; i < num_clients; i++) {
			if (client_fds[i] != BST_INVALID_SOCKET && client_fds[i] < FD_SETSIZE && FD_ISSET(client_fds[i], &set))
				result |= (1 << i);
		}
	} else {
		if (this->fd >= 0 && this->fd < FD_SETSIZE && FD_ISSET(this->fd, &set))
			result = SWAIT_DATA;
	}

	return (SocketWait)result;
}

/* ---- Blocking mode ---- */

bool BSTSocket::setBlocking() {
	if (socket_mode == CLIENT && this->fd >= 0)
		return setBlockingFD(this->fd);

	/* Server: set all client fds to blocking */
	bool ok = true;
	for (int i = 0; i < num_clients; i++) {
		if (client_fds[i] != BST_INVALID_SOCKET)
			ok &= setBlockingFD(client_fds[i]);
	}
	return ok;
}

bool BSTSocket::setNonBlocking() {
	if (socket_mode == CLIENT && this->fd >= 0)
		return setNonBlockingFD(this->fd);

	/* Server: set all client fds to non-blocking */
	bool ok = true;
	for (int i = 0; i < num_clients; i++) {
		if (client_fds[i] != BST_INVALID_SOCKET)
			ok &= setNonBlockingFD(client_fds[i]);
	}
	return ok;
}

/* ---- Status queries ---- */

bool BSTSocket::isClosed() const { return closed; }
bool BSTSocket::isConnected() const { return connected; }
int  BSTSocket::getNumClients() const { return num_clients; }
BSTSocket::SocketMode BSTSocket::getMode() const { return socket_mode; }
BSTSocket::SocketType BSTSocket::getSocketType() const { return sock_type; }
long BSTSocket::bytesIn() const { return bytes_in_total; }
long BSTSocket::bytesOut() const { return bytes_out_total; }

in_addr BSTSocket::getClientIP(int client) const {
	in_addr addr;
	memset(&addr, 0, sizeof(addr));
	if (client >= 0 && client < num_clients)
		addr = client_addrs[client].sin_addr;
	return addr;
}

in_addr BSTSocket::getServerIP() const {
	return server_addr.sin_addr;
}

/* ---- Internal helpers ---- */

bool BSTSocket::makeServerSocket() {
	if (server_fd != BST_INVALID_SOCKET) {
		/* Already open */
		closed = false;
		connected = true;
		return true;
	}

	int type = (sock_type == BST_TCP) ? SOCK_STREAM : SOCK_DGRAM;
	server_fd = ::socket(AF_INET, type, 0);
	if (server_fd < 0) {
		pmesg(VERBOSE_ERROR, "BSTSocket::makeServerSocket: socket() failed: %s\n", strerror(errno));
		return false;
	}

	/* Allow address reuse */
	int opt = 1;
	setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family      = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port        = htons(port_num);

	if (::bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		pmesg(VERBOSE_ERROR, "BSTSocket::makeServerSocket: bind(%d) failed: %s\n",
		      port_num, strerror(errno));
		::close(server_fd);
		server_fd = BST_INVALID_SOCKET;
		return false;
	}

	if (sock_type == BST_TCP) {
		if (::listen(server_fd, BST_MAX_CLIENTS) < 0) {
			pmesg(VERBOSE_ERROR, "BSTSocket::makeServerSocket: listen() failed: %s\n", strerror(errno));
			::close(server_fd);
			server_fd = BST_INVALID_SOCKET;
			return false;
		}
	}

	setNonBlockingFD(server_fd);

	closed    = false;
	connected = true;

	pmesg(VERBOSE_STATUS, "BSTSocket: server listening on port %d (%s)\n",
	      port_num, (sock_type == BST_TCP ? "TCP" : "UDP"));

	return true;
}

bool BSTSocket::makeClientSocket() {
	/* Close any existing connection */
	if (this->fd >= 0) {
		::close(this->fd);
		this->fd = -1;
	}

	if (strnlen(host, MAX_IP_LENGTH) <= 1) {
		pmesg(VERBOSE_ERROR, "BSTSocket::ERROR - invalid host address\n");
		return false;
	}

	struct hostent * server = gethostbyname(host);
	if (server == NULL) {
		pmesg(VERBOSE_ERROR, "BSTSocket::ERROR: no such host %s:%d\n", host, port_num);
		return false;
	}

	int type = (sock_type == BST_TCP) ? SOCK_STREAM : SOCK_DGRAM;
	this->fd = ::socket(AF_INET, type, 0);
	if (this->fd < 0) {
		pmesg(VERBOSE_ERROR, "BSTSocket::ERROR - socket() failed: %s\n", strerror(errno));
		return false;
	}

	setNonBlockingFD(this->fd);

	if (sock_type == BST_TCP) {
		int flag = 1;
		setsockopt(this->fd, IPPROTO_TCP, TCP_NODELAY, (void *)&flag, sizeof(flag));
	}

	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	memcpy(&server_addr.sin_addr.s_addr, server->h_addr, server->h_length);
	server_addr.sin_port = htons(port_num);

	if (!connectHost()) {
		::close(this->fd);
		this->fd = -1;
		closed = true;
		return false;
	}

	closed = false;
	connected = true;

	pmesg(VERBOSE_STATUS, "BSTSocket: connected to %s:%d (%s)\n",
	      host, port_num, (sock_type == BST_TCP ? "TCP" : "UDP"));

	return true;
}

bool BSTSocket::setNonBlockingFD(int sock_fd) {
	int flags = fcntl(sock_fd, F_GETFL, 0);
	if (flags < 0) return false;
	return fcntl(sock_fd, F_SETFL, flags | O_NONBLOCK) >= 0;
}

bool BSTSocket::setBlockingFD(int sock_fd) {
	int flags = fcntl(sock_fd, F_GETFL, 0);
	if (flags < 0) return false;
	return fcntl(sock_fd, F_SETFL, flags & ~O_NONBLOCK) >= 0;
}

void BSTSocket::acceptPendingClients() {
	if (socket_mode != SERVER) return;
	if (server_fd == BST_INVALID_SOCKET) return;
	if (sock_type != BST_TCP) return;

	/* Non-blocking check for pending connections */
	while (num_clients < BST_MAX_CLIENTS) {
		struct sockaddr_in addr;
		socklen_t len = sizeof(addr);
		int new_fd = ::accept(server_fd, (struct sockaddr *)&addr, &len);
		if (new_fd < 0) break; /* EAGAIN / no more pending */

		setNonBlockingFD(new_fd);

		int flag = 1;
		setsockopt(new_fd, IPPROTO_TCP, TCP_NODELAY, (void *)&flag, sizeof(flag));

		client_fds[num_clients]   = new_fd;
		client_addrs[num_clients] = addr;
		num_clients++;

		pmesg(VERBOSE_STATUS, "BSTSocket::acceptPendingClients: accepted client %d\n", num_clients - 1);
	}
}

void BSTSocket::closeAllClients() {
	for (int i = 0; i < num_clients; i++) {
		if (client_fds[i] != BST_INVALID_SOCKET) {
			::close(client_fds[i]);
			client_fds[i] = BST_INVALID_SOCKET;
		}
	}
	num_clients = 0;
}
