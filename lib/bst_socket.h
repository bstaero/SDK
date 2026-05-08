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
#ifndef BST_SOCKET_H
#define BST_SOCKET_H

#include <sys/select.h>
#include <netinet/in.h>

#include "bst_interface.h"

#define MAX_IP_LENGTH    128
#define BST_MAX_CLIENTS  20
#define BST_INVALID_SOCKET (-1)

class BSTSocket : public BSTInterface {
	public:
		enum SocketMode  { SERVER, CLIENT };
		enum SocketType  { BST_TCP = 0, BST_UDP = 1 };
		enum SocketWait  { SWAIT_INT = -3, SWAIT_TIMEOUT = -2, SWAIT_ERROR = -1,
		                   SWAIT_DATA = 0, SWAIT_PEER = 0x8000 };

		BSTSocket();
		virtual ~BSTSocket();

		/* --- Standard CommunicationsInterface --- */
		bool initialize(const char * ip = "localhost",
		                const char * port = "55555",
		                const char * protocol = "UDP");
		bool open(void);

		/* Override BSTInterface read/write to support server mode */
		int16_t read(uint8_t * buf, uint16_t buf_size);
		int16_t write(uint8_t * buf, uint16_t size);

		/* --- Extended socket operations --- */

		/* Open in a specific mode (SERVER or CLIENT) */
		bool openAs(SocketMode mode);

		/* Close the socket and all client connections */
		bool close();

		/* Client mode: connect to server */
		bool connectHost();

		/* Server mode: accept a waiting client; returns client index or -1 */
		int connectClient();

		/* Server mode: remove a client by index; returns index or -1 */
		int removeClient(int client);

		/* Per-client I/O (server mode) */
		int readFrom(int client, char * buf, int length);
		int writeTo(int client, const char * buf, int length);

		/* Simple I/O (client mode, or char* convenience) */
		int readBytes(char * buf, int length);
		int writeBytes(const char * buf, int length);

		/* Select/fd_set integration */
		int setFD(fd_set & set, bool is_tx = false);
		SocketWait checkFD(fd_set & set);

		/* Blocking mode */
		bool setBlocking();
		bool setNonBlocking();

		/* Status queries */
		bool isClosed() const;
		bool isConnected();
		bool isConnected() const;
		int  getNumClients() const;
		SocketMode getMode() const;
		SocketType getSocketType() const;

		/* Byte counters */
		long bytesIn() const;
		long bytesOut() const;

		/* Client info */
		in_addr getClientIP(int client) const;
		in_addr getServerIP() const;

	private:
		char host[MAX_IP_LENGTH];
		int  port_num;
		SocketType  sock_type;
		SocketMode  socket_mode;

		int  server_fd;
		int  client_fds[BST_MAX_CLIENTS];
		struct sockaddr_in client_addrs[BST_MAX_CLIENTS];
		int  num_clients;
		int  next_read_client;

		struct sockaddr_in server_addr;
		struct sockaddr_in last_udp_sender;
		bool has_udp_sender;

		bool closed;
		long bytes_in_total;
		long bytes_out_total;

		/* Internal helpers */
		bool makeServerSocket();
		bool makeClientSocket();
		bool setNonBlockingFD(int sock_fd);
		bool setBlockingFD(int sock_fd);
		void closeAllClients();
		void acceptPendingClients();
};

#endif
