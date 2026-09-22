/*=+--+=#=+--         SwiftCore Flight Management Software        --+=#=+--+=#*\
|               Copyright (C) 2026 Black Swift Technologies LLC.               |
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
#include "tak_client.h"
#include "example_common.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <netdb.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define TAK_RECONNECT_PERIOD 5.0  // [s]
#define TAK_CONNECT_TIMEOUT  2    // [s]
#define TAK_MULTICAST_TTL    8

// macOS has no MSG_NOSIGNAL, SO_NOSIGPIPE is set on the socket instead
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

// CoT timestamps are ISO-8601 UTC, e.g. 2026-09-21T17:04:05.123Z
static void cotTime(char * buf, size_t len, double offset) {
	struct timeval tv;
	gettimeofday(&tv, NULL);

	double t = tv.tv_sec + tv.tv_usec * 1e-6 + offset;
	time_t sec = (time_t)t;
	int ms = (int)((t - sec) * 1000.0);

	struct tm utc;
	gmtime_r(&sec, &utc);

	char tmp[32];
	strftime(tmp, sizeof(tmp), "%Y-%m-%dT%H:%M:%S", &utc);
	snprintf(buf, len, "%s.%03dZ", tmp, ms);
}

static void xmlEscape(char * out, size_t len, const char * in) {
	size_t n = 0;
	out[0] = '\0';
	if(in == NULL) return;

	for(; *in && n + 7 < len; in++) {
		switch(*in) {
			case '&':  n += snprintf(out + n, len - n, "&amp;");  break;
			case '<':  n += snprintf(out + n, len - n, "&lt;");   break;
			case '>':  n += snprintf(out + n, len - n, "&gt;");   break;
			case '"':  n += snprintf(out + n, len - n, "&quot;"); break;
			case '\'': n += snprintf(out + n, len - n, "&apos;"); break;
			default:   out[n++] = *in; out[n] = '\0';            break;
		}
	}
}

TakClient::TakClient() {
	fd = -1;
	host[0] = '\0';
	port[0] = '\0';
	transport = TAK_TCP;
	last_connect_attempt = -TAK_RECONNECT_PERIOD;
}

TakClient::~TakClient() {
	close();
}

bool TakClient::open(const char * a_host, const char * a_port, TakTransport_t a_transport) {
	snprintf(host, sizeof(host), "%s", a_host);
	snprintf(port, sizeof(port), "%s", a_port);
	transport = a_transport;

	return connect();
}

void TakClient::close() {
	if(fd >= 0) ::close(fd);
	fd = -1;
}

bool TakClient::connect() {
	close();
	last_connect_attempt = getElapsedTime();

	struct addrinfo hints, * res;
	memset(&hints, 0, sizeof(hints));
	hints.ai_family   = AF_INET;
	hints.ai_socktype = (transport == TAK_TCP) ? SOCK_STREAM : SOCK_DGRAM;

	int err = getaddrinfo(host, port, &hints, &res);
	if(err != 0) {
		printf("TAK: cannot resolve %s:%s - %s\n", host, port, gai_strerror(err));
		return false;
	}

	fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
	if(fd < 0) {
		perror("TAK: socket");
		freeaddrinfo(res);
		return false;
	}

#ifdef SO_NOSIGPIPE
	int one = 1;
	setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif

	if(transport == TAK_UDP) {
		int ttl = TAK_MULTICAST_TTL;
		setsockopt(fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
	}

	// Non-blocking connect with a timeout so an unreachable server can't
	// stall the telemetry loop. For UDP this just sets the default peer.
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

	int rc = ::connect(fd, res->ai_addr, res->ai_addrlen);
	freeaddrinfo(res);

	if(rc < 0 && errno == EINPROGRESS) {
		fd_set wfds;
		FD_ZERO(&wfds);
		FD_SET(fd, &wfds);
		struct timeval tv = {TAK_CONNECT_TIMEOUT, 0};

		int so_error = ETIMEDOUT;
		socklen_t so_len = sizeof(so_error);
		if(select(fd + 1, NULL, &wfds, NULL, &tv) > 0)
			getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &so_len);

		rc = so_error == 0 ? 0 : -1;
		errno = so_error;
	}

	if(rc < 0) {
		printf("TAK: connect to %s:%s failed - %s\n", host, port, strerror(errno));
		close();
		return false;
	}

	printf("TAK: connected to %s:%s (%s)\n", host, port, transport == TAK_TCP ? "tcp" : "udp");
	return true;
}

bool TakClient::write(const char * buf, int len) {
	if(fd < 0) {
		if(getElapsedTime() - last_connect_attempt < TAK_RECONNECT_PERIOD) return false;
		if(!connect()) return false;
	}

	int sent = 0;
	while(sent < len) {
		int n = ::send(fd, buf + sent, len - sent, MSG_NOSIGNAL);
		if(n < 0) {
			if(errno == EAGAIN || errno == EWOULDBLOCK) {
				// socket buffer full, drop this update rather than block
				return false;
			}
			printf("TAK: send failed - %s\n", strerror(errno));
			close();
			return false;
		}
		sent += n;
	}

	return true;
}

bool TakClient::send(const TakTrack_t & track) {
	char now[32], stale[32];
	cotTime(now, sizeof(now), 0.0);
	cotTime(stale, sizeof(stale), track.stale);

	char callsign[64], remarks[256];
	xmlEscape(callsign, sizeof(callsign), track.callsign);
	xmlEscape(remarks, sizeof(remarks), track.remarks);

	char xml[1024];
	int len = snprintf(xml, sizeof(xml),
			"<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>"
			"<event version=\"2.0\" uid=\"%s\" type=\"%s\" how=\"m-g\" "
			"time=\"%s\" start=\"%s\" stale=\"%s\">"
			"<point lat=\"%.7f\" lon=\"%.7f\" hae=\"%.1f\" ce=\"10.0\" le=\"10.0\"/>"
			"<detail>"
			"<contact callsign=\"%s\"/>"
			"<track course=\"%.1f\" speed=\"%.1f\"/>"
			"<precisionlocation geopointsrc=\"GPS\" altsrc=\"GPS\"/>"
			"<remarks>%s</remarks>"
			"</detail>"
			"</event>",
			track.uid, track.type,
			now, now, stale,
			track.latitude, track.longitude, track.altitude,
			callsign,
			track.course, track.speed,
			remarks);

	if(len <= 0 || len >= (int)sizeof(xml)) return false;

	return write(xml, len);
}
