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
#ifndef _TAK_CLIENT_H_
#define _TAK_CLIENT_H_

#include <inttypes.h>

/* Minimal Cursor-on-Target (CoT) sender.
 *
 * TCP : streaming CoT input of a TAK Server (default plain-text port 8087)
 * UDP : unicast to a TAK Server UDP input, or multicast straight to ATAK
 *       clients on the LAN (SA multicast group 239.2.3.1:6969)
 *
 * TLS (port 8089) is not handled here -- put a TAK Server plain-text input
 * in front, or wrap the connection with stunnel. */

enum TakTransport_t {TAK_TCP, TAK_UDP};

typedef struct {
	const char * uid;       // unique, stable id for this track
	const char * callsign;  // label shown on the map
	const char * type;      // CoT type, e.g. "a-f-A-M-F-Q" (friendly UAV)
	double latitude;        // [deg]
	double longitude;       // [deg]
	double altitude;        // [m]
	double course;          // [deg true]
	double speed;           // [m/s]
	float  stale;           // [s] how long TAK should keep the marker
	const char * remarks;   // free text, may be NULL
} TakTrack_t;

class TakClient {
	public:
		TakClient();
		~TakClient();

		bool open(const char * host, const char * port, TakTransport_t transport);
		void close();

		bool isConnected() const { return fd >= 0; }

		// Build a CoT event for the track and send it. For TCP, a dropped
		// connection is re-established on a later call (rate-limited).
		bool send(const TakTrack_t & track);

	private:
		int fd;
		char host[64];
		char port[16];
		TakTransport_t transport;
		float last_connect_attempt;

		bool connect();
		bool write(const char * buf, int len);
};

#endif
