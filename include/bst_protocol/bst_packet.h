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
#ifndef _PACKET_H_
#define _PACKET_H_

#ifdef __cplusplus
  #include <stdint.h>
  #include <string.h>
  #include "structs.h"
#endif

/*! \class Packet
 *  \brief Class implementing the creation and decoding of internal packets
 *
 *  The internal communications packet structure consists of a two byte header, a byte to identify the packet type, a byte to identify the packet action, two bytes to identify packet data size, four bytes for the desination address, four bytes for the sender address, the data bytes, and a two byte Fletcher 16 checksum.  This is shown in Table \latexonly\ref{t:packet_structure}\endlatexonly.
 *  \latexonly
 *  \begin{table}
 *    \centering
 *    \caption{Internal packet structure.}
 *    \label{t:packet_structure}
 *    \begin{tabular}{|c|c|c|c|c|c|c|c|c|c|c|c|}
 *      \hline
 *        HDR0 & HDR1 & TYPE & ACTION & SIZE & TO & FROM & DATA0 & \dots & DATAn & CHKSUM0 & CHKSUM1 \\
 *      \hline
 *        U & \$ &&&&&&&&&& \\
 *      \hline
 *    \end{tabular}
 *  \end{table}
 *  \endlatexonly
 *
 *  The two header bytes are always a capital 'U' (0x55) followed by '$' (0x24).  The data struct contained within the packet is identified using a Packet_t enumerated type in the TYPE field, and a byte in the SIZE field to indicate it's size.  The relationship between the packet TYPE field and the struct contained in the DATA field has been detailed in Table \latexonly\ref{t:packet_data_structs}\endlatexonly.
 *
 *  \latexonly
 *  \begin{table}
 *    \centering
 *    \caption{Headers and associated data structs.}
 *    \label{t:packet_data_structs}
 *    \begin{tabular}{ll}
 *      \hline
 *        TYPE identifier (Packet\_t) & Data Type \\
 *      \hline
 *        SENSORS & Sensors\_t \\
 *        STATE & State\_t \\
 *        CONTROL & Control\_t \\
 *        COMMAND & Command\_t \\
 *        SURFACES & SurfaceSummary\_t \\
 *        LIMITS & Limits\_t \\
 *        CONTROL\_LOOPS & PID\_t \\
 *        FLIGHT\_PLAN & WayPt\_t \\
 *        DEL\_FLIGHT\_PLAN & int \\
 *        HEALTH\_AND\_STATUS & SystemStatus\_t \\
 *        GCS\_DATA & GCSData\_t \\
 *      \hline
 *    \end{tabular}
 *  \end{table}
 *  \endlatexonly
 *
 *  Finally, the last two bytes of the packet contain the Fletcher 16 checksum generated using all of the previous bytes including the header.
 */

/*! \brief Maximum size for a packet */
#define BST_MAX_PACKET_SIZE 128

/*! \brief Location of the first packet header byte */
#define PKT_HDR0   0
#define PKT_HDR1   1
#define PKT_TYPE   2
#define PKT_ACTION 3
#define PKT_SIZE   4
#define PKT_TO     6
#define PKT_FROM   10
#define PKT_DATA(ADdreSS)   (6 + 8*ADdreSS)

#define HEADER_SIZE(ADdreSS) (PKT_DATA(ADdreSS))
#define CHECKSUM_SIZE 2
#define OVERHEAD(ADdreSS) (HEADER_SIZE(ADdreSS) + CHECKSUM_SIZE)

typedef enum {
	PKT_ACTION_STATUS,
	PKT_ACTION_REQUEST,
	PKT_ACTION_COMMAND,
	PKT_ACTION_ACK,
	PKT_ACTION_NACK,
	PKT_ACTION_LOAD
} PacketAction_t;

typedef uint8_t Packet_t;

typedef enum {
	BUFF_INVALID,
	BUFF_NEED_MORE,
	BUFF_VALID=0x6 // set to min packet size
} BufferCheck_t;

static const uint32_t NO_ID            = 0x00000000;
static const uint32_t ONE_HOP          = 0x00FFFFFF;
static const uint32_t NODE_TYPE_MASK   = 0xFF000000;
static const uint32_t NODE_SERIAL_MASK = 0x00FFFFFF;

static const uint32_t ALL_UAVS    = 0x41FFFFFF;
static const uint32_t ALL_GCS     = 0x53FFFFFF;
static const uint32_t ALL_TABLETS = 0x54FFFFFF;
static const uint32_t ALL_NODES   = 0xFFFFFFFF;

static const uint32_t UAV_ID      = 0x41000000;
static const uint32_t GCS_ID      = 0x53000000;
static const uint32_t TABLET_ID   = 0x54000000;

uint16_t Packet_validateBuffer(const uint8_t * const data, uint16_t n);
uint16_t Packet_packetSize(const uint8_t * const data);
uint16_t Packet_dataSize(const uint8_t * const data);

#ifdef __cplusplus

class Packet {
	public:
		Packet();
		Packet(Packet_t type);

		~Packet();

		uint32_t getToAddress (void) const;
		void setToAddress(uint32_t id);
		bool isToID (uint32_t id) const;

		uint32_t getFromAddress (void) const;
		void setFromAddress(uint32_t id);
		bool isFromID (uint32_t id) const;

		uint16_t getSize (void) const;
		uint16_t getDataSize (void) const;
		void setSize(uint16_t size);

		Packet_t getType(void) const;
		void setType(Packet_t type);

		PacketAction_t getAction(void) const;
		void setAction(PacketAction_t action);

		void setData(uint8_t * data, uint16_t size);
		void getData(uint8_t * data);

		inline const uint8_t * getDataPtr() const {return packet + PKT_DATA(uses_address);}

		bool isValid(uint8_t data) ;
		bool isRequest(void) const;

		uint16_t validateBuffer(uint8_t *data, uint16_t n);
		bool setFromBuffer(uint8_t *data, uint16_t n);

		uint8_t * getPacket(void);

		void clear();

		bool operator == (const Packet& pkt) const;
		bool operator != (const Packet& pkt) const;

		inline void setAddressing(bool on_off) {uses_address = on_off;}
	private:
		void  initialize();
		uint8_t packet[BST_MAX_PACKET_SIZE];
		uint8_t ptr;

		bool big_endian;

		bool uses_address;

		void setCheckSum(void);

		bool checkFletcher16(uint8_t * data, uint16_t size);
		void setFletcher16 (uint8_t * data, uint16_t size);

		// used by isValid to reset buffer while checking for start of new packet
		void resetPacketBuffer();
};
#endif

#endif
