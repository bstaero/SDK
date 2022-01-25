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

#ifndef _FLIGHT_PLAN_PROTOCOL_H_
#define _FLIGHT_PLAN_PROTOCOL_H_

#include "bst_module.h"

class BSTModuleFlightPlan : public BSTCommunicationsModule {
	public:
		BSTModuleFlightPlan();

		virtual void update();

		void send(uint8_t type, uint8_t * data, uint16_t size, const void * parameter);
		void sendCommand(uint8_t type, uint8_t * data, uint16_t size, const void * parameter);
		void request(uint8_t type, uint8_t parameter);

		void parse(uint8_t type, uint8_t action, uint8_t * data, uint16_t size);

		void setWaypointTimeout(float timeout);
		void setDefaultWaypointTimeout();


	private:

		int num_waypoints;
		int waypoint_i;

		typedef enum {
			WAITING,
			SENT_FP_MAP,
			SENDING_WAYPOINTS,
			WAITING_FOR_FINAL_MAP,
			WAITING_FOR_FINAL_MAP_RX,
			WAITING_FOR_WAYPOINTS,
			FINAL_ACK
		} FPSendState_t;

		Waypoint_t temp_wp;
		Waypoint_t rx_temp_plan[MAX_WAYPOINTS];
		Waypoint_t tx_temp_plan[MAX_WAYPOINTS];
		FlightPlanMap_t rx_fp_map;
		FlightPlanMap_t tx_fp_map;

		//FIXME -- implement this better, shouldn't need the exta memory, just use rx_temp_plan more intelligently
		Waypoint_t temp_plan[MAX_WAYPOINTS];

		FPSendState_t fp_send_state;
		float last_wpt_received;
		float last_waypoint_req;
		float last_fpmap_tx;
		uint8_t num_fpmap_tx;
		uint8_t num_fpmap_term_tx;
		uint8_t last_requested_waypoint;
		uint8_t last_requested_waypoint_count;
		float last_flight_plan_sent;
		float last_waypoint_sent;
		bool all_waypoints_received;
		bool requesting_missing_points;
		float waypoint_timeout;

		PacketAction_t send_action;

		bool haveAllWaypoints();
		void requestMissingWaypoints();
		void sendTermination();
		void finishSend(uint8_t type, uint8_t * data, uint16_t size, const void * parameter);

		void validateReceivedPlan();
		void reset();
};

#endif
