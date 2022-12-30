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

#ifndef __FLIGHT_PLAN_H_
#define __FLIGHT_PLAN_H_

#include <inttypes.h>
#include <string.h>
#include "vector.h"
#include "structs.h"
#include "helper_functions.h"

void lla2local(Vector * const v, const Waypoint_t * const wp, double lat, double lon, float alt);

void lla2local(Vector * const v, const Waypoint_t * const wp1, const Waypoint_t * const wp2);
void lla2local(Vector * const v, const Waypoint_t * const wp, double lat, double lon);
void lla2local(Vector * const v, double to_lat, double to_lon, double from_lat, double from_lon);
float lla2local(double to_lat, double to_lon, double from_lat, double from_lon);
void lla2local(float &x, float &y, double to_lat, double to_lon, double from_lat, double from_lon);
float lla2Bearing(const Waypoint_t * const wp1, const Waypoint_t * const wp2);
float lla2Bearing(const double lat, const double lon, const Waypoint_t * const wp2);
float lla2Bearing(const double f_lat, const double f_lon, const double t_lat, const double t_lon);

inline bool isValidInd(const int num) {
	return ( num != INVALID_WAYPOINT && num < MAX_WAYPOINTS );
}

inline bool waypointHasAction(uint16_t action, const Waypoint_t & wpt) {
	return ( (wpt.action & action) == action );
}

inline void setWaypointAction(uint16_t action, Waypoint_t &wpt) {
	wpt.action |= action;
}

inline void ClearMapBit(uint8_t *map, uint8_t ind) {
	if(map == NULL) return;
	map[ind/8] &= ~(0x01 << (ind % 8));
}

inline void SetMapBit(uint8_t *map, uint8_t ind) {
	if(map == NULL) return;
	map[ind/8] |=  (0x01 << (ind % 8));
}

inline bool CheckMapBit(uint8_t *map, uint8_t ind) {
	if(map == NULL) return false;
	return (map[ind/8] & (0x01 << (ind % 8))) != 0;
}

class FlightPlan {
	private:
		bool del(uint8_t number);

		FlightPlanMap_t fp_map; // full map of all waypoints
		FlightPlanMap_t curr_map; // map of current plan 

		Waypoint_t plan[MAX_WAYPOINTS];
		uint8_t i_wp, i_wpn;

		bool resetWaypoint(const int ind);
		void setCurrentPlanMap();

	public:
		FlightPlan(void);

		void reset();

		bool add(const Waypoint_t *  const points, uint8_t num_waypts);
		bool add(const FlightPlan * const fp);
		bool del(const FlightPlanMap_t * const map);

		bool load(const char * filename);
		uint8_t parse(const char * filename, Waypoint_t * waypts, uint8_t max_num_waypts);

		void clear(void);
//#ifdef VERBOSE
		void print(const char * msg);
		void printMap(void);
//#endif

		bool setCurrent(uint8_t number);
		bool setNext(void);
		void clearCurrentWaypoint() { reset(); }

		uint8_t getNum(void) const;

		const Waypoint_t * getCurrent(void) const;
		const Waypoint_t * getNext(void) const;
		const Waypoint_t * getWaypoint(uint8_t number) const;

		const Waypoint_t * getPrevious(void) const;
		const Waypoint_t * getPrevious(const uint8_t ind) const;
		const Waypoint_t * findFirst(const uint8_t ind) const;

		uint8_t getAll(Waypoint_t * const waypts) const;
		void getBounds(float * const bounds) const;
    uint8_t getClosest(double ac_lat, double ac_lon, uint8_t closest_point) const;

		inline const FlightPlanMap_t * getMap(void) const {return &fp_map;}
		inline void setMap(const FlightPlanMap_t * const map) {
			if(map == NULL) return;
			memcpy(&fp_map,map,sizeof(FlightPlanMap_t));
		}
		void getCurrentPlanMap(FlightPlanMap_t * const map) const;
		void getMissingWaypoints (FlightPlanMap_t * const map) const;

		void getMapContaining (uint8_t num, FlightPlanMap_t * map) const;

		bool closes() const;

		bool currentPlanContains(uint8_t num) const;
		bool currentPlanHasAction(uint16_t action) const;
		const Waypoint_t * currentPlanFindAction(uint16_t action) const;

		float getPathLength(uint8_t start_ind, uint16_t action) const;

		bool isHelix() const;
		bool isContinuous() const;

		//void removeCurrentPlan(FlightPlanMap_t * map);
};

#endif
