#include "flight_plan.h"

#include <stdio.h>
//#include "main.h"
#include "debug.h"
		
FlightPlan::FlightPlan() {

	pmesg(VERBOSE_ALLOC, "FlightPlan::FlightPlan()\n");

	i_wp  = INVALID_WAYPOINT; 
	i_wpn = INVALID_WAYPOINT;

	// make sure the waypoints are all initialized to invalid
	for(int i=0; i < MAX_WAYPOINTS; i++) {
		resetWaypoint(i);
	}
}

bool FlightPlan::resetWaypoint(const int ind) 
{
	if( !isValidInd(ind) )
		return false;

	plan[ind].num       = INVALID_WAYPOINT;
	plan[ind].next      = INVALID_WAYPOINT;
	plan[ind].latitude  = 0.0;
	plan[ind].longitude = 0.0;
	plan[ind].altitude  = 0.0;
	plan[ind].radius    = 0.0;
	plan[ind].action    = ACTION_NONE;

	// clear the bit from map
	BITCLEAR(fp_map.map, ind);
	BITCLEAR(curr_map.map, ind);

#ifdef VERBOSE
	//print("reset waypoint");
#endif

	return true;
}

void FlightPlan::reset() {
	i_wp  = INVALID_WAYPOINT; 
	i_wpn = INVALID_WAYPOINT;
}

bool FlightPlan::add(const Waypoint_t * const points, uint8_t num_waypts) {
	if(num_waypts > MAX_WAYPOINTS || points == NULL) 
		return false;

	for(uint8_t i=0; i<num_waypts; i++) {
		if( isValidInd( points[i].num ) ) {
			//pmesg(VERBOSE_INFO, "Waypoint action=%u\n", points[i].action);

			memcpy(&plan[points[i].num], &points[i], sizeof(Waypoint_t));

			// if we are modifying current point, change the next waypoint
			if(points[i].num == i_wp) {
				i_wpn = points[i].next;

				if( !isValidInd(i_wpn) ) 
					i_wpn = INVALID_WAYPOINT;
				else if( i_wpn == i_wp)
					i_wpn = INVALID_WAYPOINT;

				/*
					 if( !isValidInd(i_wpn) || !isValidInd(plan[i_wpn].num) )
					 i_wpn = INVALID_WAYPOINT;
					 */
			}

			// add to fp map
			BITSET(fp_map.map, points[i].num);
		}
	}

/*#ifdef VERBOSE
		print("add");
#endif*/

	return true;
}

bool FlightPlan::add(const FlightPlan * const fp) 
{
	if(fp == NULL) return false;
	if(!fp->closes()) return false;

	uint8_t fp_size = 0;

	for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {
		if(fp->getWaypoint(i) != NULL) {
			if(!this->add(fp->getWaypoint(i),1)) {
				return false;
			} else {
				fp_size++;
			}
		}
	}

#ifdef VERBOSE
	print("add fp");
#endif

	return true;
}

bool FlightPlan::del(uint8_t number) 
{
	if( !isValidInd( number ) ) 
		return false;
	
	bool retval = resetWaypoint(number);

/*#ifdef VERBOSE
	char out[32];
	sprintf(out,"del wpt %u",number);
	print(out);
#endif*/

	return retval;
}

bool FlightPlan::del(const FlightPlanMap_t * const map) 
{
	FlightPlanMap_t sent_map, temp_map;
	memcpy(&sent_map, map, sizeof(FlightPlanMap_t));

	// loop through and find the waypoints we were asked to delete
	for(uint8_t i=0; i<MAX_WAYPOINTS; i++){

		if( BITGET(sent_map.map, i) ) {

			getMapContaining(i, &temp_map);

			// now loop back through and make sure we delete complete flight plans that contain those points
			for(uint8_t j=0; j<MAX_WAYPOINTS; j++){
				if(BITGET(temp_map.map, j) ) { 
					if(!del(j)) {
						return false;
					} else {
						// No need to delete that point anymore if it's in the map
						BITCLEAR(sent_map.map, j);
					}
				}
			}

		}
	}

#ifdef VERBOSE
	print("delete");
#endif

	return true;
}

bool FlightPlan::load(const char * filename) {
	FILE * file;
	file = fopen (filename,"r");

	pmesg(VERBOSE_INFO, "opening %s\n\r",filename);

	Waypoint_t temp;
	unsigned int num, next, action;

	bool found_wp = false;
	char line [256];

	if (file!=NULL)
	{
		// clear all waypoints from plan
		clear();

		while(fgets(line,256,file) != NULL) {
			if(sscanf(line,"%u %u %lf %lf %f %f %u", &num, &next, &temp.latitude, &temp.longitude, &temp.altitude, &temp.radius, &action) == 7) {
				temp.num = num;
				temp.next = next;
				temp.action = action;

				if(temp.num < MAX_WAYPOINTS) {
					memcpy(&plan[temp.num], &temp, sizeof(Waypoint_t));
					fp_map.map[temp.num/8] |= 0x01 << (temp.num%8);
					if(!found_wp)
						found_wp = true;
				}
			}
		}

		fclose (file);

#ifdef VERBOSE
		print("load");
#endif
		return found_wp;
	} else {
		pmesg(VERBOSE_ERROR, "ERROR - Couldn't open file\n\r");
	}

	return false;
}

uint8_t FlightPlan::parse(const char * filename, Waypoint_t * waypts, uint8_t max_num_waypts) {

	FILE * file;
	file = fopen (filename,"r");

	pmesg(VERBOSE_INFO, "opening %s\n\r",filename);

	Waypoint_t temp;
	uint8_t plan_i = 0;
	unsigned int num, next, action;

	char line [256];

	if (file!=NULL)
	{
		// clear all waypoints from plan
		clear();

		while(fgets(line,256,file) != NULL) {
			if(sscanf(line,"%u %u %lf %lf %f %f %u", &num, &next, &temp.latitude, &temp.longitude, &temp.altitude, &temp.radius, &action) == 7) {
				temp.num = num;
				temp.next = next;
				temp.action = action;

				if(temp.num < max_num_waypts) {
					memcpy(&waypts[plan_i++],&temp,sizeof(Waypoint_t));
				}
			}
		}

		fclose (file);

#ifdef VERBOSE
		print("parse");
#endif
	} 
#ifdef VERBOSE
	else {
		pmesg(VERBOSE_ERROR,"Couldn't open file\n\r");
	}
#endif

	return plan_i;
}

bool FlightPlan::setCurrent(uint8_t number) {
	if( isValidInd(number) ) {

		// now check if the waypoint requested is valid
		if( !isValidInd(plan[number].num) ) {
			pmesg(VERBOSE_ERROR, "FlightPlan::setCurrent - invalid waypoint: number=%u plan[number].num=%u\n",
					number, plan[number].num);
			return false;
		}

		i_wp = number; 
		i_wpn = plan[i_wp].next;

		// now check that i_wpn is valid
		if( !isValidInd(i_wpn) || !isValidInd(plan[i_wpn].num) )
			i_wpn = INVALID_WAYPOINT;

		// check that next is not the same point
		else if( i_wpn == i_wp )
			i_wpn = INVALID_WAYPOINT;

		// update current plan map
		setCurrentPlanMap();

		return true;
	} 

	pmesg(VERBOSE_ERROR, "FlightPlan::setCurrent - invalid waypoint number=%u\n", number);

	return false;
}

bool FlightPlan::setNext(void) {
	return setCurrent(i_wpn);
}

const Waypoint_t * FlightPlan::getCurrent(void) const {
	if( !isValidInd(i_wp) || !isValidInd(plan[i_wp].num) )
		return NULL;

	return &plan[i_wp];
}

const Waypoint_t * FlightPlan::getNext(void) const {
	// check for bad index or bad waypoint
	if( !isValidInd(i_wpn) || !isValidInd(plan[i_wpn].num) )
		return NULL;

	// also check that the next index is not current index
	if( i_wp == i_wpn )
		return NULL;
	
	return &plan[i_wpn];
}

const Waypoint_t * FlightPlan::getPrevious(const uint8_t ind) const {
	if( !isValidInd(ind) )
		return NULL;

	for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {
		if( isValidInd( plan[i].num ) ) {
			if(plan[i].next == ind)
				return &plan[i];
		}
	}

	return NULL;
}

const Waypoint_t * FlightPlan::findFirst(const uint8_t ind) const {
	if( !isValidInd(ind) )
		return NULL;

	// get start waypoint
	const Waypoint_t * wpt = getWaypoint(ind);
	if( wpt == NULL ) return NULL;

	for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {

		// get previous
		const Waypoint_t * wpt_p = getPrevious(wpt->num);

		// make sure previous exists
		if( wpt_p == NULL )
			return wpt;

		// make sure previous is not ourselfs, or start ind
		if( wpt_p->num == ind )
			return wpt_p;

		// update wpt to previous
		wpt = wpt_p;

	}

	return NULL;

}

const Waypoint_t * FlightPlan::getPrevious(void) const {
	return this->getPrevious(i_wp);
}

const Waypoint_t * FlightPlan::getWaypoint(uint8_t number) const {
	if( !isValidInd(number) || !isValidInd(plan[number].num) )
		return NULL;

	return &plan[number];
}

void FlightPlan::clear(void) {
	for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {
		del(i);
	}
}

#ifdef VERBOSE
void FlightPlan::print(const char * msg) {
	pmesg(VERBOSE_FP,"\n-----[ %s ]-----\n",msg);
	printMap();

	for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {
		if(plan[i].num != INVALID_WAYPOINT) {
			pmesg(VERBOSE_FP,"%d %d %0.02f %0.02f %0.02f %0.02f 0x%x\n\r",
					(int)plan[i].num,
					(int)plan[i].next,
					plan[i].latitude,
					plan[i].longitude,
					plan[i].altitude,
					plan[i].radius,
					plan[i].action
					);
		}
	}

	pmesg(VERBOSE_FP, "current: %d next: %d\n\r",i_wp, i_wpn);
}

void FlightPlan::printMap(void) {
	for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {
		if(BITGET( fp_map.map, i) ) {
			pmesg(VERBOSE_FP,"o");
		} else {
			pmesg(VERBOSE_FP,".");
		}
	}
	pmesg(VERBOSE_FP,"\n");
}
#endif

uint8_t FlightPlan::getNum(void) const {
	uint8_t num_waypts = 0;
	for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {
		if( isValidInd( plan[i].num ) )
			num_waypts++;
	}

	return num_waypts;
}

uint8_t FlightPlan::getAll(Waypoint_t * const waypts) const {
	uint8_t plan_i = 0;

	// check for bad input
	if( waypts == NULL )
		return 0;

	for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {
		if( isValidInd(plan[i].num) ) {
			memcpy(&waypts[plan_i], &plan[i], sizeof(Waypoint_t));
			plan_i++;
		}
	}

	return plan_i;
}

void FlightPlan::getBounds(float * const bounds) const {
	bounds[0]=-91.0; bounds[1]=91.0; bounds[2]=-181.0; bounds[3]=181.0;

	for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {
		if( isValidInd(plan[i].num) ) {
			if (plan[i].latitude > bounds[0]) bounds[0] = plan[i].latitude;
			if (plan[i].latitude < bounds[1]) bounds[1] = plan[i].latitude;
			if (plan[i].longitude > bounds[2]) bounds[2] = plan[i].longitude;
			if (plan[i].longitude < bounds[3]) bounds[3] = plan[i].longitude;
		}
	}
}

uint8_t FlightPlan::getClosest(double ac_lat, double ac_lon, uint8_t closest_point) const {

  float min_range = 1e9;
  float tmp_range = 0.f;
  float c_ac_lat = cosf(ac_lat);

  for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {
    if(isValidInd(plan[i].num)) {
      if(currentPlanContains(plan[i].num)) {
        // L1-norm
        // TODO could make this an L2 norm on these two values and it will
        // match a range comparison, but L1 is cheaper.
        tmp_range = fabs(plan[i].latitude-ac_lat) + fabs(c_ac_lat*(plan[i].longitude-ac_lon));
        if(tmp_range < min_range) {
          min_range = tmp_range;
          closest_point = i;
        }
      }
    }
  }
  return closest_point;
}

void FlightPlan::getCurrentPlanMap(FlightPlanMap_t * const map) const {
	memcpy(map, &curr_map, sizeof(FlightPlanMap_t));
}

void FlightPlan::getMapContaining (uint8_t num, FlightPlanMap_t * map) const {
	//pmesg(VERBOSE_INFO, "FlightPlan::setCurrentPlanMap\n");

	// get current value
	uint8_t i = num;

	// clear current value
	for(uint8_t i=0; i<WAYPOINT_MAP_SIZE; i++)
		map->map[i] = 0;

	// if not asking for a valid point
	if( !isValidInd(num) )
		return;

	// if we don't have this point currently, return
	if( !isValidInd(plan[num].num) )
		return;

	// loop through plan, prevent infinite loop by using count
	uint16_t count = 0;
	do {
		// see if we looped back on ourselfs
		if( BITGET(map->map, i) )
			break;

		// set current value
		BITSET(map->map, i);

		// pmesg(VERBOSE_INFO, " %u", i);

		// get next ind
		i = plan[i].next;

		// validate next ind is valid and not me
		if( !isValidInd(i) )
			break;

		count++;
	} while(count < MAX_WAYPOINTS);

	// now go in previous direction
	count = 0;
	i = num;
	do {

		const Waypoint_t * wpt = this->getPrevious(i);
		if( wpt != NULL )
			i = wpt->num;

		// validate next ind is valid and not me
		if( !isValidInd(i) )
			break;

		// see if we looped back on ourselfs
		if( BITGET( map->map, i) )
			break;

		// set current value
		BITSET( map->map, i);

		// pmesg(VERBOSE_INFO, " %u", i);

		count++;
	} while( count < MAX_WAYPOINTS);

	//pmesg(VERBOSE_INFO, "\n");

}

void FlightPlan::setCurrentPlanMap() {
	pmesg(VERBOSE_INFO, "FlightPlan::setCurrentPlanMap (%u)\n", i_wp);

	getMapContaining(i_wp, &curr_map);
}

void FlightPlan::getMissingWaypoints (FlightPlanMap_t * map) const 
{
	for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {
		if( BITGET( fp_map.map, i) ) {
			if( !isValidInd(plan[i].num) ) {
				BITSET( map->map, i);
			}
		}
	}
}

// FIXME -- not implemented well, probably should make it so that an invalid radius is something very large
bool FlightPlan::closes() const 
{
	/*
	for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {
		if( isValidInd(plan[i].num) ) {
			// if it's number is OK and the waypoint it points to is OK, it should be good, so long as we don't have orphaned points
			// TODO -- need to deal with orphaned points
			if( isValidInd(plan[i].next) && !isValidInd(plan[plan[i].next].num) ) {
				return false;
			}

		}
	}
	*/

	// FIXME -- any reason to check this?
	return true;
}

bool FlightPlan::currentPlanContains(uint8_t num) const 
{
	if( !isValidInd(num) )
		return false;

	return BITGET(curr_map.map, num);
}

const Waypoint_t * FlightPlan::currentPlanFindAction(uint16_t action) const 
{
	for(uint8_t i=0; i<WAYPOINT_MAP_SIZE; i++) {
		if( curr_map.map[i] ) {
			for(uint8_t j=0; j < 8; j++) {
				if( (curr_map.map[i] & (0x01 << j)) != 0 ) {
					if( (plan[i*8 + j].action & action) != 0 )
						return &plan[i*8+j];
				}
			}
		}
	}

	return NULL;
}


bool FlightPlan::currentPlanHasAction(uint16_t action) const 
{
	for(uint8_t i=0; i<WAYPOINT_MAP_SIZE; i++) {
		if( curr_map.map[i] ) {
			for(uint8_t j=0; j < 8; j++) {
				if( (curr_map.map[i] & (0x01 << j)) != 0 ) {
					if( (plan[i*8 + j].action & action) != 0 )
						return true;
				}
			}
		}
	}

	return false;
}

/*
void FlightPlan::removeCurrentPlan(FlightPlanMap_t * map) 
{
	for(uint8_t i=0; i<WAYPOINT_MAP_SIZE; i++)
		map->map[i] &= (~curr_map.map[i]);
}
*/

float FlightPlan::getPathLength(uint8_t start_ind, uint16_t action ) const
{
	if( !isValidInd(start_ind) )
		return 0;	

	float d = 0;
	FlightPlanMap_t map;

	// prevent infintite while loop
	uint16_t count = 0;
	const Waypoint_t *wpt_c;
	const Waypoint_t *wpt_n;

	while(count < MAX_WAYPOINTS) {
		//  get waypoint at ind
		wpt_c = getWaypoint(start_ind);
		if( wpt_c == NULL ) return d;

		// set map
		BITSET( map.map, start_ind);

		// make sure the next index is not me and is valid
		if( isValidInd(wpt_c->next) && wpt_c->next != wpt_c->num ) {

			// get next waypoint
			wpt_n = getWaypoint(wpt_c->next);
			if( wpt_n == NULL ) return d;

			// now make sure the two waypoints have the flag
			if( action == ACTION_NONE || ((wpt_c->action & action) != 0 && (wpt_n->action & action) != 0) ) {
				// now get distance between the two
				d += lla2local(wpt_n->latitude, wpt_n->longitude, wpt_c->latitude, wpt_c->longitude); // FIXME -- this takes too much computational time
				//x = LAT_TO_M (wpt_c->latitude - wpt_n->latitude);
				//y = LON_TO_M (wpt_c->longitude - wpt_n->longitude, wpt_c->latitude);
				//d += sqrt( x*x+y*y);
			}

			// set next ind
			start_ind = wpt_c->next;

			// see if we have looped back on ourselfs
			if( BITGET( map.map, start_ind) )
				return d;

			// update count
			count++;
		} else
			return d;
	}

	return d;
}

// LLA2LOCAL - convert latitude, longitude, to x-y in a local frame
void lla2local(Vector *v, const Waypoint_t * wp, double lat, double lon, float alt) {
	v->operator()(2) = wp->altitude - alt;
	return lla2local(v, wp->latitude, wp->longitude, lat, lon);
}

void lla2local(Vector *v, const Waypoint_t * wp1, const Waypoint_t *wp2) {
	return lla2local(v, wp1->latitude, wp1->longitude, wp2->latitude, wp2->longitude);
}

void lla2local(Vector *v, const Waypoint_t * wp, double lat, double lon) {
	return lla2local(v, wp->latitude, wp->longitude, lat, lon);
}

// FIXME - need to encapsulate these into an object
double d_lat;
double d_lon;
double c_from_lat;
double s_from_lat;
double c_to_lat;
double s_to_lat;

float polar_r;
float polar_th;

float lla2local(double to_lat, double to_lon, double from_lat, double from_lon)
{
	Vector tmp_v;
	tmp_v.setSize(2);

	lla2local(&tmp_v, to_lat, to_lon, from_lat, from_lon);
	return sqrtf(tmp_v(0)*tmp_v(0) + tmp_v(1)*tmp_v(1));
}

void lla2local(Vector *local, double to_lat, double to_lon, double from_lat, double from_lon)
{
	// input condition check
	if( !local ) return;

	// do unit conversion [deg]->[rad]
	to_lat   *= DEG_TO_RADL;
	to_lon   *= DEG_TO_RADL;
	from_lat *= DEG_TO_RADL;
	from_lon *= DEG_TO_RADL;

	d_lat = (to_lat - from_lat);
	d_lon = (to_lon - from_lon);

	// unwrap longitude
	if( d_lon < -M_PI ) d_lon += 2*M_PI;
	else if( d_lon > M_PI ) d_lon -= 2*M_PI;

	// Compute the local position in polar coordinates
	c_from_lat = cos(from_lat);
	s_from_lat = sin(from_lat);
	c_to_lat   = cos(to_lat);
	s_to_lat   = sin(to_lat);

	polar_r = 6378137.f*(2*sqrtf( (d_lat*d_lat/4) + c_from_lat*c_to_lat*(d_lon*d_lon/4) ));
	polar_th = atan2f( sinf(d_lon)*c_to_lat, c_from_lat*s_to_lat - s_from_lat*c_to_lat*cosf(d_lon) );

	// Convert polar to cartesian
	local->operator()(0) = polar_r * cosf(polar_th);
	local->operator()(1) = polar_r * sinf(polar_th);
}


double lat1;
double lon1;
double lat2;
double lon2;
float y;
float x;

float lla2Bearing(const Waypoint_t *wp1, const Waypoint_t *wp2) 
{
	if( !wp1 || !wp2 ) return 0;

	lat1 = wp1->latitude * DEG_TO_RAD;
	lon1 = wp1->longitude * DEG_TO_RAD;

	lat2 = wp2->latitude * DEG_TO_RAD;
	lon2 = wp2->longitude * DEG_TO_RAD;

	d_lon = (lon2-lon1);

	y = sinf(d_lon) * cosf(lat2);
	x = cosf(lat1)*sinf(lat2) - sinf(lat1)*cosf(lat2)*cosf(d_lon);

	return atan2f(y, x);
}

float lla2Bearing(const double lat, const double lon, const Waypoint_t *wp2) 
{
	if( !wp2 ) return 0;

	lat1 = lat * DEG_TO_RADL;
	lon1 = lon * DEG_TO_RADL;

	lat2 = wp2->latitude * DEG_TO_RADL;
	lon2 = wp2->longitude * DEG_TO_RADL;

	d_lon = (lon2-lon1);

	y = sin(d_lon) * cos(lat2);
	x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(d_lon);

	return atan2f(y, x);
}

float lla2Bearing(const double f_lat, const double f_lon, const double t_lat, const double t_lon)
{
	lat1 = f_lat * DEG_TO_RADL;
	lon1 = f_lon * DEG_TO_RADL;

	lat2 = t_lat * DEG_TO_RADL;
	lon2 = t_lon * DEG_TO_RADL;

	d_lon = (lon2-lon1);

	y = sin(d_lon) * cos(lat2);
	x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(d_lon);

	return atan2f(y, x);
}
