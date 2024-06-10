#include <stdio.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <stdlib.h>
#include <time.h>

#include "debug.h"

#include "bst_module_flight_plan.h"

//#define WAYPOINT_INTERVAL     0.05 // [s]
#define WAYPOINT_RX_TIMEOUT   2.0  // [s]
#define MAX_WAYPOINT_REQUEST  10

extern "C" {
  float getElapsedTime(); // defined elsewhere
}

BSTModuleFlightPlan::BSTModuleFlightPlan(char *n) : BSTCommunicationsModule () {

	pmesg(VERBOSE_ALLOC, "BSTModuleFlightPlan::BSTModuleFlightPlan()\n");
	strncpy(name, n, 8);

	max_num_data_types = 4;
	data_types = new DataType_t[4]; 

	registerDataType(FLIGHT_PLAN, 0, true, true);
	registerDataType(FLIGHT_PLAN_MAP, sizeof(FlightPlanMap_t), true, true);
	registerDataType(FLIGHT_PLAN_WAYPOINT, sizeof(Waypoint_t), true, true);
	registerDataType(DUBIN_PATH, sizeof(DubinsPath_t), false, true); 

	reset();

	waypoint_timeout = WAYPOINT_RX_TIMEOUT;

	send_action = PKT_ACTION_STATUS;
}

void BSTModuleFlightPlan::update() {

	// check receiving state
	if(fp_send_state == WAITING_FOR_WAYPOINTS || fp_send_state == WAITING_FOR_FINAL_MAP_RX) {

		// send termination if we have all waypoints
		if( fp_send_state == WAITING_FOR_FINAL_MAP_RX ) {
			sendTermination();
		} else {
			if( ! all_waypoints_received )
				requestMissingWaypoints();
			else
				validateReceivedPlan();
		}
	}

	// check sending state
	else if(fp_send_state != WAITING ) {
		finishSend(FLIGHT_PLAN,(uint8_t *)tx_temp_plan,0,&tx_fp_map);
	}

}

void BSTModuleFlightPlan::sendCommand(uint8_t type, uint8_t * data, uint16_t size, const void * parameter) {
	pmesg(VERBOSE_FP, "sendCommand - state=%u\n", fp_send_state);
	switch(type) {
		case FLIGHT_PLAN:
			send_action = PKT_ACTION_COMMAND;
			send(type,data,size,parameter);
			break;
		case FLIGHT_PLAN_MAP:
			BSTCommunicationsModule::sendCommand(type,data,size,NULL);
			break;
		case FLIGHT_PLAN_WAYPOINT:
			BSTCommunicationsModule::sendCommand(type,data,size,NULL);
			break;
	}
}

bool BSTModuleFlightPlan::haveAllWaypoints() {

	for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {
		if(BITGET( rx_fp_map.map, i) && rx_temp_plan[i].num == INVALID_WAYPOINT) {
			return false;
		}
	}

	return true;
}


void BSTModuleFlightPlan::requestMissingWaypoints() {
	//rx_temp_plan.getMissingWaypoints(&map);

	if(parent == NULL) return;

	if( !all_waypoints_received ) {

		// slow down how often we check
		float now = getElapsedTime();
		if (now - last_waypoint_req < waypoint_timeout)
			return;
				
		// pmesg(VERBOSE_FP, "requestMissingWaypoint\n");
		// printf("now=%f last_waypoint_req=%f last_wpt_received=%f waypoint_timeout=%f\n", now, last_waypoint_req, last_wpt_received, waypoint_timeout);

		for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {
			if( BITGET( rx_fp_map.map, i) && rx_temp_plan[i].num == INVALID_WAYPOINT) {
				if(last_requested_waypoint != i) {
					last_requested_waypoint_count = 0;
					last_requested_waypoint = i;
				}
				if(last_requested_waypoint_count++ > MAX_WAYPOINT_REQUEST) {
					last_requested_waypoint_count = 0;
					last_requested_waypoint = INVALID_WAYPOINT;

					pmesg(VERBOSE_WARN, "waypoint transmission exceeded, transmission failed %i\n",i);

/*#ifdef DEBUG
				switch(rx_fp_map.mode) {
					case NONE:
						pmesg(VERBOSE_FP,"NACK(FP_MAP[NONE]\n");
					case ADD:
						pmesg(VERBOSE_FP,"NACK(FP_MAP[ADD]\n");
					case DELETE:
						pmesg(VERBOSE_FP,"NACK(FP_MAP[DELETE]\n");
					case FINISH:
						pmesg(VERBOSE_FP,"NACK(FP_MAP[FINISH]\n");
				}
#endif*/
				parent->write(FLIGHT_PLAN_MAP,PKT_ACTION_NACK,(uint8_t *)&rx_fp_map,sizeof(FlightPlanMap_t),NULL);

					reset();

					return;
				}

				pmesg(VERBOSE_FP, "requesting waypoint %i\n",i);
				requesting_missing_points = true;

				parent->write(FLIGHT_PLAN_WAYPOINT,PKT_ACTION_REQUEST,(uint8_t *)&i,1,NULL);
				last_waypoint_req = getElapsedTime();

				return;
			}
		}

		pmesg(VERBOSE_FP, "no missing waypoints\n");

		// change state, we have received all 
		all_waypoints_received = true;
		requesting_missing_points = false;
		//last_flight_plan_sent = getElapsedTime() - waypoint_timeout + (waypoint_timeout/20.0);

	}

}

// check for valid closing plan
void BSTModuleFlightPlan::validateReceivedPlan() 
{
#if 0
	num_waypoints = 0;
	//FIXME -- replace with:   if(rx_temp_plan.closes()) {
	for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {
		if(rx_temp_plan[i].num != INVALID_WAYPOINT) {
			num_waypoints ++;
			if(rx_temp_plan[i].next == INVALID_WAYPOINT) {

				pmesg(VERBOSE_WARN, "waypoint does not close, transmission failed %i\n",i);

				parent->write(FLIGHT_PLAN_MAP,PKT_ACTION_NACK,(uint8_t *)&rx_fp_map,sizeof(FlightPlanMap_t),NULL);

				reset();

				return;
			}
		}
	}

	// checks passed, change state to final map hand shake
	fp_send_state = WAITING_FOR_FINAL_MAP_RX;

	pmesg(VERBOSE_FP, "%s FLIGHT_PLAN : FINAL MAP\n", name);

#else

	if(getElapsedTime() - last_flight_plan_sent > waypoint_timeout) {
		pmesg(VERBOSE_FP, "validateReceviedPlan fp_send_state=%u\n", fp_send_state);

		// make sure mode is ADD 
		rx_fp_map.mode = ADD;

		if(receiveCommand_function(FLIGHT_PLAN,(uint8_t *)rx_temp_plan,MAX_WAYPOINTS*sizeof(Waypoint_t),&rx_fp_map)) 
		{
			pmesg(VERBOSE_INFO, "Navigation accepted plan, sending ACK\n");
			parent->write(FLIGHT_PLAN_MAP,PKT_ACTION_ACK,(uint8_t *)&rx_fp_map,sizeof(FlightPlanMap_t),NULL);
		} else {
			pmesg(VERBOSE_WARN, "Navigation didn't accept plan, sending NACK\n");
			parent->write(FLIGHT_PLAN_MAP,PKT_ACTION_NACK,(uint8_t *)&rx_fp_map,sizeof(FlightPlanMap_t),NULL);
		}

		// checks passed, change state to final map hand shake
		fp_send_state = WAITING_FOR_FINAL_MAP_RX;

		pmesg(VERBOSE_FP, "%s FLIGHT_PLAN : FINAL MAP\n", name);

		last_fpmap_tx = 0;
		last_flight_plan_sent = getElapsedTime();
	}

#endif
}


// reset state variables
void BSTModuleFlightPlan::reset() 
{
	pmesg(VERBOSE_FP, "%s FLIGHT_PLAN : WAITING\n", name);

	fp_send_state = WAITING;

	num_waypoints = 0;
	waypoint_i = INVALID_WAYPOINT;

	last_fpmap_tx = 0;
	num_fpmap_tx = 0;
	num_fpmap_term_tx = 0;

	last_wpt_received = 0;
	last_waypoint_req = 0;
	last_waypoint_sent = 0;

	last_requested_waypoint = INVALID_WAYPOINT;
	last_requested_waypoint_count = 0;
	last_flight_plan_sent = 0;

	all_waypoints_received = false;
	requesting_missing_points = false;
}

void BSTModuleFlightPlan::sendTermination() {

	// check for failure
	if( num_fpmap_term_tx >= MAX_WAYPOINT_REQUEST ) {
		pmesg(VERBOSE_WARN, "FLIGHT_PLAN : Timed out on final FLIGHT_PLAN_MAP [TERM-ACK]\n");
		reset();

		return;
	}

	if( getElapsedTime() - last_fpmap_tx < waypoint_timeout ) 
		return;

	// Send termination
	rx_fp_map.mode = FINISH;
	parent->write(FLIGHT_PLAN_MAP,PKT_ACTION_ACK,(uint8_t *)&rx_fp_map,sizeof(FlightPlanMap_t),NULL);

	num_fpmap_term_tx++;
	last_fpmap_tx = getElapsedTime();

	pmesg(VERBOSE_FP, "%s <- FLIGHT_PLAN_MAP [TERM-ACK]\n", name);

}

void BSTModuleFlightPlan::send(uint8_t type, uint8_t * data, uint16_t size, const void * parameter) {

	if(parent == NULL) return;

	switch(type) {
		case FLIGHT_PLAN:
			switch(fp_send_state) {
				case WAITING :
					num_waypoints = 0;
					waypoint_i = INVALID_WAYPOINT;

					// fill out flightplan map
					if(parameter != NULL)
						memcpy(&tx_fp_map, (FlightPlanMap_t *)parameter, sizeof(FlightPlanMap_t));
					else {
						bzero((uint8_t *)&tx_fp_map,sizeof(FlightPlanMap_t));
					}

					// fill out the waypoints
					for(uint8_t i=0; i<MAX_WAYPOINTS; i++) {
						// set starting waypoint
						uint8_t index = ((Waypoint_t *)data)[i].num;

						if( index == INVALID_WAYPOINT ) break;

						// set starting waypoint
						if(waypoint_i == INVALID_WAYPOINT) waypoint_i = index;

						memcpy(&tx_temp_plan[index], &((Waypoint_t *)data)[i], sizeof(Waypoint_t));
						num_waypoints++;
					}

					pmesg(VERBOSE_INFO, "Sending %u waypoints\n",num_waypoints);

#ifdef VERBOSE
					switch(tx_fp_map.mode) {
						case NONE: pmesg(VERBOSE_FP, "%s <- FLIGHT_PLAN_MAP(NONE)\n", name); break;
						case ADD: pmesg(VERBOSE_FP, "%s <- FLIGHT_PLAN_MAP(ADD)\n", name); break;
						case DELETE: pmesg(VERBOSE_FP, "%s <- FLIGHT_PLAN_MAP(DEL)\n", name); break;
						case FINISH: pmesg(VERBOSE_FP, "<- FLIGHT_PLAN_MAP(FIN)\n"); break;
						default: pmesg(VERBOSE_ERROR, "<- INVALID FLIGHT_PLAN_MAP\n");
					}
#endif

					pmesg(VERBOSE_PACKETS, "FLIGHT PLAN MAP SIZE ---- %u\n", sizeof(_FlightPlanMap_t));

					// send flight plan map
					parent->write(FLIGHT_PLAN_MAP, send_action, (uint8_t *)&tx_fp_map, sizeof(FlightPlanMap_t), NULL);
					last_flight_plan_sent = getElapsedTime();

					fp_send_state = SENT_FP_MAP;

					pmesg(VERBOSE_FP,"%s SENT_FP_MAP\n", name);

					break;

				case SENT_FP_MAP:
				case SENDING_WAYPOINTS:
				case WAITING_FOR_FINAL_MAP:
					send_action = PKT_ACTION_STATUS;
					pmesg(VERBOSE_WARN, "Denied send request, still sending last plan\n");
					break;
				default: break;
			}
			break;

		case FLIGHT_PLAN_MAP:
		case FLIGHT_PLAN_WAYPOINT:
			break;
		case LAST_MAPPING_WAYPOINT:
			BSTCommunicationsModule::send(type,data,size,NULL);
			break;

		case DUBIN_PATH:
			BSTCommunicationsModule::send(type,data,size,NULL);
			break;
	}
}


void BSTModuleFlightPlan::finishSend(uint8_t type, uint8_t * data, uint16_t size, const void * parameter) {

	if(parent == NULL) return;

	int n;

	switch(type) {
		case FLIGHT_PLAN:
			switch(fp_send_state) {

				case SENT_FP_MAP:
					// check for failure
					if( num_fpmap_tx >= MAX_WAYPOINT_REQUEST ) {
						pmesg(VERBOSE_WARN, "FLIGHT_PLAN : Timed out on send FLIGHT_PLAN_MAP\n");
						reset();

						return;
					}

					if(getElapsedTime() - last_flight_plan_sent > waypoint_timeout) {
#ifdef VERBOSE
						switch(tx_fp_map.mode) {
							case NONE: pmesg(VERBOSE_FP,"%s <- FLIGHT_PLAN_MAP(NONE)\n", name); break;
							case ADD: pmesg(VERBOSE_FP,"%s <- FLIGHT_PLAN_MAP(ADD)\n", name); break;
							case DELETE: pmesg(VERBOSE_FP,"<- FLIGHT_PLAN_MAP(DEL)\n"); break;
							case FINISH: pmesg(VERBOSE_FP,"<- FLIGHT_PLAN_MAP(FINISH)\n"); break;
						}
#endif
						parent->write(FLIGHT_PLAN_MAP,send_action, (uint8_t *)&tx_fp_map, sizeof(FlightPlanMap_t), NULL);
						last_flight_plan_sent = getElapsedTime();

						num_fpmap_tx++;
					}
					break;

				case SENDING_WAYPOINTS:

					// validate state
					if( waypoint_i == INVALID_WAYPOINT ) {
						pmesg(VERBOSE_WARN, "INVALID STATE FOR SENDING FLIGHTPLAN\n");

						reset();
						return;
					}

					// FIXME -- should remove and let the hardware limit this
					if (getElapsedTime() - last_waypoint_sent >= (waypoint_timeout / 20.0))
					{
						pmesg(VERBOSE_FP,"%s <- FLIGHT_PLAN_WAYPOINT %u \n",name, waypoint_i);

						// send waypoint
						n = parent->write(FLIGHT_PLAN_WAYPOINT, send_action, (uint8_t *)(&tx_temp_plan[waypoint_i]), sizeof(Waypoint_t), NULL);

						last_waypoint_sent = getElapsedTime();

						if( n ) {

							pmesg(VERBOSE_FP,"WAPT: wrote=%u  size=%u\n", n, sizeof(Waypoint_t));

							// get next waypoint to send
							uint8_t last_waypoint = waypoint_i;
							for(uint8_t i=waypoint_i+1; i<MAX_WAYPOINTS; i++) {
								if(BITGET(tx_fp_map.map, i)) {
									waypoint_i = i;
									break;
								}
							}

							// if we didn't find another one, we are done sending
							if(waypoint_i == last_waypoint) {
								fp_send_state = WAITING_FOR_FINAL_MAP;
								//last_flight_plan_sent = getElapsedTime() - waypoint_timeout + (waypoint_timeout/20.0);

								last_flight_plan_sent = getElapsedTime();
								pmesg(VERBOSE_FP,"%s WAITING_FOR_FINAL_MAP\n", name);
							}
						}
					}
					break;

				case WAITING_FOR_FINAL_MAP:
					if(getElapsedTime() - last_flight_plan_sent > waypoint_timeout*MAX_WAYPOINT_REQUEST) {
						pmesg(VERBOSE_WARN,"FLIGH_PLAN : Final ACK timed out\n");
						reset();
					}
					break;

				default: 
					break;
			}
			break;

		default:
			break;
	}
}

void BSTModuleFlightPlan::request(uint8_t type, uint8_t parameter){
	if(parent == NULL) return;

	switch(type) {
		case FLIGHT_PLAN:
			if(fp_send_state == WAITING) {
				uint8_t i = 0;
				parent->write(FLIGHT_PLAN_MAP,PKT_ACTION_REQUEST,&i,1,NULL);
			}
			break;
		case FLIGHT_PLAN_WAYPOINT:
			parent->write(type,PKT_ACTION_REQUEST,&parameter,1,NULL);
			break;

		default:
			parent->write(type,PKT_ACTION_REQUEST,&parameter,1,NULL);
			break;
	}
}

void BSTModuleFlightPlan::parse(uint8_t type, uint8_t action, uint8_t * data, uint16_t size) {

	if(parent == NULL) return;

	//printf("BSTModuleFlightPlan::parse - type=%u\n", type);

	switch(type) {

		/* FLIGHT PLAN */
		case FLIGHT_PLAN_MAP:

			pmesg(VERBOSE_FP,"%s -> FLIGHT_PLAN_MAP\n", name);

			// we received a MAP, so we can update the next map
			last_flight_plan_sent = 0;

			// check for action of packet
			if(action != PKT_ACTION_REQUEST) {

				// validate size
				if(size != sizeof(FlightPlanMap_t)) break;

				// act on it
				switch(action) {
					case PKT_ACTION_COMMAND:

						if( fp_send_state != WAITING ) {
							pmesg(VERBOSE_ERROR,"FLIGHT PLAN MODULE IS NOT IN READY STATE - RESETTING\n");
							reset();
						}

						// copy to rx map
						pmesg(VERBOSE_FP,"FLIGHT_PLAN_MAP:PKT_ACTION_COMMAND - mode=%u\n", ((FlightPlanMap_t*)data)->mode);
						memcpy((uint8_t *)&rx_fp_map, data, sizeof(FlightPlanMap_t));

						switch(rx_fp_map.mode) {
							case ADD:
								pmesg(VERBOSE_FP,"   ADD_FLIGHT_PLAN\n");

								pmesg(VERBOSE_FP,"   <- PKT_ACTION_ACK\n");
								parent->write(type,PKT_ACTION_ACK,data,size,NULL);

								bzero(rx_temp_plan, sizeof(Waypoint_t) * MAX_WAYPOINTS);
								for(uint16_t i=0; i<MAX_WAYPOINTS; i++)
									rx_temp_plan[i].num = INVALID_WAYPOINT;

								last_waypoint_req  = last_wpt_received = getElapsedTime();
								fp_send_state = WAITING_FOR_WAYPOINTS;
								all_waypoints_received = false;

								pmesg(VERBOSE_FP,"   WAITING_FOR_WAYPOINTS\n");
								pmesg(VERBOSE_INFO,"Recieving new flight plan update\n");

								break;

							case DELETE:
								if(receiveCommand_function(FLIGHT_PLAN,NULL,0,&rx_fp_map))
									parent->write(type,PKT_ACTION_ACK,data,size,NULL);
								else
									parent->write(type,PKT_ACTION_NACK,data,size,NULL);
								break;

							case NONE:
								break;

							case FINISH:
								break;
						}

						break;

					case PKT_ACTION_ACK:

						// copy to rx map
						pmesg(VERBOSE_FP,"FLIGHT_PLAN_MAP:PKT_ACTION_ACK - mode=%u\n", ((FlightPlanMap_t*)data)->mode);
						memcpy((uint8_t *)&rx_fp_map, data, sizeof(FlightPlanMap_t));

						switch(rx_fp_map.mode) {
							case ADD:
								switch(fp_send_state) {
									case WAITING:
										break;

									case SENT_FP_MAP:
										//pmesg(VERBOSE_FP,"   SENT_FP_MAP\n");

										if(memcmp(&tx_fp_map,&rx_fp_map,sizeof(FlightPlanMap_t)) == 0) {
											if(num_waypoints == 0) {
												fp_send_state = WAITING_FOR_FINAL_MAP;

												pmesg(VERBOSE_FP,"   WAITING_FOR_FINAL_MAP\n");

												last_flight_plan_sent = getElapsedTime();
											} else {
												fp_send_state = SENDING_WAYPOINTS;
												pmesg(VERBOSE_FP,"   SENDING_WAYPOINTS\n");

												last_waypoint_sent = 0;
											}

										}
										break;

									case SENDING_WAYPOINTS:
										break;

									case WAITING_FOR_FINAL_MAP:
										//pmesg(VERBOSE_FP,"Got final FP MAP, transmission success\n");
										pmesg(VERBOSE_FP, "Flight plan successfully sent\n");

										//receiveReply_function(FLIGHT_PLAN,(uint8_t *)tx_temp_plan,sizeof(Waypoint_t) * num_waypoints,true,&rx_fp_map);

										fp_send_state = FINAL_ACK;

										break;

									case WAITING_FOR_WAYPOINTS:
									case WAITING_FOR_FINAL_MAP_RX:
									case FINAL_ACK:
										break;
								}
								break;

							case DELETE:

								pmesg(VERBOSE_FP,"   ACK DEL_FLIGHT_PLAN\n");
								//pmesg(VERBOSE_FP, "Got final FP MAP, transmission success\n");
								pmesg(VERBOSE_FP, "Flight plan successfully deleted\n");

								receiveReply_function(FLIGHT_PLAN,(uint8_t *)tx_temp_plan,sizeof(Waypoint_t) * num_waypoints,true,&rx_fp_map);

								reset();
								break;

							case FINISH:
								pmesg(VERBOSE_FP,"FLIGHT_PLAN : ACK - Got final FP MAP ACK, transmission success\n");
								pmesg(VERBOSE_INFO, "Flight plan update completed\n");


								switch( fp_send_state ) {
									case WAITING_FOR_FINAL_MAP:
										parent->write(type,PKT_ACTION_ACK,data,size,NULL);
										break;

									case FINAL_ACK:
										parent->write(type,PKT_ACTION_ACK,data,size,NULL);
										receive_function(FLIGHT_PLAN,(uint8_t *)rx_temp_plan,num_waypoints*sizeof(Waypoint_t),&rx_fp_map);
										break;

									default:
										break;
								}

								receiveReply_function(FLIGHT_PLAN,(uint8_t *)tx_temp_plan,sizeof(Waypoint_t) * num_waypoints,true,&rx_fp_map);

								reset();

								pmesg(VERBOSE_INFO, "%s fp_send_State=%u\n", name, fp_send_state);

								break;

							case NONE:
								switch(fp_send_state) {
									case WAITING:
										break;

									case SENT_FP_MAP:
										pmesg(VERBOSE_FP, "   SENT_FP_MAP\n");

										if(memcmp(&tx_fp_map,&rx_fp_map,sizeof(FlightPlanMap_t)) == 0) {
											if(num_waypoints == 0) {
												fp_send_state = WAITING_FOR_FINAL_MAP;

												last_flight_plan_sent = getElapsedTime();
											} else {
												fp_send_state = SENDING_WAYPOINTS;
												pmesg(VERBOSE_FP, "SENDING_WAYPOINTS\n");

												last_waypoint_sent = 0;
											}

										}
										break;
									case SENDING_WAYPOINTS:
										break;
									case WAITING_FOR_FINAL_MAP:
										//pmesg(VERBOSE_FP, "Got final FP MAP, transmission success\n");
										pmesg(VERBOSE_INFO, "Flight plan successfully updated\n");

										receiveReply_function(FLIGHT_PLAN,(uint8_t *)tx_temp_plan,sizeof(Waypoint_t) * num_waypoints,true,&rx_fp_map);

										// Send termination
										tx_fp_map.mode = FINISH;
										parent->write(FLIGHT_PLAN_MAP,PKT_ACTION_ACK,(uint8_t *)&tx_fp_map,sizeof(FlightPlanMap_t),NULL);


										reset();
										break;
									case WAITING_FOR_WAYPOINTS:
									case WAITING_FOR_FINAL_MAP_RX:
									case FINAL_ACK:
										break;
								}
								break;
						}
						break;
					case PKT_ACTION_NACK:
						// TODO
						//pmesg(VERBOSE_WARN,"FLIGHT_PLAN_MAP:PKT_ACTION_NACK - mode=%u\n", ((FlightPlanMap_t*)data)->mode);
						pmesg(VERBOSE_WARN,"Flight plan update was rejected (mode=%u)\n", ((FlightPlanMap_t*)data)->mode);
						break;

					case PKT_ACTION_STATUS:
						// copy to rx map
						pmesg(VERBOSE_FP,"FLIGHT_PLAN_MAP:PKT_ACTION_STATUS - mode=%u\n", ((FlightPlanMap_t*)data)->mode);
						memcpy((uint8_t *)&rx_fp_map, data, sizeof(FlightPlanMap_t));

						switch(rx_fp_map.mode) {
							case ADD:
							case DELETE:
							case FINISH:
								break;
							case NONE:
								bzero(rx_temp_plan,sizeof(Waypoint_t) * MAX_WAYPOINTS);
								for(uint16_t i=0; i<MAX_WAYPOINTS; i++)
									rx_temp_plan[i].num = INVALID_WAYPOINT;

								last_waypoint_req  = last_wpt_received = getElapsedTime();
								fp_send_state = WAITING_FOR_WAYPOINTS;

								parent->write(type,PKT_ACTION_ACK,data,size,NULL);
								break;
						}

						break;
				}
			} else {
				publish_function(FLIGHT_PLAN,data[0]);
			}
			break;

		case FLIGHT_PLAN_WAYPOINT:
			if(action != PKT_ACTION_REQUEST) {
				if(size != sizeof(Waypoint_t)) break;
				memcpy((uint8_t *)&temp_wp,data,sizeof(Waypoint_t));
				switch(action) {
					case PKT_ACTION_STATUS:
					case PKT_ACTION_COMMAND:
						if(fp_send_state == WAITING_FOR_WAYPOINTS) {
							if(temp_wp.num < MAX_WAYPOINTS) {

								memcpy(&rx_temp_plan[temp_wp.num],&temp_wp,sizeof(Waypoint_t));
								last_waypoint_req = last_wpt_received = getElapsedTime();

								// FIXME - put back in and remove below when this works well with low rate comms
								//parent->write(type,PKT_ACTION_ACK,data,size,NULL);

								if(last_requested_waypoint == temp_wp.num) {
									pmesg(VERBOSE_FP,"GOT MISSING WAYPOINT\n");
									last_waypoint_req = getElapsedTime() - waypoint_timeout + waypoint_timeout/20.0;
								// FIXME -remove here for above
									parent->write(type,PKT_ACTION_ACK,data,size,NULL);
								}

								// check to see if we received all waypoints
								if( !all_waypoints_received && haveAllWaypoints() ) {
									pmesg(VERBOSE_FP, "HAVE ALL WAYPOINTS\n");
									all_waypoints_received = true;
									requesting_missing_points = false;
									//last_flight_plan_sent = getElapsedTime() - waypoint_timeout + (waypoint_timeout/20.0);
									last_flight_plan_sent = 0;
									validateReceivedPlan();
								}
							}
						}
						break;
					case PKT_ACTION_ACK: // FIXME -- put in for reading from arbiter eeprom
						last_waypoint_sent = getElapsedTime() - (waypoint_timeout/20.0);
						if(last_waypoint_sent < 0.0) last_waypoint_sent = 0.0;

						pmesg(VERBOSE_FP,"-> FLIGHT_PLAN_WAYPOINT ACK %u\n",((Waypoint_t *)data)->num);

						//receiveReply_function(type,data,size,true,NULL);

						break;
					case PKT_ACTION_NACK: // FIXME -- put in for reading from arbiter eeprom
						//receiveReply_function(type,data,size,false,NULL);
						break;
				}
			} else {
				if(fp_send_state == WAITING_FOR_FINAL_MAP) {
					if(data[0] < MAX_WAYPOINTS) {

						BSTCommunicationsModule::send(FLIGHT_PLAN_WAYPOINT,(uint8_t *)(&tx_temp_plan[data[0]]),sizeof(Waypoint_t),NULL);

						//last_flight_plan_sent = getElapsedTime() - waypoint_timeout + (waypoint_timeout/20.0);

					} else {
						pmesg(VERBOSE_WARN,"Invalid waypoint %u from request\n",data[0]);
					}
				}

			}

			break;

		case DUBIN_PATH:
			BSTCommunicationsModule::parse(type, action, data, size );
			break;

		case INVALID_PACKET:
		default:
			// should not receive these
			pmesg( VERBOSE_ERROR, "INVALID_PACKET\n");
			break;
	}
}

void BSTModuleFlightPlan::setWaypointTimeout(float timeout) {
	pmesg( VERBOSE_INFO, "Changing waypoint timeout=%f\n", timeout);
	waypoint_timeout = timeout;
}

void BSTModuleFlightPlan::setDefaultWaypointTimeout(void) {
	waypoint_timeout = WAYPOINT_RX_TIMEOUT;
}

float BSTModuleFlightPlan::getDefaultWaypointTimeout(void) {
	return WAYPOINT_RX_TIMEOUT;
}
