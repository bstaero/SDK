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
//
// Terminal "bus monitor" for the BST protocol.  See monitor.h for the API.
//
// Layout (UAVCAN-GUI-Tool inspired, but in the terminal):
//
//   header   : link, uptime, aggregate packet rate, paused flag
//   table    : one row per packet type seen -> id, name, rate, count, bytes, age
//   detail   : decoded fields (known telemetry) or hex+ascii of the selection
//   footer   : key bindings
//
// Rendering is done with plain ANSI escapes (no ncurses dependency): the cursor
// is homed each frame, every line is cleared to EOL, and the area below the
// last line is cleared, so the frame redraws in place without flicker.
//
#include "monitor.h"

#include "example_common.h"   // running, getElapsedTime(), inputAvailable()
#include "structs.h"          // PacketTypes_t + telemetry layouts (+ using bst::comms)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <math.h>
#include <sys/ioctl.h>

/* ------------------------------------------------------------------ config */

#define NUM_TYPES       256
#define MAX_CAPTURE     256     // bytes of the most recent payload kept per type
#define DETAIL_MAX      40      // max decoded lines a packet can produce

static const float RATE_WINDOW   = 1.0f;   // [s] window used to compute Hz
static const float RENDER_PERIOD = 0.1f;   // [s] minimum time between redraws

/* ------------------------------------------------------------------- state */

typedef struct {
	uint32_t total;                 // packets seen since start / last clear
	uint32_t win_count;             // packets seen in the current rate window
	float    hz;                    // most recently computed rate
	uint16_t last_size;             // payload size of the most recent packet
	uint16_t cap_size;              // bytes actually captured (<= MAX_CAPTURE)
	uint8_t  last_data[MAX_CAPTURE];
	float    last_time;             // getElapsedTime() of the most recent packet
} TypeStat;

static TypeStat stats[NUM_TYPES];

static char  conn_str[80] = "";
static float win_start    = 0.0f;
static float last_render  = -1.0f;
static bool  force_render = false;

static int   sel        = 0;        // selection index into the visible list
static int   table_top  = 0;        // first visible-list index shown (scroll)
static bool  hex_mode   = false;    // force hex view even when a decoder exists
static bool  paused     = false;

static int   vis_count  = 0;        // visible types last render (for clamping)
static int   term_rows  = 24;
static int   term_cols  = 80;

/* ------------------------------------------------------------- packet names */

static const char * packetName(uint8_t type)
{
	switch(type) {
		case SENSORS_HUMIDITY:          return "SENSORS_HUMIDITY";
		case SENSORS_GPS:               return "SENSORS_GPS";
		case SENSORS_ACCELEROMETER:     return "SENSORS_ACCELEROMETER";
		case SENSORS_GYROSCOPE:         return "SENSORS_GYROSCOPE";
		case SENSORS_MAGNETOMETER:      return "SENSORS_MAGNETOMETER";
		case SENSORS_IMU:               return "SENSORS_IMU";
		case SENSORS_DYNAMIC_PRESSURE:  return "SENSORS_DYNAMIC_PRESSURE";
		case SENSORS_STATIC_PRESSURE:   return "SENSORS_STATIC_PRESSURE";
		case SENSORS_AIR_TEMPERATURE:   return "SENSORS_AIR_TEMPERATURE";
		case SENSORS_AGL:               return "SENSORS_AGL";
		case SENSORS_CALIBRATE:         return "SENSORS_CALIBRATE";
		case SENSORS_BOARD_ORIENTATION: return "SENSORS_BOARD_ORIENTATION";
		case SENSORS_GNSS_ORIENTATION:  return "SENSORS_GNSS_ORIENTATION";
		case SENSORS_MHP:               return "SENSORS_MHP";
		case SENSORS_GNSS_RTCM:         return "SENSORS_GNSS_RTCM";
		case SENSORS_MHP_SENSORS:       return "SENSORS_MHP_SENSORS";
		case SENSORS_MHP_9H_SENSORS:    return "SENSORS_MHP_9H_SENSORS";
		case SENSORS_MHP_9H_TIMING:     return "SENSORS_MHP_9H_TIMING";
		case SENSORS_DYNP_CALIBRATION:  return "SENSORS_DYNP_CALIBRATION";
		case SENSORS_GYRO_CALIBRATION:  return "SENSORS_GYRO_CALIBRATION";
		case SENSORS_MAG_CALIBRATION:   return "SENSORS_MAG_CALIBRATION";
		case SENSORS_MAG_CURRENT_CAL:   return "SENSORS_MAG_CURRENT_CAL";
		case SENSORS_ADSB:              return "SENSORS_ADSB";
		case SENSORS_MHP_GNSS:          return "SENSORS_MHP_GNSS";
		case SENSORS_MHP_TIMING:        return "SENSORS_MHP_TIMING";
		case SENSORS_PROXIMITY:         return "SENSORS_PROXIMITY";
		case SENSORS_RTK_HEADING:       return "SENSORS_RTK_HEADING";
		case STATE_STATE:               return "STATE_STATE";
		case STATE_ESTIMATOR_PARAM:     return "STATE_ESTIMATOR_PARAM";
		case STATE_FLIGHT_CONTROLLER:   return "STATE_FLIGHT_CONTROLLER";
		case CONTROL_COMMAND:           return "CONTROL_COMMAND";
		case CONTROL_PID:               return "CONTROL_PID";
		case CONTROL_FLIGHT_PARAMS:     return "CONTROL_FLIGHT_PARAMS";
		case CONTROL_FILTER_PARAMS:     return "CONTROL_FILTER_PARAMS";
		case ACTUATORS_VALUES:          return "ACTUATORS_VALUES";
		case ACTUATORS_CALIBRATION:     return "ACTUATORS_CALIBRATION";
		case ACTUATORS_ROTOR_PARAMS:    return "ACTUATORS_ROTOR_PARAMS";
		case ACTUATORS_MIXING_PARAMS:   return "ACTUATORS_MIXING_PARAMS";
		case HANDSET_VALUES:            return "HANDSET_VALUES";
		case HANDSET_CALIBRATION:       return "HANDSET_CALIBRATION";
		case INPUT_HANDSET_VALUES:      return "INPUT_HANDSET_VALUES";
		case INPUT_HANDSET_SETUP:       return "INPUT_HANDSET_SETUP";
		case INPUT_JOYSTICK_VALUES:     return "INPUT_JOYSTICK_VALUES";
		case INPUT_JOYSTICK_SETUP:      return "INPUT_JOYSTICK_SETUP";
		case SYSTEM_POWER_ON:           return "SYSTEM_POWER_ON";
		case SYSTEM_INITIALIZE:         return "SYSTEM_INITIALIZE";
		case SYSTEM_HEALTH_AND_STATUS:  return "SYSTEM_HEALTH_AND_STATUS";
		case SYSTEM_HARDWARE_ERROR:     return "SYSTEM_HARDWARE_ERROR";
		case SYSTEM_REBOOT:             return "SYSTEM_REBOOT";
		case TELEMETRY_HEARTBEAT:       return "TELEMETRY_HEARTBEAT";
		case TELEMETRY_POSITION:        return "TELEMETRY_POSITION";
		case TELEMETRY_ORIENTATION:     return "TELEMETRY_ORIENTATION";
		case TELEMETRY_PRESSURE:        return "TELEMETRY_PRESSURE";
		case TELEMETRY_CONTROL:         return "TELEMETRY_CONTROL";
		case TELEMETRY_SYSTEM:          return "TELEMETRY_SYSTEM";
		case TELEMETRY_GCS:             return "TELEMETRY_GCS";
		case TELEMETRY_GCS_LOCATION:    return "TELEMETRY_GCS_LOCATION";
		case TELEMETRY_PAYLOAD:         return "TELEMETRY_PAYLOAD";
		case TELEMETRY_GCS_SVIN:        return "TELEMETRY_GCS_SVIN";
		case TELEMETRY_DEPLOYMENT_TUBE: return "TELEMETRY_DEPLOYMENT_TUBE";
		case HWIL_SENSORS:              return "HWIL_SENSORS";
		case HWIL_ACTUATORS:            return "HWIL_ACTUATORS";
		case HWIL_CAN:                  return "HWIL_CAN";
		case FLIGHT_PLAN:               return "FLIGHT_PLAN";
		case FLIGHT_PLAN_MAP:           return "FLIGHT_PLAN_MAP";
		case FLIGHT_PLAN_WAYPOINT:      return "FLIGHT_PLAN_WAYPOINT";
		case LAST_MAPPING_WAYPOINT:     return "LAST_MAPPING_WAYPOINT";
		case DUBIN_PATH:                return "DUBIN_PATH";
		case VEHICLE_PARAMS:            return "VEHICLE_PARAMS";
		case VEHICLE_LIMITS:            return "VEHICLE_LIMITS";
		case VEHICLE_LAUNCH_PARAMS:     return "VEHICLE_LAUNCH_PARAMS";
		case VEHICLE_LAND_PARAMS:       return "VEHICLE_LAND_PARAMS";
		case MISSION_CHECKLIST:         return "MISSION_CHECKLIST";
		case MISSION_PARAMETERS:        return "MISSION_PARAMETERS";
		case MISSION_HDOB_CONFIG:       return "MISSION_HDOB_CONFIG";
		case PAYLOAD_TRIGGER:           return "PAYLOAD_TRIGGER";
		case PAYLOAD_PARAMS:            return "PAYLOAD_PARAMS";
		case PAYLOAD_NDVI:              return "PAYLOAD_NDVI";
		case PAYLOAD_LDCR:              return "PAYLOAD_LDCR";
		case PAYLOAD_CONTROL:           return "PAYLOAD_CONTROL";
		case PAYLOAD_CAMERA_TAG:        return "PAYLOAD_CAMERA_TAG";
		case PAYLOAD_STATUS:            return "PAYLOAD_STATUS";
		case PAYLOAD_SERIAL:            return "PAYLOAD_SERIAL";
		case PAYLOAD_DATA_CHANNEL_0:    return "PAYLOAD_DATA_CHANNEL_0";
		case PAYLOAD_DATA_CHANNEL_1:    return "PAYLOAD_DATA_CHANNEL_1";
		case PAYLOAD_DATA_CHANNEL_2:    return "PAYLOAD_DATA_CHANNEL_2";
		case PAYLOAD_DATA_CHANNEL_3:    return "PAYLOAD_DATA_CHANNEL_3";
		case PAYLOAD_DATA_CHANNEL_4:    return "PAYLOAD_DATA_CHANNEL_4";
		case PAYLOAD_DATA_CHANNEL_5:    return "PAYLOAD_DATA_CHANNEL_5";
		case PAYLOAD_DATA_CHANNEL_6:    return "PAYLOAD_DATA_CHANNEL_6";
		case PAYLOAD_DATA_CHANNEL_7:    return "PAYLOAD_DATA_CHANNEL_7";
		case PAYLOAD_S0_SENSORS:        return "PAYLOAD_S0_SENSORS";
		case INVALID_PACKET:            return "INVALID_PACKET";
		default:                        return NULL;
	}
}

/* ------------------------------------------------- "no data" sentinels */
//
// Firmware encodes an invalid / no-data telemetry field as the *maximum value
// of its integer type* (see shared/.../comm_handler_base.cpp getTelemetryValue:
// `if(value == numeric_limits<type>::max()) return NAN;`).  These helpers undo
// the scaling and return NaN for that sentinel so we don't print absurd numbers
// (e.g. a "no fix" latitude of INT64_MAX reading as +922.3372037 deg).
//
static double tv_i8 (int8_t   v, double s) { return v == INT8_MAX   ? NAN : v / s; }
static double tv_u8 (uint8_t  v, double s) { return v == UINT8_MAX  ? NAN : v / s; }
static double tv_i16(int16_t  v, double s) { return v == INT16_MAX  ? NAN : v / s; }
static double tv_u16(uint16_t v, double s) { return v == UINT16_MAX ? NAN : v / s; }
static double tv_i32(int32_t  v, double s) { return v == INT32_MAX  ? NAN : v / s; }
static double tv_u32(uint32_t v, double s) { return v == UINT32_MAX ? NAN : v / s; }
static double tv_i64(int64_t  v, double s) { return v == INT64_MAX  ? NAN : v / s; }

// Format a possibly-NaN value into one of a few rotating static buffers (so
// several can appear in one snprintf call); NaN renders as "---".
static const char * fd(double v, const char * fmt)
{
	static char bufs[8][32];
	static unsigned bi = 0;
	char * b = bufs[bi++ & 7];
	if(isnan(v)) snprintf(b, 32, "---");
	else         snprintf(b, 32, fmt, v);
	return b;
}

/* ----------------------------------------------------------- field decoders */
//
// Each decoder fills `out` with up to `max` field lines and returns the count.
// Returning 0 means "no decoder" -> the caller falls back to a hex/ascii dump.
//

static int decodePosition(const uint8_t * d, uint16_t size, char out[][120], int max)
{
	if(size < sizeof(TelemetryPosition_t)) return 0;
	TelemetryPosition_t p; memcpy(&p, d, sizeof(p));
	int n = 0;
	if(n<max) snprintf(out[n++],120,"  system_time   %s s",     fd(tv_u32(p.system_time,1000.0),"%.3f"));
	if(n<max) snprintf(out[n++],120,"  latitude      %s deg",   fd(tv_i64(p.latitude,1e16),"%+.7f"));
	if(n<max) snprintf(out[n++],120,"  longitude     %s deg",   fd(tv_i64(p.longitude,1e16),"%+.7f"));
	if(n<max) snprintf(out[n++],120,"  altitude      %s m MSL", fd(tv_i32(p.altitude,1000.0),"%.3f"));
	if(n<max) snprintf(out[n++],120,"  gps_altitude  %s m MSL", fd(tv_i32(p.gps_altitude,1000.0),"%.3f"));
	if(n<max) snprintf(out[n++],120,"  height (AGL)  %s m",     fd(tv_i32(p.height,1000.0),"%.3f"));
	if(n<max) snprintf(out[n++],120,"  laser_dist    %s m",     fd(tv_u16(p.laser_distance,100.0),"%.2f"));
	if(n<max) snprintf(out[n++],120,"  velocity      <%s %s %s> m/s",
			fd(tv_i16(p.velocity[0],100.0),"%+.2f"), fd(tv_i16(p.velocity[1],100.0),"%+.2f"), fd(tv_i16(p.velocity[2],100.0),"%+.2f"));
	if(n<max) snprintf(out[n++],120,"  acceleration  <%s %s %s> m/s^2",
			fd(tv_i16(p.acceleration[0],100.0),"%+.2f"), fd(tv_i16(p.acceleration[1],100.0),"%+.2f"), fd(tv_i16(p.acceleration[2],100.0),"%+.2f"));
	return n;
}

static int decodeOrientation(const uint8_t * d, uint16_t size, char out[][120], int max)
{
	if(size < sizeof(TelemetryOrientation_t)) return 0;
	TelemetryOrientation_t o; memcpy(&o, d, sizeof(o));
	int n = 0;
	if(n<max) snprintf(out[n++],120,"  system_time   %s s", fd(tv_u32(o.system_time,1000.0),"%.3f"));
	if(n<max) snprintf(out[n++],120,"  quaternion    [%s %s %s %s]",
			fd(tv_i16(o.q[0],10000.0),"%+.4f"), fd(tv_i16(o.q[1],10000.0),"%+.4f"),
			fd(tv_i16(o.q[2],10000.0),"%+.4f"), fd(tv_i16(o.q[3],10000.0),"%+.4f"));
	if(n<max) snprintf(out[n++],120,"  omega         <%s %s %s> deg/s",
			fd(tv_i16(o.omega[0],100.0),"%+.2f"), fd(tv_i16(o.omega[1],100.0),"%+.2f"), fd(tv_i16(o.omega[2],100.0),"%+.2f"));
	if(n<max) snprintf(out[n++],120,"  magnetometer  <%s %s %s> uT",
			fd(tv_i16(o.magnetometer[0],100.0),"%+.2f"), fd(tv_i16(o.magnetometer[1],100.0),"%+.2f"), fd(tv_i16(o.magnetometer[2],100.0),"%+.2f"));
	return n;
}

static int decodePressure(const uint8_t * d, uint16_t size, char out[][120], int max)
{
	if(size < sizeof(TelemetryPressure_t)) return 0;
	TelemetryPressure_t p; memcpy(&p, d, sizeof(p));
	int n = 0;
	if(n<max) snprintf(out[n++],120,"  system_time   %s s",  fd(tv_u32(p.system_time,1000.0),"%.3f"));
	if(n<max) snprintf(out[n++],120,"  static_press  %s Pa", fd(tv_u32(p.static_pressure,10.0),"%.1f"));
	if(n<max) snprintf(out[n++],120,"  dynamic_press %s Pa", fd(tv_i16(p.dynamic_pressure,10.0),"%.1f"));
	if(n<max) snprintf(out[n++],120,"  air_temp      %s C",  fd(tv_i16(p.air_temperature,100.0),"%.2f"));
	if(n<max) snprintf(out[n++],120,"  humidity      %s %%", fd(tv_u16(p.humidity,100.0),"%.2f"));
	if(n<max) snprintf(out[n++],120,"  wind          <%s %s %s> m/s",
			fd(tv_i16(p.wind[0],100.0),"%+.2f"), fd(tv_i16(p.wind[1],100.0),"%+.2f"), fd(tv_i16(p.wind[2],100.0),"%+.2f"));
	if(n<max) snprintf(out[n++],120,"  ias / tas     %s / %s m/s", fd(tv_i16(p.ias,100.0),"%.2f"), fd(tv_i16(p.tas,100.0),"%.2f"));
	if(n<max) snprintf(out[n++],120,"  alpha / beta  %s / %s deg", fd(tv_i16(p.alpha,100.0),"%+.2f"), fd(tv_i16(p.beta,100.0),"%+.2f"));
	return n;
}

static int decodeSystem(const uint8_t * d, uint16_t size, char out[][120], int max)
{
	if(size < sizeof(TelemetrySystem_t)) return 0;
	TelemetrySystem_t s; memcpy(&s, d, sizeof(s));
	int n = 0;
	if(n<max) snprintf(out[n++],120,"  system_time   %s s", fd(tv_u32(s.system_time,1000.0),"%.3f"));
	if(n<max) snprintf(out[n++],120,"  battery       %s V  %s A  %s %%",
			fd(tv_u16(s.batt_voltage,1000.0),"%.3f"), fd(tv_i16(s.batt_current,100.0),"%+.2f"), fd(tv_u16(s.batt_percent,100.0),"%.2f"));
	if(n<max) snprintf(out[n++],120,"  watt_hours    %s Wh", fd(tv_u16(s.batt_watt_hours,10.0),"%.1f"));
	if(n<max) snprintf(out[n++],120,"  flight_time   %u s", s.flight_time);
	if(n<max) snprintf(out[n++],120,"  gps           sats %u   pdop %s   fix %u",
			s.satellites, fd(tv_u16(s.pdop,100.0),"%.2f"), (unsigned)s.fix_type);
	if(n<max) snprintf(out[n++],120,"  rssi          %s dB", fd(tv_i8(s.rssi,1.0),"%.0f"));
	if(n<max) snprintf(out[n++],120,"  lost_comm/gps %u / %u   engine %u",
			s.lost_comm, s.lost_gps, s.engine_on);
	if(n<max) snprintf(out[n++],120,"  error_code    0x%08X", s.error_code);
	if(n<max) snprintf(out[n++],120,"  ap_mode       %u   flight_mode %u",
			(unsigned)s.autopilot_mode, (unsigned)s.flight_mode);
	return n;
}

static int decodeControl(const uint8_t * d, uint16_t size, char out[][120], int max)
{
	if(size < sizeof(TelemetryControl_t)) return 0;
	TelemetryControl_t c; memcpy(&c, d, sizeof(c));
	int n = 0;
	if(n<max) snprintf(out[n++],120,"  system_time   %s s", fd(tv_u32(c.system_time,1000.0),"%.3f"));
	if(n<max) snprintf(out[n++],120,"  rpy           <%s %s %s> rad",
			fd(tv_i16(c.roll,10000.0),"%+.4f"), fd(tv_i16(c.pitch,10000.0),"%+.4f"), fd(tv_i16(c.yaw,10000.0),"%+.4f"));
	if(n<max) snprintf(out[n++],120,"  rates         <%s %s %s> rad/s",
			fd(tv_i16(c.roll_rate,100.0),"%+.2f"), fd(tv_i16(c.pitch_rate,100.0),"%+.2f"), fd(tv_i16(c.yaw_rate,100.0),"%+.2f"));
	if(n<max) snprintf(out[n++],120,"  velocity      <%s %s %s> m/s",
			fd(tv_i16(c.velocity[0],100.0),"%+.2f"), fd(tv_i16(c.velocity[1],100.0),"%+.2f"), fd(tv_i16(c.velocity[2],100.0),"%+.2f"));
	if(n<max) snprintf(out[n++],120,"  altitude      %s m", fd(tv_i32(c.altitude,1000.0),"%.3f"));
	if(n<max) snprintf(out[n++],120,"  waypoint      %u   look_at %u", c.waypoint, c.look_at_point);
	if(n<max) snprintf(out[n++],120,"  modes lat/alt/nav  %u / %u / %u",
			(unsigned)c.lat_mode, (unsigned)c.alt_mode, (unsigned)c.nav_mode);
	if(n<max) {
		char * o = out[n]; int len = 0;
		len += snprintf(o+len, 120-len, "  actuators    ");
		for(int i=0;i<8 && len < 108;i++)
			len += snprintf(o+len, 120-len, " %s", fd(tv_i16(c.actuators[i],100.0),"%+.0f"));
		snprintf(o+len, 120-len, " ...");
		n++;
	}
	return n;
}

static int decodeDetail(uint8_t type, const uint8_t * d, uint16_t size, char out[][120], int max)
{
	switch(type) {
		case TELEMETRY_POSITION:    return decodePosition(d, size, out, max);
		case TELEMETRY_ORIENTATION: return decodeOrientation(d, size, out, max);
		case TELEMETRY_PRESSURE:    return decodePressure(d, size, out, max);
		case TELEMETRY_SYSTEM:      return decodeSystem(d, size, out, max);
		case TELEMETRY_CONTROL:     return decodeControl(d, size, out, max);
		default:                    return 0;
	}
}

/* ---------------------------------------------------------------- internals */

static void getTermSize()
{
	struct winsize ws;
	if(ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_row > 0 && ws.ws_col > 0) {
		term_rows = ws.ws_row;
		term_cols = ws.ws_col;
	} else {
		term_rows = 24; term_cols = 80;
	}
	if(term_cols > 480) term_cols = 480;   // emit()/separators use 512-byte buffers
	if(term_cols < 40)  term_cols = 40;
	if(term_rows < 10)  term_rows = 10;
}

// Width we actually write to: one short of the terminal so we never touch the
// last column (writing the final column arms the terminal's auto-wrap, and the
// following '\n' then double-advances -> the screen "twitches"/jumps).  \033[K
// clears the rest of the line, so we never need to pad to full width.
static int lineWidth()
{
	int w = term_cols - 1;
	if(w < 1) w = 1;
	if(w > 510) w = 510;
	return w;
}

// Newlines are written *between* lines, never after the last one: a trailing
// '\n' on the bottom row scrolls the whole frame up by one (a source of the
// "jumping").  render() sets line_first = true right after homing the cursor.
static bool line_first = true;

// Emit one line: truncate (no padding), clear to EOL.
static void emit(const char * s)
{
	char buf[512];
	int w = lineWidth();
	snprintf(buf, w + 1, "%s", s);
	if(!line_first) fputs("\n", stdout);
	line_first = false;
	fputs(buf, stdout);
	fputs("\033[K", stdout);
}

// Emit one line in reverse video (used for the selected table row).  Here we do
// pad to lineWidth() so the highlight bar spans the row -- but still stop one
// column short of the edge, so no auto-wrap.
static void emitSelected(const char * s)
{
	char buf[512];
	int w = lineWidth();
	snprintf(buf, w + 1, "%-*.*s", w, w, s);
	if(!line_first) fputs("\n", stdout);
	line_first = false;
	fputs("\033[7m", stdout);
	fputs(buf, stdout);
	fputs("\033[0m\033[K", stdout);
}

static int buildVisible(uint8_t * vis)
{
	int n = 0;
	for(int i = 0; i < NUM_TYPES; i++)
		if(stats[i].total > 0) vis[n++] = (uint8_t)i;
	return n;
}

static void tickRates(float now)
{
	float dt = now - win_start;
	if(dt < RATE_WINDOW) return;
	for(int i = 0; i < NUM_TYPES; i++) {
		stats[i].hz = (float)stats[i].win_count / dt;
		stats[i].win_count = 0;
	}
	win_start = now;
}

static void clearStats()
{
	memset(stats, 0, sizeof(stats));
	win_start = getElapsedTime();
	sel = 0; table_top = 0;
}

static void moveSel(int d) { sel += d; force_render = true; }

static void render(float now)
{
	getTermSize();

	uint8_t vis[NUM_TYPES];
	vis_count = buildVisible(vis);

	// clamp selection / scroll window
	if(sel >= vis_count) sel = vis_count - 1;
	if(sel < 0) sel = 0;

	// Budget the screen so the total emitted line count is EXACTLY term_rows
	// (emitting more scrolls the frame -> "jumping").  There are 7 fixed lines:
	//   header, sep, column-header, post-table sep, detail title, pre-footer sep,
	//   footer.  The remaining `body` rows are split between the table and detail.
	int body = term_rows - 7;
	if(body < 2) body = 2;
	int detail_body = body / 2;
	if(detail_body > 14) detail_body = 14;
	if(detail_body < 3)  detail_body = 3;
	int table_rows = body - detail_body;
	if(table_rows < 1) { table_rows = 1; detail_body = body - table_rows; }

	if(sel < table_top)               table_top = sel;
	if(sel >= table_top + table_rows) table_top = sel - table_rows + 1;
	if(table_top < 0) table_top = 0;

	// aggregate rate
	float total_hz = 0.0f;
	uint64_t total_pkts = 0;
	for(int i = 0; i < NUM_TYPES; i++) { total_hz += stats[i].hz; total_pkts += stats[i].total; }

	char line[512];

	fputs("\033[H", stdout);   // home cursor
	line_first = true;

	// --- header ---
	snprintf(line, sizeof(line),
			"BST PAYLOAD MONITOR  [%s]  up %.1fs  %d types  %llu pkts  %.0f pkt/s%s",
			conn_str, now, vis_count, (unsigned long long)total_pkts, total_hz,
			paused ? "   *** PAUSED ***" : "");
	emit(line);

	memset(line, '-', term_cols); line[term_cols < (int)sizeof(line) ? term_cols : (int)sizeof(line)-1] = 0;
	emit(line);

	// --- table column header ---
	snprintf(line, sizeof(line),
			"   ID  %-26s %9s  %9s  %6s   %s",
			"PACKET TYPE", "RATE", "COUNT", "BYTES", "AGE");
	emit(line);

	// --- table rows ---
	for(int r = 0; r < table_rows; r++) {
		int idx = table_top + r;
		if(idx >= vis_count) { emit(""); continue; }
		uint8_t t = vis[idx];
		const char * name = packetName(t);
		char namebuf[32];
		if(!name) { snprintf(namebuf, sizeof(namebuf), "type_%u", t); name = namebuf; }
		float age = now - stats[t].last_time;
		snprintf(line, sizeof(line),
				" %c %3u  %-26.26s %6.1f Hz  %9u  %6u   %5.1fs",
				(idx == sel) ? '>' : ' ', t, name, stats[t].hz,
				stats[t].total, stats[t].last_size, age);
		if(idx == sel) emitSelected(line);
		else           emit(line);
	}

	// --- separator ---
	memset(line, '-', term_cols); line[term_cols < (int)sizeof(line) ? term_cols : (int)sizeof(line)-1] = 0;
	emit(line);

	// --- detail pane ---
	if(vis_count > 0) {
		uint8_t t = vis[sel];
		const char * name = packetName(t);
		char namebuf[32];
		if(!name) { snprintf(namebuf, sizeof(namebuf), "type_%u", t); name = namebuf; }

		snprintf(line, sizeof(line),
				"DETAIL  %s  (id %u, %u bytes, last %.1fs ago)%s",
				name, t, stats[t].last_size, now - stats[t].last_time,
				hex_mode ? "   [hex]" : "");
		emit(line);

		char dlines[DETAIL_MAX][120];
		int n = hex_mode ? 0 : decodeDetail(t, stats[t].last_data, stats[t].last_size, dlines, DETAIL_MAX);

		if(n > 0) {
			for(int i = 0; i < detail_body; i++)
				emit(i < n ? dlines[i] : "");
		} else {
			// hex + ascii dump of the captured payload
			int cap = stats[t].cap_size;
			for(int r = 0; r < detail_body; r++) {
				int off = r * 16;
				if(off >= cap) { emit(""); continue; }
				char hexp[64] = {0}; int ho = 0;
				char asc[20]  = {0}; int ao = 0;
				for(int b = 0; b < 16; b++) {
					if(off + b < cap) {
						uint8_t v = stats[t].last_data[off + b];
						ho += snprintf(hexp+ho, sizeof(hexp)-ho, "%02x ", v);
						asc[ao++] = (v >= 32 && v < 127) ? (char)v : '.';
					} else {
						ho += snprintf(hexp+ho, sizeof(hexp)-ho, "   ");
					}
				}
				asc[ao] = 0;
				snprintf(line, sizeof(line), "  %04x  %s |%s|", off, hexp, asc);
				emit(line);
			}
		}
	} else {
		emit("  (waiting for packets...)");
		for(int i = 1; i < detail_body; i++) emit("");
	}

	// --- footer ---
	memset(line, '-', term_cols); line[term_cols < (int)sizeof(line) ? term_cols : (int)sizeof(line)-1] = 0;
	emit(line);
	emit(" [j/k or up/down] select   [x] hex/decode   [p] pause   [c] clear   [q] quit");

	fputs("\033[J", stdout);   // clear anything below
	fflush(stdout);
}

static void handleKey(int c)
{
	if(c == 27) {                       // ESC: start of an arrow / nav sequence
		// Read the rest of the sequence straight from stdio (it was buffered with
		// the ESC).  A lone ESC is ignored -- it must NOT quit, or a split escape
		// sequence would disconnect the session.
		int c2 = getchar();
		if(c2 < 0) { clearerr(stdin); return; }      // lone ESC -> ignore
		if(c2 == '[' || c2 == 'O') {
			int c3 = getchar();
			if(c3 < 0) { clearerr(stdin); return; }
			switch(c3) {
				case 'A': moveSel(-1); break;                                   // up
				case 'B': moveSel(1);  break;                                   // down
				case 'H': sel = 0;            force_render = true; break;       // Home
				case 'F': sel = vis_count-1;  force_render = true; break;       // End
				case '5': case '6': { int t = getchar(); if(t < 0) clearerr(stdin); } break; // PgUp/PgDn '~'
				default: break;
			}
		}
		return;
	}

	switch(c) {
		case 'k': moveSel(-1); break;
		case 'j': moveSel(1);  break;
		case 'g': sel = 0;            force_render = true; break;
		case 'G': sel = vis_count-1;  force_render = true; break;
		case 'x': hex_mode = !hex_mode; force_render = true; break;
		case 'p':
		case ' ': paused = !paused;     force_render = true; break;
		case 'c': clearStats();         force_render = true; break;
		case 'q':
		case 3:   running = false; break;   // 'q' / Ctrl-C
		default:  break;
	}
}

/* -------------------------------------------------------------------- public */

void monitorInit(const char * conn)
{
	memset(stats, 0, sizeof(stats));
	if(conn) { strncpy(conn_str, conn, sizeof(conn_str)-1); conn_str[sizeof(conn_str)-1] = 0; }
	win_start    = getElapsedTime();
	last_render  = -1.0f;
	force_render = true;
	fputs("\033[?25l\033[2J", stdout);   // hide cursor, clear screen
	fflush(stdout);
}

void monitorPacket(uint8_t type, const void * data, uint16_t size)
{
	TypeStat * s = &stats[type];
	s->total++;
	s->win_count++;
	s->last_size = size;
	s->last_time = getElapsedTime();

	uint16_t n = size;
	if(n > MAX_CAPTURE) n = MAX_CAPTURE;
	if(data && n) memcpy(s->last_data, data, n);
	s->cap_size = n;
}

void monitorUpdate(void)
{
	float now = getElapsedTime();

	// Drain all pending input.  getchar() is non-blocking here (initTerminal sets
	// the tty to VMIN=0/VTIME=0) and returns EOF the instant nothing is buffered.
	// We read straight through stdio rather than gating on inputAvailable()
	// (a select() on the fd): stdio buffers the whole "ESC [ A" arrow burst on the
	// first read, leaving the fd empty, so select() would strand the tail of the
	// sequence -- which previously made an arrow key look like a bare ESC.
	for(;;) {
		int c = getchar();
		if(c < 0) { clearerr(stdin); break; }
		handleKey(c);
		if(!running) return;
	}

	if(!paused) tickRates(now);

	bool do_render = force_render || (now - last_render >= RENDER_PERIOD);
	if(paused) do_render = force_render;   // frozen frame unless a key forces a redraw

	if(do_render) {
		render(now);
		last_render  = now;
		force_render = false;
	}
}

void monitorShutdown(void)
{
	fputs("\033[?25h\033[2J\033[H", stdout);   // show cursor, clear, home
	fflush(stdout);
}
