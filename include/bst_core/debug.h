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

#ifndef __DEBUG_H
#define __DEBUG_H
#include <stdarg.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum _VerboseLevel_t {
	VERBOSE_NONE, // 0
	VERBOSE_ERROR, 
	VERBOSE_WARN, 
	VERBOSE_STATUS, 
	VERBOSE_INFO, 
	VERBOSE_PAYLOAD, // 5
	VERBOSE_FP,
	VERBOSE_ALLOC, 
	VERBOSE_PARAM, 
	VERBOSE_SENSORS, 
	VERBOSE_EKF,  // 10
	VERBOSE_PACKETS,  
	VERBOSE_ALL_PACKETS,
	VERBOSE_CAN, 
	VERBOSE_ALL 
} VerboseLevel_t;


#ifndef VERBOSE
/* gcc's cpp has extensions; it allows for macros with a variable number of
 *    arguments. We use this extension here to preprocess pmesg away. */
#define pmesg(level, format, args...) ((void)0)

#else
/* To be implemented elsewhere */
void consoleout(const char *msg);

extern int8_t verbose; /* the higher, the more messages... */

void pmesg(VerboseLevel_t level, const char *format, ...);
/* print a message, if it is considered significant enough.
 *       Adapted from [K&R2], p. 174 */
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H */
