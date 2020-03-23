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

#ifndef _STRUCTS_H_
#define _STRUCTS_H_

#include <inttypes.h>
#include <math.h>

#define BST_PROTO

#if defined(BST_PROTO)

  #include "comm_packets.h"
  using namespace bst::comms;

  #if defined(VEHICLE_FIXEDWING)
    #include "fixedwing.h"
		using namespace bst::comms::fixedwing;

  #elif defined(VEHICLE_MULTIROTOR)
    #include "multicopter.h"
		using namespace bst::comms::multicopter;

  #endif

  #include "payload.h"
	using namespace bst::comms::payload;

  #include "gcs.h"
    using namespace bst::comms::gcs;

#endif

#define NUM_CMDS             CMD_INVALID
#define NUM_SURFACE_COMMANDS INVALID_SURFACE
#define CTRL_NUM_LOOPS       CTRL_INVALID
#define CTRL_NUM_CMD_FILT    CMD_FILT_INVALID

/*-----[ MACROS FOR BIT ARRAYS ]-----*/
#define BIT_ARRAY_SIZE(n) (n/8 + (n%8 != 0 ? 1 : 0))
#define BITMASK(b) (1 << ((b) % 8))
#define BITSLOT(b) ((b) / 8)
#define BITGET(a, b)   ((a)[BITSLOT(b)] & BITMASK(b))
#define BITSET(a, b)   ((a)[BITSLOT(b)] |= BITMASK(b))
#define BITCLEAR(a, b) ((a)[BITSLOT(b)] &= ~BITMASK(b))
#define BITFLIP(a, b)  ((a)[BITSLOT(b)] ^= BITMASK(b))

#endif
