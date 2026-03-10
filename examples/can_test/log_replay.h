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
#ifndef _LOG_REPLAY_H_
#define _LOG_REPLAY_H_

#include <inttypes.h>

// Replay time bounds (seconds relative to start of log, -1 = not set)
extern float replay_start_s;
extern float replay_stop_s;

// Returns true if replay mode was entered and has now exited back to menu
bool runLogReplay(const char * filename);

#endif
