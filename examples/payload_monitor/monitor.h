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
// Self-contained terminal "bus monitor" for the BST protocol.
//
// Drop-in usage:
//   monitorInit("serial /dev/ttyUSB0 @ 57600");  // once, after initTerminal()
//   ... in the receive callback:  monitorPacket(type, data, size);
//   ... once per main-loop pass:  monitorUpdate();   // handles keys + redraw
//   monitorShutdown();                               // before restoreTerminal()
//
// The monitor keeps per-type counters, computes a live receive rate for each
// BST packet type, and renders a full-screen table plus a decoded/hex detail
// pane for the selected type.  It owns stdin while running (raw mode is set up
// by the caller via initTerminal()).
//
#ifndef _MONITOR_H_
#define _MONITOR_H_

#include <inttypes.h>

// conn: short human-readable description of the link, shown in the header.
void monitorInit(const char * conn);

// Record one received packet.  Safe to call with data == NULL / size == 0.
void monitorPacket(uint8_t type, const void * data, uint16_t size);

// Handle pending keystrokes and redraw the screen (rate-limited internally).
// Sets the global `running` flag to false when the user quits.
void monitorUpdate(void);

// Restore the terminal screen (cursor, scroll) before exit.
void monitorShutdown(void);

#endif
