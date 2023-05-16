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
#ifndef _MAGNETOMETER_CALIBRATIONS_H_
#define _MAGNETOMETER_CALIBRATIONS_H_

#define USING_MAG_ZERO

/* -----[ Calibration ] ----- */

#ifndef USING_MAG_ZERO

#define  MAG_M_0_0  0.0
#define  MAG_M_1_0  0.0
#define  MAG_M_2_0  0.0
#define  MAG_M_0_1  0.0
#define  MAG_M_1_1  0.0
#define  MAG_M_2_1  0.0
#define  MAG_M_0_2  0.0
#define  MAG_M_1_2  0.0
#define  MAG_M_2_2  0.0

#define  MAG_B_0    0.0
#define  MAG_B_1    0.0
#define  MAG_B_2    0.0

#endif 

/* -----[ Zero ] ----- */

#ifdef USING_MAG_ZERO

#define  MAG_M_0_0  1.0
#define  MAG_M_1_0  0.0
#define  MAG_M_2_0  0.0
#define  MAG_M_0_1  0.0
#define  MAG_M_1_1  1.0
#define  MAG_M_2_1  0.0
#define  MAG_M_0_2  0.0
#define  MAG_M_1_2  0.0
#define  MAG_M_2_2  1.0

#define  MAG_B_0    0.0
#define  MAG_B_1    0.0
#define  MAG_B_2    0.0

#endif 


#endif
