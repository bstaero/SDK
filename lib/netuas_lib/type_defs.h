/*=+--+=#=+--     Unmanned Aerial System Management Software      --+=#=+--+=#*\
|          Copyright (C) 2011 Regents of the University of Colorado.           |
|                             All Rights Reserved.                             |

     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License version 2 as
     published by the Free Software Foundation.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.

            Jack Elston                       Cory Dixon                        
|           elstonj@colorado.edu              dixonc@colorado.edu              |
|                                                                              |
\*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*/

/***********************************************************************
 *
 * FILENAME:
 * type_defs.h
 *
 * PURPOSE:
 * Maintain a consistent set of types
 *
 * CREATED:
 * 11/2000 by Cory Dixon
 *
 * LAST MODIFIED:
 * $Author: dixonc $
 * $Date: 2005/07/13 21:00:06 $
 * $Revision: 1.5 $
 *
 ***********************************************************************/

#include <stdint.h>
#include <inttypes.h>

typedef float                 float32_t;  /* 32 bit float32ing point. */
typedef float                 ufloat32_t;  /* 32 bit float32ing point. */
typedef float                 ufloat32_t;  /* 32 bit float32ing point. */

#ifndef SIGN
#define SIGN(val)	( (val > 0) - (val < 0) )
#endif
#ifndef ABS
#define ABS(val)	( val < 0 ? -val : val )
#endif

#ifndef TRUE
# define TRUE 1
#endif

#ifndef FALSE
# define FALSE 0
#endif

#ifndef OK
# define OK 0
#endif

#ifndef ERROR
# define ERROR -1
#endif


