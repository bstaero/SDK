#*=+--+=#=+--            SwiftPilot Autopilot Software            --+=#=+--+=#*#
#               Copyright (C) 2019 Black Swift Technologies LLC.               #
#                             All Rights Reserved.                             #
#                                                                              #
#    This program is free software: you can redistribute it and/or modify      #
#    it under the terms of the GNU General Public License version 2 as         #
#    published by the Free Software Foundation.                                #
#                                                                              #
#    This program is distributed in the hope that it will be useful,           #
#    but WITHOUT ANY WARRANTY; without even the implied warranty of            #
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
#    GNU General Public License for more details.                              #
#                                                                              #
#    You should have received a copy of the GNU General Public License         #
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.     #
#                                                                              #
#                                 Jack Elston                                  #
#                          elstonj@blackswifttech.com                          #
#                                                                              #
#*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*#

HOST_OS := $(shell uname -s)

CC=gcc
CXX=g++

VPATH     = ../../src \
            ../../src/bst_can \
            ../../src/bst_protocol \
            ../../src/bst_protocol/modules \
            ../../src/bst_core \

INCLUDES += -I. \
					  -I../../include \
					  -I../../include/bst_can \
					  -I../../include/bst_protocol \
					  -I../../include/bst_protocol/messages \
					  -I../../include/bst_protocol/modules \
					  -I../../include/bst_core \
					  -I../../lib \
					  -I../../lib/netuas \

FLAGS=-g -DVEHICLE_FIXEDWING -DBOARD_core -DBOARD_GNSS -DBOARD_GCS -DBOARD_DEPLOYMENT -DSDK -DIMPLEMENTATION_xplane -DIMPLEMENTATION_swil
FLAGS+=-DVERBOSE 

OBJ=comm_protocol.o comm_interface.o bst_module_basic.o bst_module_flight_plan.o debug.o vector.o helper_functions.o flight_plan.o
OBJ+=bst_protocol.o bst_packet.o bst_module.o
OBJ+=simulated_can.o bridge.o

ifneq ($(HOST_OS), Darwin)
FLAGS+=-lrt
endif

%.o:%.cpp
	$(CXX) $(FLAGS) -c -o $@ $^ $(INCLUDES)
 
%.o: %.c
	$(CXX) $(FLAGS) -c -o $@ $< $(INCLUDES)
 
default: bstsdklibs test

test: $(OBJ) main.o test.o test_handler.o
	$(CXX) -o $@ $^ $(INCLUDES) $(FLAGS) -L../../lib -lnetuas_lib -lrt

clean:
	cd ../../lib && $(MAKE) clean
	rm -f *.o
	rm -f test

bstsdklibs: ../../lib/libnetuas_lib.a

../../lib/libnetuas_lib.a:
	cd ../../lib && $(MAKE)

