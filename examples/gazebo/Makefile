#*=+--+=#=+--            SwiftPilot Autopilot Software            --+=#=+--+=#*#
#               Copyright (C) 2012 Black Swift Technologies LLC.               #
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

INCLUDES += -I. \
					  -I../../include \
					  -I../../include/bst_protocol \
					  -I../../include/bst_protocol/messages \
					  -I../../include/bst_protocol/modules \
					  -I../../include/bst_core \
					  -I../../lib \
					  -I../../lib/netuas \

FLAGS=-g -DVERBOSE -DVEHICLE_MULTIROTOR -lm

OBJ= main.o test.o test_handler.o

ifneq ($(HOST_OS), Darwin)
FLAGS+=-lrt
endif

%.o:%.cpp
	$(CXX) $(FLAGS) -c -o $@ $^ $(INCLUDES)
 
%.o: %.c
	$(CXX) $(FLAGS) -c -o $@ $< $(INCLUDES)
 
default: bstsdklibs test

test: $(OBJ)
	$(CXX) -o $@ $^ $(INCLUDES) $(FLAGS) -L../../lib -lnetuas_lib -lbstmr_lib -lrt

clean:
	cd ../../lib && $(MAKE) clean
	rm -f *.o
	rm -f test

bstsdklibs: ../../lib/libnetuas_lib.a ../../lib/libbstmr_lib.a

../../lib/libnetuas_lib.a:
	cd ../../lib && $(MAKE)

../../lib/libbstmr_lib.a:
	cd ../../lib && $(MAKE)

