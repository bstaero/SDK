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

FLAGS = $(CFLAGS) 

VPATH     += 
INCLUDES  += -I./
OBJ       += main.o test.o test_handler.o

FLAGS+=-g -DVERBOSE

ifneq ($(HOST_OS), Darwin)
LDFLAGS+=-lrt
endif

#----------[ NetUAS ]----------#
INCLUDES  += -I../../lib -I../../lib/netuas

#-------[ BST Protocol ]-------#
VPATH     += ../../src \
						 ../../src/bst_core \
						 ../../src/bst_protocol
INCLUDES  += -I../../include \
						 -I../../include/bst_core \
						 -I../../include/bst_protocol \
						 -I../../include/bst_protocol/messages
OBJ       += comm_interface.o bst_packet.o helper_functions.o debug.o

%.o:%.cpp
	$(CXX) $(FLAGS) -c -o $@ $^ $(INCLUDES)
 
%.o: %.c
	$(CXX) $(FLAGS) -c -o $@ $< $(INCLUDES)
 
all: default

default: libnetuas test

test: $(OBJ)
	$(CXX) -o $@ $^ -L../../lib -lnetuas_lib $(LDFLAGS) 

clean:
	cd ../../lib && $(MAKE) clean
	rm -f *.o
	rm -f test

libnetuas: ../../lib/libnetuas_lib.a

../../lib/libnetuas_lib.a:
	cd ../../lib && $(MAKE)

