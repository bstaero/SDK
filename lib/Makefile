CC=gcc
CXX=g++
CFLAGS= -fPIC -DVEHICLE_MULTIROTOR

LDFLAGS=-lrt

FLAGS = $(CFLAGS) $(LDFLAGS) -DVERBOSE -DSDK

INCLUDES+=-I. \
			    -I./netuas \
			    -I../include/bst_can \
			    -I../include/bst_core \
			    -I../include/bst_protocol \
			    -I../include/bst_protocol/messages \
			    -I../include/bst_protocol/modules \

VPATH+=./netuas \
			 ../src/bst_can \
			 ../src/bst_core \
			 ../src/bst_protocol \
			 ../src/bst_protocol/modules \

OBJ=

NETUAS_OBJ=netuas_serial.o netuas_socket.o serial.o socket.o timeLib.o 

BST_OBJ= simulated_can.o \
			   bridge.o \
			   comm_interface.o \
			   debug.o \
			   comm_protocol.o \
			   helper_functions.o \
			   vector.o \
			   flight_plan.o \
			   bst_module.o \
			   bst_protocol.o \
			   bst_packet.o \
			   bst_module_flight_plan.o \
			   bst_module_basic.o \

default: libnetuas_lib.a libbstmr_lib.a

libnetuas_lib.a: $(NETUAS_OBJ)
	ar rcs $@ $^

libbstmr_lib.a: $(BST_OBJ)
	ar rcs $@ $^

%.o:%.cpp
	$(CXX) $(FLAGS) -c -o $@ $^ $(INCLUDES) $(DEFINES)
 
%.o: %.c
	$(CXX) $(FLAGS) -c -o $@ $< $(INCLUDES) $(DEFINES)

%.o:%.cpp %.h
	$(CXX) $(FLAGS) -c -o $@ $< $(INCLUDES) $(DEFINES)
 
%.o: %.c %.h
	$(CXX) $(FLAGS) -c -o $@ $< $(INCLUDES) $(DEFINES)

allclean: clean
	rmdir netuas

clean:
	rm -f *.o
	rm -f netuas/*.o
	rm -f *.a
	rm -f *.so
