CC=gcc
CXX=g++
#CC=-aarch64-linux-gnu-gcc
#CXX=aarch64-linux-gnu-g++

CFLAGS= -fPIC

libbstfw_FLAGS= -DVEHICLE_MULTIROTOR
libbstmr_FLAGS= -DVEHICLE_FIXEDWING

LDFLAGS=-lrt

FLAGS = $(CFLAGS) $(LDFLAGS) -DVERBOSE -DSDK
#FLAGS = $(CFLAGS) $(LDFLAGS) -DSDK

ifeq ($(LOW_BANDWIDTH),1)
FLAGS += -DLOW_BANDWIDTH
endif

ifeq ($(SERIAL_COMMS),1)
FLAGS += -DSERIAL_COMMS
endif

INCLUDES+=-I. \
			    -I../include/bst_can \
			    -I../include/bst_core \
			    -I../include/bst_protocol \
			    -I../include/bst_protocol/messages \
			    -I../include/bst_protocol/modules \

VPATH+=../src/bst_can \
			 ../src/bst_core \
			 ../src/bst_protocol \
			 ../src/bst_protocol/modules \

OBJ=

BST_COMM_OBJ=bst_serial.o bst_socket.o bst_interface.o bst_time.o

BST_OBJ= simulated_can.o \
			   bridge.o \
			   comm_interface.o \
			   debug.o \
			   file_interface.o \
			   comm_protocol.o \
			   helper_functions.o \
			   vector.o \
			   flight_plan.o \
			   bst_module.o \
			   bst_protocol.o \
			   bst_packet.o \
			   bst_module_flight_plan.o \
			   bst_module_basic.o \

.PHONY: default libbst libbstfw libbstmr
default:
	BIN_DIR=bin/bst make libbst
	BIN_DIR=bin/libbstfw make libbstfw
	BIN_DIR=bin/libbstmr make libbstmr

libbst: ${BIN_DIR} libbst_lib.a

libbst_lib.a: $(addprefix ${BIN_DIR}/, $(BST_COMM_OBJ))
	ar rcs $@ $^

libbstfw: FLAGS+= -DVEHICLE_FIXEDWING
libbstfw: ${BIN_DIR} libbstfw_lib.a

libbstfw_lib.a: $(addprefix ${BIN_DIR}/, $(BST_OBJ))
	ar rcs $@ $^

libbstmr: FLAGS+= -DVEHICLE_MULTIROTOR
libbstmr: ${BIN_DIR} libbstmr_lib.a

libbstmr_lib.a: $(addprefix ${BIN_DIR}/, $(BST_OBJ))
	ar rcs $@ $^


${BIN_DIR}:
	@mkdir -p ${BIN_DIR}

${BIN_DIR}/%.o:%.cpp
	$(CXX) -c -o $@ $^ $(INCLUDES) $(DEFINES) $(FLAGS)
 
${BIN_DIR}/%.o: %.c
	$(CXX) -c -o $@ $< $(INCLUDES) $(DEFINES) $(FLAGS)

${BIN_DIR}/%.o:%.cpp %.h
	$(CXX) -c -o $@ $< $(INCLUDES) $(DEFINES) $(FLAGS)
 
${BIN_DIR}/%.o: %.c %.h
	$(CXX) -c -o $@ $< $(INCLUDES) $(DEFINES) $(FLAGS)

allclean: clean

clean:
	rm -rf bin
	rm -f *.a
	rm -f *.so
