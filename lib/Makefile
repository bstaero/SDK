CC=gcc
CXX=g++
CFLAGS= -fPIC

LDFLAGS=-lrt

FLAGS = $(CFLAGS) $(LDFLAGS) -DVERBOSE -DSDK

INCLUDES+=-I. \
			 -I./netuas \
			 -I../include/bst_core \

VPATH+=./netuas \

OBJ=

NETUAS_OBJ=netuas_serial.o netuas_socket.o serial.o socket.o timeLib.o 

default: libnetuas_lib.a

libnetuas_lib.a: $(NETUAS_OBJ)
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
