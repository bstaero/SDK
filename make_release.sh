#!/bin/bash

#lib
cp ../shared/devices/communications/serial/netuas_serial.cpp     ./lib/
cp ../shared/devices/communications/serial/netuas_serial.h       ./lib/
cp ../shared/devices/communications/netuas_lib/*.h               ./lib/netuas
cp ../shared/devices/communications/netuas_lib/*.cpp             ./lib/netuas

#inc
cp ../shared/devices/communications/can/simulated_can.h          ./include/bst_can/
cp ../shared/devices/communications/can/protocol/bridge.h        ./include/bst_can/

cp ../shared/tasks/communications/interface/comm_interface.h     ./include/bst_core/
cp ../shared/comm_protocols/comm_protocol.h                      ./include/bst_core/
cp ../shared/lib/debug.h                                         ./include/bst_core/
cp ../shared/tasks/controller/navigation/flight_plan.h           ./include/bst_core/
cp ../shared/lib/helper_functions.h                              ./include/bst_core/
cp ../shared/lib/vector.h                                        ./include/bst_core/

cp ../shared/comm_protocols/bst/bst_module.h                     ./include/bst_protocol/
cp ../shared/comm_protocols/bst/bst_packet.h                     ./include/bst_protocol/
cp ../shared/comm_protocols/bst/bst_protocol.h                   ./include/bst_protocol/
cp ../shared/comm_protocols/bst/modules/*.h                      ./include/bst_protocol/modules/
cp ../shared/comm_protocols/bst/messages/*.h                     ./include/bst_protocol/messages/

#src
cp ../shared/devices/communications/can/simulated_can.cpp        ./src/bst_can/
cp ../shared/devices/communications/can/protocol/bridge.c        ./src/bst_can/

cp ../shared/tasks/communications/interface/comm_interface.cpp   ./src/bst_core/
cp ../shared/comm_protocols/comm_protocol.cpp                    ./src/bst_core/
cp ../shared/lib/debug.c                                         ./src/bst_core/
cp ../shared/tasks/controller/navigation/flight_plan.cpp         ./src/bst_core/
cp ../shared/lib/helper_functions.cpp                            ./src/bst_core/
cp ../shared/lib/vector.cpp                                      ./src/bst_core/

cp ../shared/comm_protocols/bst/bst_module.cpp                   ./src/bst_protocol/
cp ../shared/comm_protocols/bst/bst_packet.cpp                   ./src/bst_protocol/
cp ../shared/comm_protocols/bst/bst_protocol.cpp                 ./src/bst_protocol/
cp ../shared/comm_protocols/bst/modules/*.cpp                    ./src/bst_protocol/modules/
