#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# LIB
cd $SCRIPT_DIR/lib
make clean
make

#TEST
cd $SCRIPT_DIR/examples/gazebo
make clean
make

#PAYLOAD
cd $SCRIPT_DIR/examples/payload
make clean
make

#MHP
cd $SCRIPT_DIR/examples/mhp
make clean
make

echo -e "\nDone\n"
