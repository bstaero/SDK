#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# LIB
cd $SCRIPT_DIR/lib
make clean
make

cd $SCRIPT_DIR/examples/can_test
make clean
make

cd $SCRIPT_DIR/examples/ch4
make clean
make

cd $SCRIPT_DIR/examples/deployment_tube_test
make clean
make

cd $SCRIPT_DIR/examples/gazebo
make clean
make

cd $SCRIPT_DIR/examples/ldcr
make clean
make

cd $SCRIPT_DIR/examples/mhp
make clean
make

cd $SCRIPT_DIR/examples/payload
make clean
make

cd $SCRIPT_DIR/examples/s0
make clean
make

cd $SCRIPT_DIR/examples/sphere
make clean
make

echo -e "\nDone\n"
