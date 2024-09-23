#!/bin/sh

PYTHON_PATH="/usr/include/python3"
PYTHON_VERSION=""

while read version; do
    VERSION=$(echo $version | cut -d "." -f 2)
    if [ ! -z "${VERSION}" ]; then
        PYTHON_VERSION=$((PYTHON_VERSION+0))
        VERSION=$((VERSION+0))
        if [ $((VERSION)) -gt $((PYTHON_VERSION)) ]; then
            PYTHON_VERSION=$VERSION
        fi
    fi
done <<<$(find /usr/include/ -maxdepth 1 -type d -name python3*)

swig -python -c++ swig_parser.i
g++ -Wall -shared -fPIC -I$PYTHON_PATH.$PYTHON_VERSION swig_parser_wrap.cxx swig_parser.cpp -o _swig_parser.so
