#!/bin/bash

check_command() {
    if ! command -v $1 2>&1 >/dev/null
    then
        echo "Missing '$1', aborting..."
        exit 1
    fi
}

check_command swig
check_command python3-config

swig -python -c++ swig_parser.i
g++ -Wall -shared -fPIC $(python3-config --includes) swig_parser_wrap.cxx swig_parser.cpp -o _swig_parser.so
