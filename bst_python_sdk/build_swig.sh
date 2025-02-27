#!/bin/bash

if ! command -v swig 2>&1 >/dev/null
then
    echo "SWIG is not installed. Aborting..."
    exit 1
fi

swig -python -c++ swig_parser.i
python3 swig_setup.py build_ext --inplace
