#!/bin/sh

while true; do
    echo "Current version: "
    grep "version=" setup.py
    read -p "Did you update setup.py to a new version? (y/n) " yn
    case $yn in
        [Yy]* ) break;;
        * ) exit;;
    esac
done

python setup.py sdist bdist_wheel

twine upload dist/*
