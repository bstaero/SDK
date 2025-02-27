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

rm -rf BSTPythonSDK.egg-info
rm -rf build
rm -rf dist
python3 setup.py sdist bdist_wheel

twine upload dist/*
