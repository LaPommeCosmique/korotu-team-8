#!/bin/bash

# Install the dependencies needed for compiling and running protobufs on RPi 4
sudo apt-get install -y protobuf-compiler

# Python dependencies should be installed in a virtual environment
pip3 install protobuf
pip3 install pytest
pip3 install disutils
pip3 install python3-dev
