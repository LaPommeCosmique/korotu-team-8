#!/bin/bash


# Compile protobuf to output folder
mkdir output
protoc --python_out=pyi_out:output interface.proto
