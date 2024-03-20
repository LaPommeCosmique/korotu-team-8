# sensor-api V1.0

## How to use
1. Install necessary dependencies in `setup_proto.sh`, it is recommended to do this in a Python virtual environment.
2. Compile protobuf using the `build_protobuf.sh` script. This is necessary anytime changes are made to the protobuf schema.
3. Install Python sensor-api package by running `python setup.py install` in the `protocol/` folder.
4. Run tests (recommended) by executing `pytest -s` in `protocol/test/`.


