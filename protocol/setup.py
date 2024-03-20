"""
File needed for building/installing sensor-api
"""

from distutils.core import setup

def main():
    setup(name="sensor-api",
        version="1.0.0",
        description="Python packaging for Korotu drone-sensor API",
        author="",
        author_email="",
        py_modules=['device',
                    'drone_side',
                    'sensor_side',
                    'transport_proto',
                    'output/interface_pb2'],)

if __name__ == "__main__":
    main()