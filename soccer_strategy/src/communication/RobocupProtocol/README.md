# RobocupProtocol
Protobuf message protocol for the Robocup Humanoid league.

This repository contains example protobuf messages and some code examples for encoding/decoding the different messages.

## Files
```bash
RobocupProtocol/
├── CMakeLists.txt                   # CMake project file
├── decode_base.cpp                  # Decode using the official message
├── decode_extended.cpp              # Decode using the extended message
├── encode_base.cpp                  # Encode using the official message
├── encode_extended.cpp              # Encode using the extended message
├── run_all.sh                       # Script to build code and then run all permutations
├── robocup.proto                    # The official message
├── robocup_extension.proto          # The extended message
└── utils.hpp                        # Utility functions for interpreting message contents
```

## Compiling
```bash
mkdir build
cd build
cmake ..
make
```

## Running
There are four binaries that are built `encode_base`, `encode_extended`, `decode_base`, and `decode_extended`. 

`encode_base` will create a message using the official message protocol and save it to a file named `base_message.pb`. 

`encode_extended` will create a message using the extended message protocol and save it to a file named `extended_message.pb`. 

`decode_base` will take a file as an argument and will decode that file using the official message protocol and print it to the screen in a json format.

`decode_extended` will take a file as an argument and will decode that file using the extended message protocol and print it to the screen in a json format.

Alternatively, if you execute the `run_all.sh` script, it will compile the code and run all permutations of encoding and decoding.

