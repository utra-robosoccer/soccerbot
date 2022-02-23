#! /bin/bash

mkdir -p build
cd build

cmake ..
make

# Encode base message and decode with the base message
echo "Encode base message and decode with the base message"
./encode_base && ./decode_base base_message.pb

# Encode base message and decode with the extended message
echo "Encode base message and decode with the extended message"
./encode_base && ./decode_extended base_message.pb

# Encode extended message and decode with the base message
echo "Encode extended message and decode with the base message"
./encode_extended && ./decode_base extended_message.pb

# Encode extended message and decode with the base message
echo "Encode extended message and decode with the base message"
./encode_extended && ./decode_extended extended_message.pb

cd ..

