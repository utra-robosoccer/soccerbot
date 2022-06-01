#!/usr/bin/env bash

if [ "$(uname -m)" = "x86_64" ]; then
    echo "Skipping for x86_64"
    uname -m
    exit 0
fi

apt update && apt-fast install -y libopenblas-dev
pip install gdown && gdown 1BoUq15BZC5tKuaBKg5wNg6NowwP6IsHz && pip install mxnet_jetson-1.9.1-py3-none-any.whl && rm -rf mxnet_jetson-1.9.1-py3-none-any.whl
exit 0
