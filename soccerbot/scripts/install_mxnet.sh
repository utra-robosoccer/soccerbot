#!/usr/bin/env bash

if [ "$(uname -m)" = "x86_64" ]; then
    echo "Skipping for x86_64"
    uname -m
    exit 0
fi

apt update && apt-fast install -y libopenblas-dev
pip install graphviz==0.8.1
pip install -i https://test.pypi.org/simple/ mxnet-jetson==1.9.1
exit 0
