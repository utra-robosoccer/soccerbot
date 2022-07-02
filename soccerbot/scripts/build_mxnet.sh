#!/usr/bin/env bash

# Builds MXNet on the arm and then creates the wheel

# https://mxnet.apache.org/get_started/jetson_setup
# https://www.mssqltips.com/sqlservertip/6802/create-wheel-file-python-package-distribute-custom-code/
apt update
apt-fast install -y build-essential \
                        git \
                        libopenblas-dev \
                        libopencv-dev \
                        python3-pip \
                        python-numpy
pip install pip --upgrade
pip3 install --upgrade \
                        pip \
                        setuptools \
                        numpy
git clone --recursive --depth 1 -b v1.9.1 https://github.com/utra-robosoccer/incubator-mxnet.git mxnet

echo "export PATH=/usr/local/cuda/bin:\$PATH" >> ~/.bashrc
echo "export MXNET_HOME=\$HOME/mxnet/" >> ~/.bashrc
echo "export PYTHONPATH=\$MXNET_HOME/python:\$PYTHONPATH" >> ~/.bashrc
export PATH=/usr/local/cuda/bin:$PATH
export MXNET_HOME=$HOME/mxnet/
export PYTHONPATH=$MXNET_HOME/python:$PYTHONPATH

cd mxnet
cp make/config_jetson.mk config.mk
make -j $(nproc)

cd python
pip3 install -e .

# Create wheel package
pip install wheel
python3 -m pip install --upgrade pip
python3 setup.py bdist_wheel
python3 -m pip install --upgrade twine
python3 -m twine upload --repository testpypi dist/*
