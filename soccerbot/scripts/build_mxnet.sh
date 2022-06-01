# https://mxnet.apache.org/get_started/jetson_setup
sudo apt-get update
sudo apt-get install -y build-essential \
                        git \
                        libopenblas-dev \
                        libopencv-dev \
                        python3-pip \
                        python-numpysudo pip install pip --upgrade
sudo pip3 install --upgrade \
                        pip \
                        setuptools \
                        numpy
git clone --recursive --depth 0 -b v1.9.1 https://github.com/utra-robosoccer/incubator-mxnet.git mxnet

echo "export PATH=/usr/local/cuda/bin:$PATH" >> ~/.bashrc
echo "export MXNET_HOME=$HOME/mxnet/" >> ~/.bashrc
echo "export PYTHONPATH=$MXNET_HOME/python:$PYTHONPATH" >> ~/.bashrc
source ~/.bashrc

cd $MXNET_HOME
cp $MXNET_HOME/make/config_jetson.mk config.mk
make -j $(nproc)

cd $MXNET_HOME/python
sudo pip3 install -e .
