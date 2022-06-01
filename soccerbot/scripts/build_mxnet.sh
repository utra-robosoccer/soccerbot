
if [ "$(uname -m)" = "x86_64" ]; then
    echo "Skipping for x86_64"
    uname -m
    exit 0
fi
# https://mxnet.apache.org/get_started/jetson_setup
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
source ~/.bashrc

cd mxnet
cp make/config_jetson.mk config.mk
make -j $(nproc)

cd python
pip3 install -e .
