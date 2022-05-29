# From https://mxnet.apache.org/versions/1.4.1/install/index.html?platform=Devices&language=Python&processor=GPU
sudo apt-get update
sudo apt-get -y install git build-essential libatlas-base-dev libopencv-dev graphviz python-pip
sudo pip install pip --upgrade
sudo pip install setuptools numpy --upgrade
sudo pip install graphviz jupyter

git clone https://github.com/utra-robosoccer/incubator-mxnet.git --recursive
cd incubator-mxnet

cp make/crosscompile.jetson.mk config.mk

# MSHADOW_CFLAGS += -DMSHADOW_USE_PASCAL=1
make -j $(nproc)

cd python
pip install --upgrade pip
pip install -e .

cd ..
export MXNET_HOME=$(pwd)
echo "export PYTHONPATH=$MXNET_HOME/python:$PYTHONPATH" >> ~/.rc
source ~/.rc
