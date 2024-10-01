#!/usr/bin/env bash
set -eEo pipefail

WEBOTS_VERSION="2022b"
WEBOTS_DOWNLOAD_URL="https://github.com/cyberbotics/webots/releases/download/R${WEBOTS_VERSION}/webots_${WEBOTS_VERSION}_amd64.deb"

check_internet_connection () {
    if ! ping -q -c 1 -W 1 google.com > /dev/null; then
        echo "No internet connection. Please check your internet connection to install the webots simulator."
        exit 1
    fi
}

# Check if the correct webots simulator WEBOTS_VERSION is installed (apt)
if apt list webots --installed | grep -q "$WEBOTS_VERSION"; then
    echo "Webots simulator release $WEBOTS_VERSION is already installed."
else
    echo "Webots simulator release $WEBOTS_VERSION is not installed. Installing..."
    # Check if we have an internet connection
    check_internet_connection
    # Check if the url exist
    if ! curl --output /dev/null --silent --head --fail "$WEBOTS_DOWNLOAD_URL"; then
        echo "Webots download url does not exist. Please check the url and update the 'WEBOTS_DOWNLOAD_URL' variable in the 'make_webots.sh' script."
        exit 1
    fi
    # Download the webots simulator dep package to temp folder
    wget --no-verbose --show-progress "$WEBOTS_DOWNLOAD_URL" -O "/tmp/webots_${WEBOTS_VERSION}.deb"
    # Install the webots simulator
    sudo apt-get install "/tmp/webots_${WEBOTS_VERSION}.deb" -y

    echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc
    export WEBOTS_HOME=/usr/local/webots
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/noetic/lib/:$WEBOTS_HOME/lib/controller"  >> ~/.bashrc
    echo "export PYTHONPATH=$PYTHONPATH:$WEBOTS_HOME/lib/controller/python38"  >> ~/.bashrc && source ~/.bashrc

fi
