#!/bin/sh
if [ $(uname -m) = x86_64 ]; then
    # This is laptop for vclient.py, install gstreamer from ppa which has
    # working rtp receiver. trusty's stock gstreamer will silently stop
    # receiving data after 1-15 minutes of operation.
    #
    # Known bad version: 1.2.4-1~ubuntu1
    # Known good version: 1.5.0.1+git20140915.711e1
    if ! [ -f /etc/apt/sources.list.d/ricotz-experimental-trusty.list ]; then
        echo Will add gstreamer PPA in 5 seconds
        sleep 5
        (set -ex
            apt-add-repository ppa:ricotz/experimental
            dpkg -r libgstreamer-plugins-good1.0-0 gstreamer1.0-plugins-good \
                gstreamer1.0-plugins-bad || true
            apt-get update
        )
        cat <<EOF
PPA added. Please re-run the script to install all packages.
Note: you may want to run:
   sudo apt-get upgrade

Warning: possible upgrade package bug. If after running the script you get an
error:
 dpkg: error processing archive /var/cache/apt/archives/libgstreamer-plugins-bad1.0-0_1.5.0.1+git20140916.ef8bf751-0ubuntu1~14.04~ricotz0_amd64.deb (--unpack):
 trying to overwrite '/usr/lib/x86_64-linux-gnu/libgstbasecamerabinsrc-1.0.so.0', which is also in package libgstreamer-plugins-good1.0-0:amd64 1.2.4-1~ubuntu1
fix it by running:
  sudo dpkg -r libgstreamer-plugins-good1.0-0
  sudo apt-get -f install
EOF
        echo Run the script again to install packages
        exit 0
    fi
fi

set -ex
apt-get install python-eventlet python-pyside pyside-tools python-pygame \
    python-pip scons avr-libc \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    python-opencv
pip install --pre pygazebo
pip install --pre trollius
pip install enum34
