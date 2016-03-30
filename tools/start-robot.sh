#!/bin/bash
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    cat <<EOF
A script to run the remote (controller) part of the mjmech system.
Runs script directly on odroid, uses ssh on pc.

Any arguments are passed to a remote side.
EOF
    exit 1
fi

set -e

mach=$(uname -m)

cd $(dirname $(dirname $(readlink -f $0)))

if [[ "$mach" == "armv7l" && "$USER" == "odroid" ]]; then
    # TODO theamk: maybe restart camera
    #  the topology is bus1 (bus2 is OTG port)
    #     -> port3 (external devices, port2 is wired eth)
    #     -> port1 (external; ports2/3 are double connection)
    # if  /sys/bus/usb/devices/1-3.1/*\:1.0/driver
    #   is a symlink to uvcvideo, all is fine; else we need to
    # echo 0 then 1 to /sys/bus/usb/devices/1-3.1/authorized
    CONFIG="-c configs/mw.ini"
    if [[ $(hostname) == "odroid" ]]; then
        # mammal robot
        CONFIG="--video.camera.preset 3 --video.video_link.dest 10.1.10.245"
        CONFIG+=" --config configs/mw_lizard.ini "
    fi
    set -x
    mech/build-armv7l/mech_warfare -t camera_driver.stats \
         $CONFIG "$@"
elif [[ "$mach" == "x86_64" && "$USER" != "odroid" ]]; then
    # upload and run
    set -x
    exec tools/odroid/push-tree.sh --no-push tools/start-robot.sh "$@"
else
    echo cannot determine what kind of machine it is
    exit 1
fi

