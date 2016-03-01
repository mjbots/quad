#!/bin/bash
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    cat <<EOF
A script to run the remote (controller) part of the mjmech system.
Runs script directly on odroid, uses ssh on pc.

Any arguments are passed to a remote side, except for the leading:
  -q   (quick) -- do not automatically recompile software
EOF
    exit 1
fi

set -e

mach=$(uname -m)

cd $(dirname $(dirname $(readlink -f $0)))

if [[ "$mach" == "armv7l" && "$USER" == "odroid" ]]; then
    echo Compiling source
    if [[ "$1" == "-q" ]]; then
        shift
    else
        (set -x;
            scons -j2 mech/build-armv7l/mech_warfare)
    fi
    # TODO theamk: maybe restart camera
    #  the topology is bus1 (bus2 is OTG port)
    #     -> port3 (external devices, port2 is wired eth)
    #     -> port1 (external; ports2/3 are double connection)
    # if  /sys/bus/usb/devices/1-3.1/*\:1.0/driver
    #   is a symlink to uvcvideo, all is fine; else we need to
    # echo 0 then 1 to /sys/bus/usb/devices/1-3.1/authorized
    set -x
    mech/build-armv7l/mech_warfare -t camera_driver.stats \
         -c configs/mw.ini "$@"
elif [[ "$mach" == "x86_64" && "$USER" != "odroid" ]]; then
    # make sure we can compile our copy
    if [[ "$1" != "-q" ]]; then
        (set -x;
            scons -j8 mech/build-x86_64/mech_warfare)
    fi
    # Use wlan IP unless specified otherwise
    # to use wires, prefix with: REMOTE=odroid-mjmech
    : ${REMOTE:=10.89.0.10}
    # upload and run
    set -x
    exec tools/odroid/push-tree.sh $REMOTE tools/start-robot.sh "$@"
else
    echo cannot determine what kind of machine it is
    exit 1
fi
