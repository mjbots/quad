#!/bin/bash
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    cat <<EOF
A script to run the remote (controller) part of the mjmech system.
Runs script directly on robot, uses ssh on pc.

Any arguments are passed to a remote side.
EOF
    exit 1
fi

set -e

mach=$(uname -m)

cd $(dirname $(dirname $(readlink -f $0)))

if [[ "$mach" == "armv7l" ]]; then
    CONFIG="-c configs/moteus.ini"
    set -x
    cd ~/mech/
    ./performance_governor.sh
    export LD_LIBRARY_PATH=.
    ./mech_warfare -t camera_driver.stats $CONFIG "$@"
elif [[ "$mach" == "x86_64" ]]; then
    set -x
    ssh pi@192.168.16.47 mech/start-robot.sh "$@"
else
    echo cannot determine what kind of machine it is
    exit 1
fi
