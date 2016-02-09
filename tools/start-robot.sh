#!/bin/bash
# A script to run the remote (controller) part
# runs script directly on odroid, uses ssh on pc
set -e

mach=$(uname -m)

cd $(dirname $(dirname $(readlink -f $0)))

if [[ "$mach" == "armv7l" && "$USER" == "odroid" ]]; then
    echo Compiling source
    set -x
    scons -j2 mech/build-armv7l/video_sender_app
    mech/build-armv7l/video_sender_app \
        -t camera_driver.stats --video_link.dest=239.89.108.10 "$@"
elif [[ "$mach" == "x86_64" && "$USER" != "odroid" ]]; then
    # make sure we can compile our copy
    set -x
    scons -j8 mech/build-x86_64/video_sender_app
    # upload and run
    exec tools/odroid/push-tree.sh odroid-mjmech tools/start-robot.sh "$@"
else
    echo cannot determine what kind of machine it is
    exit 1
fi
