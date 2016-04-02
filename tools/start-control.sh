#!/bin/bash
jsname=$(readlink -e \
        /dev/input/by-id/usb-Logitech_Logitech_Dual_Action-event-joystick)
if [[ "$jsname" == "" ]]; then
    echo Fatal: no joystick found
    exit 1
fi
mkdir -p logs
now=$(date +%F-%H%M%S)
set -x

# Use REMOTE env var to set remote IP
: ${REMOTE:=10.89.0.10}

./mech/build-x86_64/mw_command \
    --log logs/mwc-$now.klg -L --display.write_video logs/mwv-$now.mp4  \
    --opt.verbose 1 --joystick $jsname \
    -t video_display.stats --display.stats_interval_s 30 \
    --target $REMOTE \
    --opt.max_translate_x_mm_s 200 --opt.max_translate_y_mm_s 200 \
    "$@"
