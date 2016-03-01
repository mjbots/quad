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
./mech/build-x86_64/mw_command \
    --log logs/mwc-$now.klg -L --display.write_video logs/mwv-$now.mkv  \
    --opt.verbose 1 --joystick $jsname \
    --target 10.89.0.10 \
    --cmd.body_y_mm 10 --opt.max_translate_x_mm_s 200 --opt.max_translate_y_mm_s 200 \
    "$@"
