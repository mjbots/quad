#!/bin/bash

HOST="$1"
if [[ "$HOST" == "" ]]; then
    HOST=127.0.0.1
fi

set -x -e
/opt/gstreamer-1.4.5/bin/gst-launch-1.0 \
    rtspsrc location=rtsp://$HOST:8554/video latency=200 \
    ! decodebin ! timeoverlay ! videoconvert ! xvimagesink sync=false
