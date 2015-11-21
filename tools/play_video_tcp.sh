#!/bin/bash

# this script requires sender to be run with option:
#  --camera.custom_h264_consumer="tcpserversink port=29173"

HOST="$1"
PORT="$2"
if [[ "$HOST" == "" ]]; then
    HOST=127.0.0.1
fi
if [[ "$PORT" == "" ]]; then
    PORT=29173
fi

set -x -e
/opt/gstreamer-1.4.5/bin/gst-launch-1.0 \
    tcpclientsrc host=$HOST port=$PORT ! h264parse \
    ! decodebin ! timeoverlay ! videoconvert ! xvimagesink sync=false
