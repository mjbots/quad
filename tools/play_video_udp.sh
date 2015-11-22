#!/bin/bash

cat <<EOF
this script requires sender to be run with option:
  --camera.custom_h264_consumer="udpsink port=29173 host=127.0.0.1"

(substitute proper destination for host)

EOF

PORT="$1"
if [[ "$PORT" == "" ]]; then
    PORT=29173
fi

set -x -e
/opt/gstreamer-1.4.5/bin/gst-launch-1.0 \
    udpsrc port=$PORT ! h264parse \
    ! decodebin ! timeoverlay ! videoconvert ! xvimagesink sync=false
