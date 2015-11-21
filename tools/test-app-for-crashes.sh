#!/bin/bash

APP=mech/build/video_sender_app
ARGS="-v -t camera_driver.stats --max_stats=1 --require_stats_good=1 --camera.device=TEST --rtsp.port=0"
PARALLEL=16
COUNT=$(expr "$1" + 0)

echo Will test $APP with $PARALLEL instances

if ! expr "$COUNT" '>=' 1 >/dev/null; then
    echo Please specify now many runs to test
    exit 1
fi

set -x -e
scons -j4 $APP
rm -fr tafc-out/ *core && mkdir tafc-out
$APP --remote_debug.port=0 $ARGS
if seq -w $COUNT | xargs -i -P$PARALLEL sh -c \
    "$APP --remote_debug.port=0 $ARGS >tafc-out/log.{} 2>&1 \
       || (mv tafc-out/log.{} tafc-out/log.{}.crash; exit 255)"
then
    : All runs were successful
else
    : Some runs crashed
    tail -n 100 tafc-out/log.*.crash
fi
