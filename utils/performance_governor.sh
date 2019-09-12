#!/bin/bash

set -e

for a in /sys/devices/system/cpu/cpu?/cpufreq/scaling_governor; do
    echo "performance" > $a
done
